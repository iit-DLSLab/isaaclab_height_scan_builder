#include "height_scan_builder.h"

#include <functional>
#include <limits>
#include <utility>
#include <vector>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/exceptions.h>
#include <Eigen/Geometry>

HeightScanBuilder::HeightScanBuilder(const rclcpp::NodeOptions & options)
: rclcpp::Node("height_scan_builder", options),
    _cloud(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>()),
    _xy_cloud(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>())
{
    const auto cloudTopic = this->declare_parameter<std::string>(
        "cloud_topic", "/filtered");
    const auto socketAddr = this->declare_parameter<std::string>(
        "socket_addr", "tcp://localhost:5005");
    const auto scanCloudTopic = this->declare_parameter<std::string>(
        "scan_cloud_topic", "height_scan_cloud");
    const auto markerTopic = this->declare_parameter<std::string>(
        "marker_topic", "height_scan_markers");
    this->_publish_markers = this->declare_parameter<bool>(
        "publish_markers", true);

    this->_height_scan_cfg.width = this->declare_parameter<double>("width", 1.0);
    this->_height_scan_cfg.height = this->declare_parameter<double>("height", 1.6);
    this->_height_scan_cfg.resolution = this->declare_parameter<double>("resolution", 0.1);
    this->_height_scan_cfg.offset.x = this->declare_parameter<double>("offset_x", 0.0);
    this->_height_scan_cfg.offset.y = this->declare_parameter<double>("offset_y", 0.0);
    this->_target_frame = this->declare_parameter<std::string>("target_frame", "base_link");
    this->_loop_rate_hz = this->declare_parameter<double>("loop_rate_hz", 30.0);

    this->_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->_tf_listener = std::make_shared<tf2_ros::TransformListener>(*this->_tf_buffer);

    this->_context = std::make_unique<zmq::context_t>(1);
    this->_socket = std::make_unique<zmq::socket_t>(*this->_context, ZMQ_PUB);
    this->_socket->connect(socketAddr);

    this->_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        scanCloudTopic,
        rclcpp::SensorDataQoS());

    if (this->_publish_markers)
    {
        this->_marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            markerTopic,
            1);
    }

    this->_cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloudTopic,
        rclcpp::SensorDataQoS(),
        std::bind(&HeightScanBuilder::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
        this->get_logger(),
        "Listening on %s and publishing scans over %s and %s at %.2f Hz",
        cloudTopic.c_str(),
        socketAddr.c_str(),
        scanCloudTopic.c_str(),
        this->_loop_rate_hz);
    RCLCPP_INFO(
        this->get_logger(),
        "MarkerArray publisher is %s",
        this->_publish_markers ? "enabled" : "disabled");
}

double HeightScanBuilder::loopRateHz() const
{
    return this->_loop_rate_hz;
}

void HeightScanBuilder::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    auto sourceCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto xyCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    pcl::fromROSMsg(*msg, *sourceCloud);

    std::int64_t stampNs = rclcpp::Time(msg->header.stamp).nanoseconds();
    try
    {
        const auto transform = this->_tf_buffer->lookupTransform(
            this->_target_frame,
            msg->header.frame_id,
            msg->header.stamp,
            rclcpp::Duration::from_seconds(0.05));

        const auto & t = transform.transform.translation;
        const auto & r = transform.transform.rotation;
        const Eigen::Translation3f translation(
            static_cast<float>(t.x),
            static_cast<float>(t.y),
            static_cast<float>(t.z));
        const Eigen::Quaternionf rotation(
            static_cast<float>(r.w),
            static_cast<float>(r.x),
            static_cast<float>(r.y),
            static_cast<float>(r.z));
        const Eigen::Affine3f eigenTransform = translation * rotation;

        pcl::transformPointCloud(*sourceCloud, *cloud, eigenTransform);
    }
    catch (const tf2::TransformException & ex)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Failed to transform cloud from '%s' to '%s': %s",
            msg->header.frame_id.c_str(),
            this->_target_frame.c_str(),
            ex.what());
        return;
    }

    xyCloud->reserve(cloud->size());

    for (const auto & point : cloud->points)
    {
        xyCloud->push_back(pcl::PointXYZ(point.x, point.y, 0.0F));
    }

    {
        std::lock_guard<std::mutex> lock(this->_cloud_mutex);
        this->_cloud = std::move(cloud);
        this->_xy_cloud = std::move(xyCloud);
        this->_stamp_ns = stampNs;
        this->_frame_id = this->_target_frame;
    }

    // Build and publish right after receiving a cloud to reduce scheduling latency.
    this->buildHeightScan();
}

void HeightScanBuilder::buildHeightScan()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyCloud;
    std::int64_t stampNs = 0;

    {
        std::lock_guard<std::mutex> lock(this->_cloud_mutex);

        if (!this->_cloud || this->_cloud->empty() || !this->_xy_cloud || this->_xy_cloud->empty())
        {
            return;
        }

        cloud = this->_cloud;
        xyCloud = this->_xy_cloud;
        stampNs = this->_stamp_ns;
    }

    HeightScanMsg scan;
    scan.set_seq(this->_seq++);
    scan.set_stamp(stampNs);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(xyCloud);

    for (double y = this->_height_scan_cfg.offset.y - this->_height_scan_cfg.width / 2; y <= this->_height_scan_cfg.offset.y + this->_height_scan_cfg.width / 2; y += this->_height_scan_cfg.resolution)
    {
        for (double x = this->_height_scan_cfg.offset.x - this->_height_scan_cfg.height / 2; x <= this->_height_scan_cfg.offset.x + this->_height_scan_cfg.height / 2; x += this->_height_scan_cfg.resolution)
        {
            pcl::PointXYZ searchPoint(x, y, 0.0F);
            std::vector<int> indices;
            std::vector<float> squaredDistances;

            if (kdtree.radiusSearch(
                    searchPoint,
                    this->_height_scan_cfg.resolution / 2,
                    indices,
                    squaredDistances) == 0)
            {
                // No point found: average the max-z of the 4 neighbouring grid positions
                const double res = this->_height_scan_cfg.resolution;
                const std::array<std::pair<double,double>, 4> neighbours = {{
                    {x - res, y}, {x + res, y}, {x, y - res}, {x, y + res}
                }};
                float zSum = 0.0f;
                int zCount = 0;
                for (const auto & [nx, ny] : neighbours)
                {
                    std::vector<int> nIndices;
                    std::vector<float> nDistances;
                    if (kdtree.radiusSearch(
                            pcl::PointXYZ(nx, ny, 0.0F),
                            res / 2,
                            nIndices,
                            nDistances) > 0)
                    {
                        float nMaxZ = -std::numeric_limits<float>::infinity();
                        for (const int nIdx : nIndices)
                        {
                            if (cloud->points[nIdx].z > nMaxZ)
                                nMaxZ = cloud->points[nIdx].z;
                        }
                        zSum += nMaxZ;
                        ++zCount;
                    }
                }
                PointScan * pointScan = scan.add_points();
                pointScan->set_x(x);
                pointScan->set_y(y);
                pointScan->set_z(zCount > 0 ? zSum / zCount : 0.0f);
                continue;
            }

            int maxZIdx = -1;
            float maxZ = -std::numeric_limits<float>::infinity();

            for (const int idx : indices)
            {
                if (cloud->points[idx].z > maxZ)
                {
                    maxZ = cloud->points[idx].z;
                    maxZIdx = idx;
                }
            }

            if (maxZIdx < 0)
            {
                PointScan * pointScan = scan.add_points();
                pointScan->set_x(x);
                pointScan->set_y(y);
                pointScan->set_z(5.0); // No point found, set to a default high value
            }
            else
            {
                PointScan * pointScan = scan.add_points();
                pointScan->set_x(x);
                pointScan->set_y(y);
                pointScan->set_z(cloud->points[maxZIdx].z);
            }
        }
    }
    this->publishScan(scan);
    this->publishPointCloud(scan);

    if (this->_publish_markers)
    {
        this->publishMarkers(scan);
    }
}

void HeightScanBuilder::publishPointCloud(const HeightScanMsg & scan)
{
    pcl::PointCloud<pcl::PointXYZ> scanCloud;
    scanCloud.reserve(static_cast<std::size_t>(scan.points_size()));

    for (int i = 0; i < scan.points_size(); ++i)
    {
        const auto & pt = scan.points(i);
        scanCloud.push_back(pcl::PointXYZ(
            static_cast<float>(pt.x()),
            static_cast<float>(pt.y()),
            static_cast<float>(pt.z())));
    }

    sensor_msgs::msg::PointCloud2 cloudMsg;
    pcl::toROSMsg(scanCloud, cloudMsg);
    cloudMsg.header.frame_id = this->_frame_id;
    cloudMsg.header.stamp = rclcpp::Time(scan.stamp());

    this->_cloud_pub->publish(cloudMsg);
}

void HeightScanBuilder::publishMarkers(const HeightScanMsg & scan)
{
    visualization_msgs::msg::MarkerArray markerArray;

    // Delete all previous markers first
    visualization_msgs::msg::Marker deleteAll;
    deleteAll.action = visualization_msgs::msg::Marker::DELETEALL;
    markerArray.markers.push_back(deleteAll);

    const auto stamp = rclcpp::Time(scan.stamp());

    for (int i = 0; i < scan.points_size(); ++i)
    {
        const auto & pt = scan.points(i);
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = this->_frame_id;
        marker.header.stamp = stamp;
        marker.ns = "height_scan";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = pt.x();
        marker.pose.position.y = pt.y();
        marker.pose.position.z = pt.z();
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
        markerArray.markers.push_back(marker);
    }

    this->_marker_pub->publish(markerArray);
}

void HeightScanBuilder::publishScan(const HeightScanMsg & scan)
{
    std::string out;
    scan.SerializeToString(&out);
    this->_socket->send(zmq::buffer(out), zmq::send_flags::none);
}
