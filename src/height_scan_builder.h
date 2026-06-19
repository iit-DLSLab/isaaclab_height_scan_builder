#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <zmq.hpp>

#include "height_scan_msg.pb.h"

struct HeightScanCfg
{
    double width = 1.0;
    double height = 1.6;
    double resolution = 0.1;
};

class HeightScanBuilder : public rclcpp::Node
{
public:
    explicit HeightScanBuilder(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    void buildHeightScan();
    void publishScan(const HeightScanMsg & scan);
    void publishPointCloud(const HeightScanMsg & scan);
    void publishMarkers(const HeightScanMsg & scan);
    double loopRateHz() const;

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _xy_cloud;
    std::unique_ptr<zmq::context_t> _context;
    std::unique_ptr<zmq::socket_t> _socket;
    std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _cloud_sub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _marker_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _cloud_pub;
    mutable std::mutex _cloud_mutex;

    HeightScanCfg _height_scan_cfg;
    std::string _frame_id;
    std::int64_t _stamp_ns{0};
    std::int32_t _seq{0};
    std::string _target_frame{"base_link"};
    double _loop_rate_hz{10.0};
    bool _publish_markers{true};
};
