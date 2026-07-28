#include "point_cloud_manager.h"

#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

namespace isaaclab
{

PointCloudManagerNode::PointCloudManagerNode(const rclcpp::NodeOptions & options)
: Node("point_cloud_manager", options)
{
  declare_parameter<std::string>("topic1", "");
  declare_parameter<std::string>("topic2", "");
  declare_parameter<std::string>("output_topic", "/cloud_merged");
  declare_parameter<std::string>("filter_frame", "base_link");
  declare_parameter<std::string>("target_frame", "base_link");
  declare_parameter<float>("voxel_leaf_size", 0.05f);
  declare_parameter<float>("box_x", 2.0f);
  declare_parameter<float>("box_y", 1.5f);
  declare_parameter<int>("sync_queue_size", 10);
  declare_parameter<double>("sync_slop", 0.1);

  const auto topic1      = get_parameter("topic1").as_string();
  const auto topic2      = get_parameter("topic2").as_string();
  std::string out_topic  = get_parameter("output_topic").as_string();
  filter_frame_          = get_parameter("filter_frame").as_string();
  target_frame_          = get_parameter("target_frame").as_string();
  voxel_leaf_size_       = static_cast<float>(get_parameter("voxel_leaf_size").as_double());
  box_x_                 = static_cast<float>(get_parameter("box_x").as_double());
  box_y_                 = static_cast<float>(get_parameter("box_y").as_double());
  const int queue_size   = get_parameter("sync_queue_size").as_int();
  const double slop      = get_parameter("sync_slop").as_double();

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  auto qos = rclcpp::SensorDataQoS();

  if (topic1.empty()) {
    RCLCPP_ERROR(get_logger(), "Parameter 'topic1' must be a non-empty topic name.");
    throw std::runtime_error("Invalid parameter: topic1 is empty");
  }

  if (!topic2.empty()) {
    std::cout << "SUBSCRIBING TO TWO TOPICS: " << topic1 << " AND " << topic2 << std::endl;
    sub1_.subscribe(this, topic1, qos.get_rmw_qos_profile());
    sub2_.subscribe(this, topic2, qos.get_rmw_qos_profile());

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(queue_size), sub1_, sub2_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(slop));
    sync_->registerCallback(&PointCloudManagerNode::syncCallback, this);
  } else {
    std::cout << "SUBSCRIBING TO SINGLE TOPIC: " << topic1 << std::endl;
    single_sub_ = create_subscription<Cloud2>(
      topic1, qos,
      std::bind(&PointCloudManagerNode::singleTopicCallback, this, std::placeholders::_1));
  }

  pub_ = create_publisher<Cloud2>(out_topic, rclcpp::SensorDataQoS());

  RCLCPP_INFO(
    get_logger(), "PointCloudManagerNode ready (%s).",
    topic2.empty() ? "single-topic mode" : "ApproximateTimeSynchronizer");
  RCLCPP_INFO(get_logger(), "  topic1: %s", topic1.c_str());
  RCLCPP_INFO(get_logger(), "  topic2: %s", topic2.c_str());
  RCLCPP_INFO(get_logger(), "  output: %s", out_topic.c_str());
  RCLCPP_INFO(get_logger(), "  filter_frame: %s", filter_frame_.c_str());
  RCLCPP_INFO(get_logger(), "  target_frame: %s", target_frame_.c_str());
  RCLCPP_INFO(get_logger(), "  voxel_leaf: %.3f m", voxel_leaf_size_);
  RCLCPP_INFO(get_logger(), "  box_x: %.2f m  box_y: %.2f m", box_x_, box_y_);
  if (!topic2.empty()) {
    RCLCPP_INFO(get_logger(), "  sync slop: %.3f s  queue: %d", slop, queue_size);
  }
}

void PointCloudManagerNode::singleTopicCallback(const Cloud2::ConstSharedPtr & msg)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (!transformCloud(msg, filter_frame_, cloud)) {
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> filtered;
  filterCloud(cloud, filtered);

  Cloud2 filtered_msg;
  pcl::toROSMsg(filtered, filtered_msg);
  filtered_msg.header.frame_id = filter_frame_;
  filtered_msg.header.stamp = msg->header.stamp;

  if (filter_frame_ == target_frame_) {
    pub_->publish(filtered_msg);
    return;
  }

  Cloud2 out_msg;
  if (!transformCloud(filtered_msg, target_frame_, out_msg)) {
    return;
  }
  out_msg.header.stamp = msg->header.stamp;
  pub_->publish(out_msg);
}

bool PointCloudManagerNode::transformCloud(
  const Cloud2::ConstSharedPtr & msg,
  const std::string & to_frame,
  pcl::PointCloud<pcl::PointXYZ> & out)
{
  sensor_msgs::msg::PointCloud2 transformed_msg;
  if (!transformCloud(*msg, to_frame, transformed_msg)) {
    return false;
  }
  pcl::fromROSMsg(transformed_msg, out);
  return true;
}

bool PointCloudManagerNode::transformCloud(
  const Cloud2 & in,
  const std::string & to_frame,
  Cloud2 & out)
{
  try {
    const auto tf = tf_buffer_->lookupTransform(
      to_frame, in.header.frame_id,
      rclcpp::Time(0, 0, get_clock()->get_clock_type()),
      rclcpp::Duration(std::chrono::milliseconds(100)));
    tf2::doTransform(in, out, tf);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Transform from '%s' to '%s' failed: %s", in.header.frame_id.c_str(), to_frame.c_str(), ex.what());
    return false;
  }

  return true;
}

void PointCloudManagerNode::syncCallback(
  const Cloud2::ConstSharedPtr & msg1,
  const Cloud2::ConstSharedPtr & msg2)
{
  // --- Transform both clouds to filter_frame ---
  pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
  if (!transformCloud(msg1, filter_frame_, cloud1) || !transformCloud(msg2, filter_frame_, cloud2)) {
    return;
  }

  // --- Merge ---
  pcl::PointCloud<pcl::PointXYZ> merged;
  merged = cloud1;
  merged += cloud2;

  // Filter cloud
  pcl::PointCloud<pcl::PointXYZ> filtered;
  filterCloud(merged, filtered);

  // --- Publish with synchronized stamp ---
  Cloud2 filtered_msg;
  pcl::toROSMsg(filtered, filtered_msg);
  filtered_msg.header.frame_id = filter_frame_;
  filtered_msg.header.stamp    = msg1->header.stamp;  // synchronized timestamp

  if (filter_frame_ == target_frame_) {
    pub_->publish(filtered_msg);
    return;
  }

  Cloud2 out_msg;
  if (!transformCloud(filtered_msg, target_frame_, out_msg)) {
    return;
  }
  out_msg.header.stamp = msg1->header.stamp;
  pub_->publish(out_msg);
}

void PointCloudManagerNode::filterCloud(
  const pcl::PointCloud<pcl::PointXYZ> & in,
  pcl::PointCloud<pcl::PointXYZ> & out) const
{
  // --- Box filter: keep points inside [-box_x/2, box_x/2] x [-box_y/2, box_y/2] ---
  {
    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setInputCloud(in.makeShared());
    crop.setMin(Eigen::Vector4f(-box_x_ / 2.0f, -box_y_ / 2.0f, -std::numeric_limits<float>::max(), 1.0f));
    crop.setMax(Eigen::Vector4f( box_x_ / 2.0f,  box_y_ / 2.0f,  std::numeric_limits<float>::max(), 1.0f));
    crop.filter(out);
  }

  // --- Voxel downsampling ---
  {
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(out.makeShared());
    voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel.filter(out);
  }

}

}  // namespace isaaclab

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<isaaclab::PointCloudManagerNode>());
  rclcpp::shutdown();
  return 0;
}
