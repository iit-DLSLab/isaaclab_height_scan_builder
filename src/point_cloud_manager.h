#pragma once

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace isaaclab
{

using Cloud2 = sensor_msgs::msg::PointCloud2;
using SyncPolicy = message_filters::sync_policies::ApproximateTime<Cloud2, Cloud2>;

class PointCloudManagerNode : public rclcpp::Node
{
public:
  explicit PointCloudManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void singleTopicCallback(const Cloud2::ConstSharedPtr & msg);

  void syncCallback(
    const Cloud2::ConstSharedPtr & msg1,
    const Cloud2::ConstSharedPtr & msg2);

  bool transformCloud(
    const Cloud2::ConstSharedPtr & msg,
    const std::string & to_frame,
    pcl::PointCloud<pcl::PointXYZ> & out);

  bool transformCloud(
    const Cloud2 & in,
    const std::string & to_frame,
    Cloud2 & out);

  void filterCloud(
    const pcl::PointCloud<pcl::PointXYZ> & in,
    pcl::PointCloud<pcl::PointXYZ> & out) const;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Single-topic subscription
  rclcpp::Subscription<Cloud2>::SharedPtr single_sub_;

  // Synchronized subscriptions
  message_filters::Subscriber<Cloud2> sub1_;
  message_filters::Subscriber<Cloud2> sub2_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // Publisher
  rclcpp::Publisher<Cloud2>::SharedPtr pub_;

  // Parameters
  std::string filter_frame_;
  std::string target_frame_;
  float voxel_leaf_size_;
  float box_x_;
  float box_y_;
};

}  // namespace isaaclab
