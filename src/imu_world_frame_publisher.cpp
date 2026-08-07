#include "imu_world_frame_publisher.h"

#include <chrono>
#include <functional>
#include <stdexcept>
#include <thread>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


namespace isaaclab
{

ImuWorldFramePublisherNode::ImuWorldFramePublisherNode(const rclcpp::NodeOptions & options)
: Node("imu_world_frame_publisher", options)
{
  declare_parameter<std::string>("imu_topic", "/imu");
  declare_parameter<std::string>("cloud_topic", "/utlidar/cloud_base");
  declare_parameter<std::string>("world_frame", "world");
  declare_parameter<std::string>("base_link", "pelvis");
  declare_parameter<std::string>("imu_frame", "imu_link");

  imu_topic_ = get_parameter("imu_topic").as_string();
  cloud_topic_ = get_parameter("cloud_topic").as_string();
  world_frame_ = get_parameter("world_frame").as_string();
  base_link_ = get_parameter("base_link").as_string();
  imu_frame_ = get_parameter("imu_frame").as_string();

  if (imu_topic_.empty()) {
    throw std::runtime_error("Parameter 'imu_topic' must be non-empty");
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // Initialize q_w_imu_ref from static transform base_link <- imu_frame
  for(;;)
  {
    try {
      const auto tf_base_imu = tf_buffer_->lookupTransform(
        base_link_, imu_frame_,
        rclcpp::Time(0, 0, get_clock()->get_clock_type()),
        rclcpp::Duration(std::chrono::milliseconds(5000)));
      const auto & q = tf_base_imu.transform.rotation;
      const Eigen::Quaterniond b_q_imu(q.w, q.x, q.y, q.z);
      
      // Apply RPY offset (π/2, 0, 0) to the reference orientation
      // const Eigen::AngleAxisd offset_angle_axis(M_PI, Eigen::Vector3d::UnitX());
      // const Eigen::Quaterniond q_offset(offset_angle_axis);
      
      q_w_imu_ref_ = b_q_imu; //* q_offset).normalized();
      has_reference_orientation_ = true;
      break;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(get_logger(), "Failed to get %s <- %s transform: %s", base_link_.c_str(), imu_frame_.c_str(), ex.what());
    }
  }

  // The static lookup is only needed during construction.
  tf_listener_.reset();
  tf_buffer_.reset();

  auto qos = rclcpp::SensorDataQoS();
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, qos,
    std::bind(&ImuWorldFramePublisherNode::imuCallback, this, std::placeholders::_1));
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic_, qos,
    std::bind(&ImuWorldFramePublisherNode::cloudCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "ImuWorldFramePublisherNode ready");
  RCLCPP_INFO(get_logger(), "  imu_topic: %s", imu_topic_.c_str());
  RCLCPP_INFO(get_logger(), "  cloud_topic: %s", cloud_topic_.c_str());
  RCLCPP_INFO(get_logger(), "  world_frame: %s", world_frame_.c_str());
  RCLCPP_INFO(get_logger(), "  base_link: %s", base_link_.c_str());
  RCLCPP_INFO(get_logger(), "  imu_frame: %s", imu_frame_.c_str());
}

void ImuWorldFramePublisherNode::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr & msg)
{
  const auto & q_msg = msg->orientation;
  const Eigen::Quaterniond w_q_imu(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
  if (w_q_imu.norm() < 1e-8) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "IMU orientation is near zero quaternion");
    return;
  }

  const Eigen::Quaterniond imu_read = w_q_imu.normalized();

  // Delta rotation from IMU relative to static reference transform.
  // b_q_imu * w_q_imu
  // q_delta_latest_ = (q_w_imu_ref_ * imu_read).normalized();
  // w_R_base = w_R_imu * base_R_imu.T()
  q_delta_latest_ = (imu_read * q_w_imu_ref_.inverse()).normalized();
  has_latest_orientation_ = true;
}

void ImuWorldFramePublisherNode::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  if (!has_latest_orientation_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Received pointcloud before any valid IMU orientation");
    return;
  }

  publishTransform(
    rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type()), q_delta_latest_);
}

void ImuWorldFramePublisherNode::publishTransform(
  const rclcpp::Time & stamp, const Eigen::Quaterniond & q_delta)
{
  // Remove the yaw component: decompose q_delta = q_yaw * q_rp, keep only q_rp.
  const double yaw = std::atan2(
    2.0 * (q_delta.w() * q_delta.z() + q_delta.x() * q_delta.y()),
    1.0 - 2.0 * (q_delta.y() * q_delta.y() + q_delta.z() * q_delta.z()));
  const Eigen::Quaterniond q_yaw(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  const Eigen::Quaterniond q_rp = (q_yaw.inverse() * q_delta).normalized();

  tf2::Quaternion tf_q(q_rp.x(), q_rp.y(), q_rp.z(), q_rp.w());
  tf_q.normalize();

  geometry_msgs::msg::TransformStamped tf_world_pelvis;
  tf_world_pelvis.header.stamp = stamp;
  tf_world_pelvis.header.frame_id = world_frame_;
  tf_world_pelvis.child_frame_id = base_link_;
  tf_world_pelvis.transform.translation.x = 0.0;
  tf_world_pelvis.transform.translation.y = 0.0;
  tf_world_pelvis.transform.translation.z = 0.0;
  tf_world_pelvis.transform.rotation = tf2::toMsg(tf_q);

  tf_broadcaster_->sendTransform(tf_world_pelvis);
}

}  // namespace isaaclab

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<isaaclab::ImuWorldFramePublisherNode>());
  rclcpp::shutdown();
  return 0;
}
