#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

struct LidarPoint
{
    Eigen::Vector3d xyz;
    float intensity;
    std::uint16_t ring;
    float time;
};

class LidarExtrinsicCalibrator : public rclcpp::Node
{
public:
    using CloudMsg = sensor_msgs::msg::PointCloud2;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<CloudMsg, CloudMsg>;

    LidarExtrinsicCalibrator()
    : rclcpp::Node("lidar_extrinsic_calibrator_node")
    {
        this->_time_tolerance = this->declare_parameter<double>("time_tolerance", 1e-6);

        const auto qos = rclcpp::SensorDataQoS();
        this->_source_sub.subscribe(
            this, "/utlidar/cloud", qos.get_rmw_qos_profile());
        this->_target_sub.subscribe(
            this, "/utlidar/cloud_base", qos.get_rmw_qos_profile());

        this->_sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(20), this->_source_sub, this->_target_sub);
        this->_sync->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.01));
        this->_sync->registerCallback(
            &LidarExtrinsicCalibrator::cloudCallback, this);

        RCLCPP_INFO(
            this->get_logger(),
            "Matching /utlidar/cloud to /utlidar/cloud_base by ring/time (tolerance %.9f s)",
            this->_time_tolerance);
    }

private:
    bool readCloud(const CloudMsg & msg, std::vector<LidarPoint> & points) const
    {
        try
        {
            sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> y(msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> z(msg, "z");
            sensor_msgs::PointCloud2ConstIterator<float> intensity(msg, "intensity");
            sensor_msgs::PointCloud2ConstIterator<std::uint16_t> ring(msg, "ring");
            sensor_msgs::PointCloud2ConstIterator<float> time(msg, "time");

            points.reserve(static_cast<std::size_t>(msg.width) * msg.height);
            for (; x != x.end(); ++x, ++y, ++z, ++intensity, ++ring, ++time)
            {
                points.push_back({
                    Eigen::Vector3d(*x, *y, *z), *intensity, *ring, *time});
            }
        }
        catch (const std::runtime_error & ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to read PointCloud2 fields: %s", ex.what());
            return false;
        }

        return true;
    }

    void cloudCallback(
        const CloudMsg::ConstSharedPtr & sourceMsg,
        const CloudMsg::ConstSharedPtr & targetMsg)
    {
        std::vector<LidarPoint> sourcePoints;
        std::vector<LidarPoint> targetPoints;
        if (!this->readCloud(*sourceMsg, sourcePoints) ||
            !this->readCloud(*targetMsg, targetPoints))
        {
            return;
        }

        std::vector<Eigen::Vector3d> matchedSource;
        std::vector<Eigen::Vector3d> matchedTarget;
        std::vector<bool> sourceUsed(sourcePoints.size(), false);
        matchedSource.reserve(targetPoints.size());
        matchedTarget.reserve(targetPoints.size());

        constexpr std::size_t examplesToPrint = 3;
        std::size_t examplesPrinted = 0;

        for (const auto & target : targetPoints)
        {
            if (!target.xyz.allFinite() || !std::isfinite(target.time))
            {
                continue;
            }

            std::size_t bestIndex = sourcePoints.size();
            double bestTimeDifference = this->_time_tolerance;
            double bestIntensityDifference = std::numeric_limits<double>::infinity();

            for (std::size_t i = 0; i < sourcePoints.size(); ++i)
            {
                const auto & source = sourcePoints[i];
                if (sourceUsed[i] || source.ring != target.ring ||
                    !source.xyz.allFinite() || !std::isfinite(source.time))
                {
                    continue;
                }

                const double timeDifference = std::abs(
                    static_cast<double>(source.time) - target.time);
                if (timeDifference > this->_time_tolerance)
                {
                    continue;
                }

                const double intensityDifference = std::abs(
                    static_cast<double>(source.intensity) - target.intensity);
                if (bestIndex == sourcePoints.size() ||
                    timeDifference < bestTimeDifference ||
                    (timeDifference == bestTimeDifference &&
                     intensityDifference < bestIntensityDifference))
                {
                    bestIndex = i;
                    bestTimeDifference = timeDifference;
                    bestIntensityDifference = intensityDifference;
                }
            }

            if (bestIndex == sourcePoints.size())
            {
                continue;
            }

            const auto & source = sourcePoints[bestIndex];
            sourceUsed[bestIndex] = true;
            matchedSource.push_back(source.xyz);
            matchedTarget.push_back(target.xyz);

            if (examplesPrinted < examplesToPrint)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Example match %zu raw: xyz=(%.6f, %.6f, %.6f), ring=%u, time=%.9f, intensity=%.6f",
                    examplesPrinted + 1,
                    source.xyz.x(), source.xyz.y(), source.xyz.z(),
                    static_cast<unsigned>(source.ring), source.time, source.intensity);
                RCLCPP_INFO(
                    this->get_logger(),
                    "Example match %zu base: xyz=(%.6f, %.6f, %.6f), ring=%u, time=%.9f, intensity=%.6f",
                    examplesPrinted + 1,
                    target.xyz.x(), target.xyz.y(), target.xyz.z(),
                    static_cast<unsigned>(target.ring), target.time, target.intensity);
                ++examplesPrinted;
            }
        }

        const double matchedPercentage = targetPoints.empty()
            ? 0.0
            : 100.0 * static_cast<double>(matchedSource.size()) / targetPoints.size();

        RCLCPP_INFO(
            this->get_logger(),
            "Points: raw=%zu, cloud_base=%zu, matched=%zu (%.2f%% of cloud_base)",
            sourcePoints.size(),
            targetPoints.size(),
            matchedSource.size(),
            matchedPercentage);

        if (matchedSource.size() < 20)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Need at least 20 correspondences; skipping this frame");
            return;
        }

        Eigen::Vector3d sourceCentroid = Eigen::Vector3d::Zero();
        Eigen::Vector3d targetCentroid = Eigen::Vector3d::Zero();
        for (std::size_t i = 0; i < matchedSource.size(); ++i)
        {
            sourceCentroid += matchedSource[i];
            targetCentroid += matchedTarget[i];
        }
        sourceCentroid /= static_cast<double>(matchedSource.size());
        targetCentroid /= static_cast<double>(matchedTarget.size());

        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
        for (std::size_t i = 0; i < matchedSource.size(); ++i)
        {
            covariance += (matchedSource[i] - sourceCentroid) *
                (matchedTarget[i] - targetCentroid).transpose();
        }

        const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
            covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d v = svd.matrixV();
        Eigen::Matrix3d rotation = v * svd.matrixU().transpose();
        if (rotation.determinant() < 0.0)
        {
            v.col(2) *= -1.0;
            rotation = v * svd.matrixU().transpose();
        }
        const Eigen::Vector3d translation = targetCentroid - rotation * sourceCentroid;

        double errorSum = 0.0;
        double squaredErrorSum = 0.0;
        double maximumError = 0.0;
        for (std::size_t i = 0; i < matchedSource.size(); ++i)
        {
            const double error =
                (rotation * matchedSource[i] + translation - matchedTarget[i]).norm();
            errorSum += error;
            squaredErrorSum += error * error;
            maximumError = std::max(maximumError, error);
        }

        const double meanError = errorSum / matchedSource.size();
        const double rmse = std::sqrt(squaredErrorSum / matchedSource.size());
        RCLCPP_INFO(
            this->get_logger(),
            "Residual [m]: mean=%.9f, RMSE=%.9f, max=%.9f",
            meanError, rmse, maximumError);

        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        transform.block<3, 3>(0, 0) = rotation;
        transform.block<3, 1>(0, 3) = translation;
        this->printTransform(transform);
    }

    void printTransform(const Eigen::Matrix4d & transform) const
    {
        const Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
        const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
        const double pitch = std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));
        const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
        constexpr double radiansToDegrees = 180.0 / 3.14159265358979323846;

        std::ostringstream matrix;
        matrix << std::fixed << std::setprecision(9) << transform;
        RCLCPP_INFO(
            this->get_logger(),
            "T_base_link_utlidar_lidar:\n%s",
            matrix.str().c_str());
        RCLCPP_INFO(
            this->get_logger(),
            "translation [m]: x=%.9f y=%.9f z=%.9f",
            transform(0, 3), transform(1, 3), transform(2, 3));
        RCLCPP_INFO(
            this->get_logger(),
            "RPY [rad]: roll=%.9f pitch=%.9f yaw=%.9f",
            roll, pitch, yaw);
        RCLCPP_INFO(
            this->get_logger(),
            "RPY [deg]: roll=%.6f pitch=%.6f yaw=%.6f",
            roll * radiansToDegrees,
            pitch * radiansToDegrees,
            yaw * radiansToDegrees);
        RCLCPP_INFO(
            this->get_logger(),
            "Ready-to-copy command:\n"
            "ros2 run tf2_ros static_transform_publisher "
            "--x %.9f --y %.9f --z %.9f "
            "--roll %.9f --pitch %.9f --yaw %.9f "
            "--frame-id base_link --child-frame-id utlidar_lidar",
            transform(0, 3), transform(1, 3), transform(2, 3),
            roll, pitch, yaw);
    }

    message_filters::Subscriber<CloudMsg> _source_sub;
    message_filters::Subscriber<CloudMsg> _target_sub;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> _sync;
    double _time_tolerance{1e-6};
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarExtrinsicCalibrator>());
    rclcpp::shutdown();
    return 0;
}
