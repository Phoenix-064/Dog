#pragma once

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <memory>
#include <string>

namespace dog_perception
{

class PerceptionNode : public rclcpp::Node
{
public:
  explicit PerceptionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct CameraExtrinsics
  {
    std::string frame_id;
    std::string child_frame_id;
    double x;
    double y;
    double z;
    double qx;
    double qy;
    double qz;
    double qw;
  };

  static std::string getDefaultExtrinsicsYamlPath();
  CameraExtrinsics loadExtrinsicsFromYaml(const std::string & yaml_path) const;
  void initializeStaticTransform();
  void lidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  std::string lidar_topic_;
  std::string extrinsics_yaml_path_;
  CameraExtrinsics camera_extrinsics_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
};

}  // namespace dog_perception