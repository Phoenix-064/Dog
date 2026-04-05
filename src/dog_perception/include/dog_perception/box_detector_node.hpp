#pragma once

#include <dog_interfaces/msg/target3_d.hpp>
#include <dog_interfaces/msg/target3_d_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/dnn.hpp>

#include <string>
#include <vector>

namespace dog_perception
{

class BoxDetectorNode : public rclcpp::Node
{
public:
  /// @brief 构造箱体类型识别节点。
  /// @param options ROS 节点选项。
  explicit BoxDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct Detection
  {
    int class_id;
    float confidence;
    int x;
    int y;
    int width;
    int height;
  };

  /// @brief 处理输入图像并发布箱体识别结果。
  /// @param image_msg 图像消息。
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg);
  /// @brief 按需加载 YOLO 模型。
  /// @return 成功加载并可推理时返回 true。
  bool ensureModelLoaded();
  /// @brief 解析 YOLO 输出并执行 NMS。
  /// @param outputs 模型输出张量。
  /// @param frame_width 图像宽。
  /// @param frame_height 图像高。
  /// @return 过滤后的检测结果。
  std::vector<Detection> parseDetections(
    const std::vector<cv::Mat> & outputs,
    int frame_width,
    int frame_height) const;
  /// @brief 发布无检测占位消息。
  /// @param image_msg 输入图像消息。
  /// @param reason 无检测原因。
  void publishNoBox(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    const std::string & reason);

  std::string image_topic_;
  std::string box_result_topic_;
  std::string model_path_;
  double confidence_threshold_;
  double nms_threshold_;
  int max_boxes_;
  std::vector<std::string> class_names_;

  cv::dnn::Net net_;
  bool model_load_attempted_;
  bool model_loaded_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<dog_interfaces::msg::Target3DArray>::SharedPtr result_pub_;
};

}  // namespace dog_perception
