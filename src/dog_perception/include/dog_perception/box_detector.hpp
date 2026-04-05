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

class BoxDetector
{
public:
  struct Params
  {
    std::string model_path;
    double confidence_threshold;
    double nms_threshold;
    int max_boxes;
    std::vector<std::string> class_names;
  };

  /// @brief 构造箱体类型识别器
  /// @param params 推理参数。
  /// @param logger 日志器。
  explicit BoxDetector(const Params & params, const rclcpp::Logger & logger);

  /// @brief 对输入图像执行检测并返回识别结果。
  /// @param image_msg 图像消息。
  /// @return 识别结果数组（无检测时返回 no_box）。
  dog_interfaces::msg::Target3DArray detect(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg);

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
  /// @brief 构造无检测占位消息。
  /// @param image_msg 输入图像消息。
  /// @param reason 无检测原因。
  dog_interfaces::msg::Target3DArray makeNoBox(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    const std::string & reason) const;

  std::string model_path_;
  double confidence_threshold_;
  double nms_threshold_;
  int max_boxes_;
  std::vector<std::string> class_names_;
  rclcpp::Logger logger_;

  cv::dnn::Net net_;
  bool model_load_attempted_;
  bool model_loaded_;
};

}  // namespace dog_perception
