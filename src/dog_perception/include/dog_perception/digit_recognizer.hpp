#pragma once

#include <dog_interfaces/msg/target3_d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>
#include <string>
#include <functional>

namespace dog_perception
{

struct ImageView
{
  sensor_msgs::msg::Image::ConstSharedPtr image;
};

struct DigitRecognizerParams
{
  int roi_x;
  int roi_y;
  int roi_width;
  int roi_height;
  double min_confidence;
  double glare_brightness_threshold;
  double glare_ratio_threshold;
};

struct DigitRecognitionResult
{
  bool has_feature;
  int label;
  float confidence;
  std::string reason;
};

class IDigitRecognizer
{
public:
  /// @brief 多态数字识别器的虚析构函数。
  virtual ~IDigitRecognizer() = default;

  /// @brief 在给定图像视图上执行数字推理。
  /// @param view 输入图像视图。
  /// @return 数字识别结果。
  virtual DigitRecognitionResult infer(const ImageView & view) = 0;
};

using DigitRecognizerCreator = std::function<std::unique_ptr<IDigitRecognizer>(
    const DigitRecognizerParams &, const rclcpp::Logger &)>;

/// @brief 注册具名数字识别器创建器。
/// @param recognizer_type 工厂使用的类型键。
/// @param creator 用于创建识别器实例的工厂回调。
/// @return 注册成功时返回 true。
bool registerDigitRecognizer(
  const std::string & recognizer_type,
  DigitRecognizerCreator creator);

/// @brief 注册内置启发式数字识别器实现。
/// @return 注册成功时返回 true。
bool registerHeuristicDigitRecognizer();
/// @brief 注册内置均值强度数字识别器实现。
/// @return 注册成功时返回 true。
bool registerMeanIntensityDigitRecognizer();

class DigitRecognizerFactory
{
public:
  /// @brief 按类型创建数字识别器，并支持回退策略。
  /// @param recognizer_type 请求的识别器类型。
  /// @param params 识别器运行时参数。
  /// @param logger 诊断日志器。
  /// @return 数字识别器实例。
  static std::unique_ptr<IDigitRecognizer> create(
    const std::string & recognizer_type,
    const DigitRecognizerParams & params,
    const rclcpp::Logger & logger);
};

/// @brief 将数字识别结果转换为 Target3D 消息形式。
/// @param image_msg 源图像消息。
/// @param output_frame_id 输出坐标系 id。
/// @param result 数字识别结果。
/// @return 用于发布的 Target3D 表示。
dog_interfaces::msg::Target3D toDigitTarget3D(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  const std::string & output_frame_id,
  const DigitRecognitionResult & result);

}  // namespace dog_perception
