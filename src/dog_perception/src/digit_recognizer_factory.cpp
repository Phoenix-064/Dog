#include "dog_perception/digit_recognizer.hpp"

#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace dog_perception
{

namespace
{
/// @brief 获取数字识别器注册表单例。
/// @return 类型到创建器的映射引用。
std::unordered_map<std::string, DigitRecognizerCreator> & recognizerRegistry()
{
  static std::unordered_map<std::string, DigitRecognizerCreator> registry;
  return registry;
}

/// @brief 获取数字识别器注册表互斥锁单例。
/// @return 互斥锁引用。
std::mutex & recognizerRegistryMutex()
{
  static std::mutex registry_mutex;
  return registry_mutex;
}

/// @brief 确保内置数字识别器只注册一次。
/// @param logger 日志器。
void ensureBuiltinDigitRecognizers(const rclcpp::Logger & logger)
{
  static std::once_flag register_once;
  std::call_once(register_once, [&logger]() {
    const bool heuristic_registered = registerHeuristicDigitRecognizer();
    const bool mean_intensity_registered = registerMeanIntensityDigitRecognizer();
    if (!heuristic_registered) {
      RCLCPP_ERROR(logger, "Failed to register builtin recognizer: heuristic");
    }
    if (!mean_intensity_registered) {
      RCLCPP_ERROR(logger, "Failed to register builtin recognizer: mean_intensity");
    }
  });
}

}  // namespace

bool registerDigitRecognizer(
  const std::string & recognizer_type,
  DigitRecognizerCreator creator)
{
  if (recognizer_type.empty() || !creator) {
    return false;
  }

  std::lock_guard<std::mutex> lock(recognizerRegistryMutex());
  auto & registry = recognizerRegistry();
  const auto iterator = registry.find(recognizer_type);
  if (iterator != registry.end()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("digit_recognizer_factory"),
      "Duplicate digit recognizer registration rejected: %s",
      recognizer_type.c_str());
    return false;
  }

  registry.emplace(recognizer_type, std::move(creator));
  return true;
}

std::unique_ptr<IDigitRecognizer> DigitRecognizerFactory::create(
  const std::string & recognizer_type,
  const DigitRecognizerParams & params,
  const rclcpp::Logger & logger)
{
  ensureBuiltinDigitRecognizers(logger);

  const std::string resolved_type = recognizer_type.empty() ? "heuristic" : recognizer_type;
  DigitRecognizerCreator creator;
  bool fallback_used = false;

  {
    std::lock_guard<std::mutex> lock(recognizerRegistryMutex());
    const auto & registry = recognizerRegistry();
    const auto iterator = registry.find(resolved_type);
    if (iterator != registry.end()) {
      creator = iterator->second;
    } else {
      const auto fallback_iterator = registry.find("heuristic");
      if (fallback_iterator != registry.end()) {
        creator = fallback_iterator->second;
        fallback_used = true;
      }
    }
  }

  if (!creator) {
    throw std::runtime_error(
            "No registered digit recognizer available. Requested type: " + resolved_type);
  }

  if (fallback_used) {
    RCLCPP_WARN(
      logger,
      "Unsupported digit recognizer type: %s, fallback to heuristic",
      resolved_type.c_str());
  }

  auto recognizer = creator(params, logger);
  if (!recognizer) {
    throw std::runtime_error(
            "Digit recognizer creator returned null. Requested type: " + resolved_type);
  }
  return recognizer;
}

dog_interfaces::msg::Target3D toDigitTarget3D(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  const std::string & output_frame_id,
  const DigitRecognitionResult & result)
{
  dog_interfaces::msg::Target3D message;
  if (image_msg) {
    message.header.stamp = image_msg->header.stamp;
  }
  message.header.frame_id = output_frame_id;
  message.target_id = result.has_feature ? ("digit_" + std::to_string(result.label)) : "no_feature";
  message.confidence = result.confidence;
  return message;
}

}  // namespace dog_perception
