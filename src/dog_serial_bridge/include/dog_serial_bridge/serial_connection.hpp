#pragma once

#include <chrono>
#include <string>

namespace dog_serial_bridge
{

struct SerialConfig
{
  std::string port;
  int baud_rate{115200};
  char line_delimiter{'\n'};
};

class SerialConnection
{
public:
  enum class ReadStatus
  {
    kLine,
    kTimeout,
    kClosed,
    kError,
  };

  struct ReadResult
  {
    ReadStatus status{ReadStatus::kTimeout};
    std::string line;
    std::string error;
    std::string partial_data;
  };

  struct ByteReadResult
  {
    ReadStatus status{ReadStatus::kTimeout};
    std::string data;
    std::string error;
  };

  virtual ~SerialConnection() = default;

  virtual bool open(const SerialConfig & config, std::string & error) = 0;
  virtual bool isOpen() const = 0;
  virtual void close() = 0;
  virtual void clearReadBuffer() {}
  virtual bool write(const std::string & data, std::string & error) = 0;
  virtual ReadResult readLine(std::chrono::milliseconds timeout) = 0;
  virtual ByteReadResult readBytes(std::chrono::milliseconds timeout)
  {
    const auto line_result = readLine(timeout);
    ByteReadResult result;
    result.status = line_result.status;
    result.data = line_result.status == ReadStatus::kLine ? line_result.line : line_result.partial_data;
    result.error = line_result.error;
    return result;
  }
};

}  // namespace dog_serial_bridge
