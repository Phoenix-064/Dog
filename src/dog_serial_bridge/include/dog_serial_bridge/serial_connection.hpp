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
  };

  virtual ~SerialConnection() = default;

  virtual bool open(const SerialConfig & config, std::string & error) = 0;
  virtual bool isOpen() const = 0;
  virtual void close() = 0;
  virtual bool write(const std::string & data, std::string & error) = 0;
  virtual ReadResult readLine(std::chrono::milliseconds timeout) = 0;
};

}  // namespace dog_serial_bridge
