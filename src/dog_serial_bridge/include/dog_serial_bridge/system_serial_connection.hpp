#pragma once

#include "dog_serial_bridge/serial_connection.hpp"

#include <mutex>
#include <string>

namespace dog_serial_bridge
{

class SystemSerialConnection : public SerialConnection
{
public:
  SystemSerialConnection();
  ~SystemSerialConnection() override;

  bool open(const SerialConfig & config, std::string & error) override;
  bool isOpen() const override;
  void close() override;
  bool write(const std::string & data, std::string & error) override;
  ReadResult readLine(std::chrono::milliseconds timeout) override;

private:
  mutable std::mutex mutex_;
  int fd_;
  char delimiter_;
  std::string buffered_data_;
};

}  // namespace dog_serial_bridge
