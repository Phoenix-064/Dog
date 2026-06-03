#include "dog_serial_bridge/system_serial_connection.hpp"

#include <algorithm>

#if defined(_WIN32)

namespace dog_serial_bridge
{

SystemSerialConnection::SystemSerialConnection()
: fd_(-1)
, delimiter_('\n')
{
}

SystemSerialConnection::~SystemSerialConnection()
{
  close();
}

bool SystemSerialConnection::open(const SerialConfig & config, std::string & error)
{
  (void)config;
  error = "serial_unsupported_on_this_platform";
  return false;
}

bool SystemSerialConnection::isOpen() const
{
  return false;
}

void SystemSerialConnection::close()
{
}

bool SystemSerialConnection::write(const std::string & data, std::string & error)
{
  (void)data;
  error = "serial_not_open";
  return false;
}

SerialConnection::ReadResult SystemSerialConnection::readLine(std::chrono::milliseconds timeout)
{
  (void)timeout;
  ReadResult result;
  result.status = ReadStatus::kClosed;
  result.error = "serial_not_open";
  return result;
}

}  // namespace dog_serial_bridge

#else

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace dog_serial_bridge
{

namespace
{

speed_t toBaudRate(const int baud_rate)
{
  switch (baud_rate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    default:
      return 0;
  }
}

std::string errnoText(const char * prefix)
{
  return std::string(prefix) + ":" + std::strerror(errno);
}

}  // namespace

SystemSerialConnection::SystemSerialConnection()
: fd_(-1)
, delimiter_('\n')
{
}

SystemSerialConnection::~SystemSerialConnection()
{
  close();
}

bool SystemSerialConnection::open(const SerialConfig & config, std::string & error)
{
  close();
  std::lock_guard<std::mutex> lock(mutex_);

  const auto speed = toBaudRate(config.baud_rate);
  if (speed == 0) {
    error = "unsupported_baud_rate";
    return false;
  }

  const int fd = ::open(config.port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    error = errnoText("open_failed");
    return false;
  }

  termios tty{};
  if (tcgetattr(fd, &tty) != 0) {
    error = errnoText("tcgetattr_failed");
    ::close(fd);
    return false;
  }

  cfmakeraw(&tty);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    error = errnoText("tcsetattr_failed");
    ::close(fd);
    return false;
  }

  fd_ = fd;
  delimiter_ = config.line_delimiter;
  buffered_data_.clear();
  error.clear();
  return true;
}

bool SystemSerialConnection::isOpen() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return fd_ >= 0;
}

void SystemSerialConnection::close()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  buffered_data_.clear();
}

bool SystemSerialConnection::write(const std::string & data, std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ < 0) {
    error = "serial_not_open";
    return false;
  }

  size_t total_written = 0U;
  while (total_written < data.size()) {
    const auto bytes_written = ::write(fd_, data.data() + total_written, data.size() - total_written);
    if (bytes_written < 0) {
      if (errno == EINTR) {
        continue;
      }
      error = errnoText("write_failed");
      return false;
    }
    total_written += static_cast<size_t>(bytes_written);
  }

  error.clear();
  return true;
}

SerialConnection::ReadResult SystemSerialConnection::readLine(std::chrono::milliseconds timeout)
{
  std::lock_guard<std::mutex> lock(mutex_);

  ReadResult result;
  if (fd_ < 0) {
    result.status = ReadStatus::kClosed;
    result.error = "serial_not_open";
    return result;
  }

  const auto delimiter_pos = buffered_data_.find(delimiter_);
  if (delimiter_pos != std::string::npos) {
    result.status = ReadStatus::kLine;
    result.line = buffered_data_.substr(0, delimiter_pos + 1U);
    buffered_data_.erase(0, delimiter_pos + 1U);
    return result;
  }

  pollfd pfd{};
  pfd.fd = fd_;
  pfd.events = POLLIN;
  const int poll_result = ::poll(&pfd, 1, static_cast<int>(timeout.count()));
  if (poll_result == 0) {
    result.status = ReadStatus::kTimeout;
    return result;
  }
  if (poll_result < 0) {
    if (errno == EINTR) {
      result.status = ReadStatus::kTimeout;
      return result;
    }
    result.status = ReadStatus::kError;
    result.error = errnoText("poll_failed");
    return result;
  }
  if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
    result.status = ReadStatus::kClosed;
    result.error = "serial_closed";
    return result;
  }

  char buffer[256];
  const auto bytes_read = ::read(fd_, buffer, sizeof(buffer));
  if (bytes_read < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
      result.status = ReadStatus::kTimeout;
      return result;
    }
    result.status = ReadStatus::kError;
    result.error = errnoText("read_failed");
    return result;
  }
  if (bytes_read == 0) {
    result.status = ReadStatus::kClosed;
    result.error = "serial_closed";
    return result;
  }

  buffered_data_.append(buffer, static_cast<size_t>(bytes_read));
  const auto new_delimiter_pos = buffered_data_.find(delimiter_);
  if (new_delimiter_pos == std::string::npos) {
    result.status = ReadStatus::kTimeout;
    return result;
  }

  result.status = ReadStatus::kLine;
  result.line = buffered_data_.substr(0, new_delimiter_pos + 1U);
  buffered_data_.erase(0, new_delimiter_pos + 1U);
  return result;
}

}  // namespace dog_serial_bridge

#endif
