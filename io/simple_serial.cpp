#include "simple_serial.hpp"
#include "tools/logger.hpp"
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <regex>
#include <chrono>
#include <fmt/core.h>

namespace io {

uint16_t crc16_modbus(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc = crc >> 1;
      }
    }
  }
  return crc;
}

SimpleSerial::SimpleSerial(const std::string& port_name) {
  fd_ = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd_ == -1) {
    tools::logger()->error("Failed to open serial port: {}", port_name);
    throw std::runtime_error("Serial port open failed");
  }

  struct termios options;
  tcgetattr(fd_, &options);
  cfsetispeed(&options, B115200);
  cfsetospeed(&options, B115200);
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_cflag |= CREAD | CLOCAL;
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_oflag &= ~OPOST;
  options.c_iflag &= ~(IXON | IXOFF | IXANY);
  tcsetattr(fd_, TCSANOW, &options);

  tools::logger()->info("Simple serial opened on {} (fd={})", port_name, fd_);

  state_ = std::make_shared<SimpleSerialState>();
  running_ = std::make_shared<std::atomic<bool>>(true);
  thread_ = std::make_shared<std::thread>(&SimpleSerial::receive_loop, this);
}

SimpleSerial::~SimpleSerial() {
  if (running_) running_->store(false);
  if (thread_ && thread_->joinable()) thread_->join();
  if (fd_ >= 0) close(fd_);
}

bool SimpleSerial::send_command(float pitch, float yaw, int fire, int frame_count) {
  if (fd_ < 0) return false;
  char frame[64];
  int len = snprintf(frame, sizeof(frame), "AAP%.2fY%.2fF%dN%d", pitch, yaw, fire, frame_count);
  uint16_t crc = crc16_modbus((uint8_t *)frame, len);
  len += snprintf(frame + len, sizeof(frame) - len, "C%04x%%\n", crc);
  
  int written = write(fd_, frame, len);
  if (written != len) {
    tools::logger()->warn("Serial write incomplete: {}/{}", written, len);
    return false;
  }
  return true;
}

void SimpleSerial::receive_loop() {
  std::string buf;
  buf.reserve(1024);
  char rbuf[256];
  // 帧正则：必须以ARP开头，后跟数字和小数点、负号，然后是C和4位16进制CRC
  static const std::regex frame_re(R"(ARP[-+]?\d+\.?\d*Y[-+]?\d+\.?\d*R[-+]?\d+\.?\d*C([0-9A-Fa-f]{4})%?)");
  static const std::regex num_re(R"([-+]?\d+(?:\.\d+)?)");

  while (running_->load()) {
    ssize_t n = read(fd_, rbuf, sizeof(rbuf));
    if (n > 0) {
      buf.append(rbuf, rbuf + n);
      if (buf.size() > 2048) {
        buf.erase(0, buf.size() - 1024);
        tools::logger()->warn("[SimpleSerial RX] Buffer overflow, cleared old data");
      }
      
      size_t pos;
      while ((pos = buf.find('\n')) != std::string::npos) {
        std::string line = buf.substr(0, pos);
        buf.erase(0, pos + 1);
        while (!line.empty() && isspace((unsigned char)line.back())) line.pop_back();
        if (line.empty()) continue;

        size_t arp_pos = line.find("ARP");
        if (arp_pos == std::string::npos) continue;
        std::string clean_line = line.substr(arp_pos);
        
        std::smatch match;
        if (std::regex_match(clean_line, match, frame_re)) {
          std::string frame = match[0].str();
          std::string crc_hex = match[1].str();
          size_t c_pos = frame.find('C');
          if (c_pos == std::string::npos) continue;
          
          std::string payload = frame.substr(0, c_pos);
          uint16_t recv_crc = static_cast<uint16_t>(strtol(crc_hex.c_str(), nullptr, 16));
          uint16_t calc_crc = crc16_modbus(reinterpret_cast<const uint8_t*>(payload.data()), payload.size());
          
          if (recv_crc != calc_crc) {
             tools::logger()->warn("[SimpleSerial RX] CRC mismatch: recv=0x{:04x}, calc=0x{:04x}", recv_crc, calc_crc);
             continue;
          }

          std::sregex_iterator it(payload.begin(), payload.end(), num_re), end;
          std::vector<double> nums;
          for (; it != end && nums.size() < 3; ++it) {
            try { nums.push_back(std::stod(it->str())); } catch (...) {}
          }
          if (nums.size() >= 3) {
            double pitch_deg = nums[0];
            double yaw_deg = nums[1];
            double roll_deg = nums[2];
            {
              std::lock_guard<std::mutex> lk(state_->m);
              state_->pitch_deg = pitch_deg;
              state_->yaw_deg = yaw_deg;
              state_->roll_deg = roll_deg;
              state_->timestamp = std::chrono::steady_clock::now();
              state_->valid = true;
            }
          }
        }
      }
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
}

} // namespace io
