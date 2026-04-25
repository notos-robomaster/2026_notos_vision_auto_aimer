#ifndef IO_SIMPLE_SERIAL_HPP
#define IO_SIMPLE_SERIAL_HPP

#include <string>
#include <chrono>
#include <mutex>
#include <thread>
#include <atomic>
#include <memory>
#include <vector>
#include "tools/logger.hpp"

namespace io {

// 简单的云台状态数据结构
struct SimpleSerialState {
  std::mutex m;
  double pitch_deg = 0.0;
  double yaw_deg = 0.0;
  double roll_deg = 0.0;
  std::chrono::steady_clock::time_point timestamp;
  bool valid = false;
};

class SimpleSerial {
public:
  SimpleSerial(const std::string& port_name);
  ~SimpleSerial();

  // 获取当前状态
  std::shared_ptr<SimpleSerialState> get_state() const { return state_; }
  
  // 发送打包后的指令
  bool send_command(float pitch, float yaw, int fire, int frame_count);
  
  bool is_open() const { return fd_ >= 0; }
  int get_fd() const { return fd_; }

private:
  int fd_ = -1;
  std::shared_ptr<SimpleSerialState> state_;
  std::shared_ptr<std::atomic<bool>> running_;
  std::shared_ptr<std::thread> thread_;

  void receive_loop();
};

// CRC16 Modbus 校验函数声明
uint16_t crc16_modbus(const uint8_t *data, size_t length);

} // namespace io

#endif // IO_SIMPLE_SERIAL_HPP
