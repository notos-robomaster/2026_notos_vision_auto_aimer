#ifndef IO__SIMPLE_GIMBAL_HPP
#define IO__SIMPLE_GIMBAL_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>

#include "serial/serial.h"
#include "tools/thread_safe_queue.hpp"
#include "io/gimbal/gimbal.hpp"  // 包含基础定义以重用VisionToGimbal等

namespace io
{
// 简化的下位机数据包（只传yaw和pitch角度）
struct __attribute__((packed)) SimpleGimbalToVision
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode;           // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
  float yaw;              // yaw角度（弧度，相对于初始位置）
  float pitch;            // pitch角度（弧度，相对于初始位置）
  float yaw_vel;          // yaw角速度（弧度/秒）
  float pitch_vel;        // pitch角速度（弧度/秒）
  float bullet_speed;     // 子弹速度（m/s）
  uint16_t bullet_count;  // 子弹累计发送次数
  uint16_t crc16;
};

static_assert(sizeof(SimpleGimbalToVision) <= 64);

// VisionToGimbal, GimbalMode, GimbalState 已在 gimbal.hpp 中定义

class SimpleGimbal
{
public:
  SimpleGimbal(const std::string & config_path);
  ~SimpleGimbal();

  GimbalMode mode() const;
  GimbalState state() const;
  std::string str(GimbalMode mode) const;
  
  // 根据时间戳获取四元数（通过yaw/pitch角度构造）
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc);

private:
  serial::Serial serial_;
  std::thread thread_;
  std::atomic<bool> quit_ = false;
  mutable std::mutex mutex_;

  SimpleGimbalToVision rx_data_;
  VisionToGimbal tx_data_;

  GimbalMode mode_ = GimbalMode::IDLE;
  GimbalState state_;
  
  // 存储带时间戳的yaw/pitch数据
  struct AngleData {
    float yaw;
    float pitch;
    std::chrono::steady_clock::time_point timestamp;
  };
  tools::ThreadSafeQueue<AngleData> queue_{1000};

  bool read(uint8_t * buffer, size_t size);
  void read_thread();
  void reconnect();
  
  // 从yaw/pitch角度构造四元数（roll=0）
  static Eigen::Quaterniond angles_to_quaternion(float yaw, float pitch);
};

}  // namespace io

#endif  // IO__SIMPLE_GIMBAL_HPP
