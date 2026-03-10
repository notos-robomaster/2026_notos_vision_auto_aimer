#include "simple_gimbal.hpp"

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{

SimpleGimbal::SimpleGimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");

  try {
    serial_.setPort(com_port);
    serial_.open();
  } catch (const std::exception & e) {
    tools::logger()->error("[SimpleGimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  thread_ = std::thread(&SimpleGimbal::read_thread, this);

  // 等待第一个数据
  queue_.pop();
  tools::logger()->info("[SimpleGimbal] First angle data received.");
}

SimpleGimbal::~SimpleGimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  serial_.close();
}

GimbalMode SimpleGimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState SimpleGimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string SimpleGimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    default:
      return "INVALID";
  }
}

// 从yaw/pitch角度构造四元数（roll=0）
Eigen::Quaterniond SimpleGimbal::angles_to_quaternion(float yaw, float pitch)
{
  // ZYX欧拉角转四元数 (Yaw绕Z, Pitch绕Y, Roll=0绕X)
  Eigen::Quaterniond q = 
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
  
  return q.normalized();
}

Eigen::Quaterniond SimpleGimbal::q(std::chrono::steady_clock::time_point t)
{
  while (true) {
    auto data_a = queue_.pop();
    auto data_b = queue_.front();
    
    auto t_a = data_a.timestamp;
    auto t_b = data_b.timestamp;
    auto t_ab = tools::delta_time(t_b, t_a);
    auto t_ac = tools::delta_time(t, t_a);
    auto k = t_ac / t_ab;
    
    // 构造两个时刻的四元数
    Eigen::Quaterniond q_a = angles_to_quaternion(data_a.yaw, data_a.pitch);
    Eigen::Quaterniond q_b = angles_to_quaternion(data_b.yaw, data_b.pitch);
    
    // 球面线性插值
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    
    if (t < t_a) return q_c;
    if (!(t_a < t && t <= t_b)) continue;

    return q_c;
  }
}

void SimpleGimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  tx_data_.mode = control ? (fire ? 2 : 1) : 0;
  tx_data_.yaw = yaw;
  tx_data_.yaw_vel = yaw_vel;
  tx_data_.yaw_acc = yaw_acc;
  tx_data_.pitch = pitch;
  tx_data_.pitch_vel = pitch_vel;
  tx_data_.pitch_acc = pitch_acc;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[SimpleGimbal] Failed to write serial: {}", e.what());
  }
}

bool SimpleGimbal::read(uint8_t * buffer, size_t size)
{
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    return false;
  }
}

void SimpleGimbal::reconnect()
{
  tools::logger()->warn("[SimpleGimbal] Attempting to reconnect...");
  try {
    if (serial_.isOpen()) {
      serial_.close();
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    serial_.open();
    tools::logger()->info("[SimpleGimbal] Reconnected successfully");
  } catch (const std::exception & e) {
    tools::logger()->error("[SimpleGimbal] Reconnect failed: {}", e.what());
  }
}

void SimpleGimbal::read_thread()
{
  tools::logger()->info("[SimpleGimbal] read_thread started.");
  int error_count = 0;

  while (!quit_) {
    if (error_count > 5000) {
      error_count = 0;
      tools::logger()->warn("[SimpleGimbal] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    // 读取帧头
    if (!read(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }

    // 验证帧头
    if (rx_data_.head[0] != 'S' || rx_data_.head[1] != 'P') continue;

    auto t = std::chrono::steady_clock::now();

    // 读取剩余数据
    if (!read(
          reinterpret_cast<uint8_t *>(&rx_data_) + sizeof(rx_data_.head),
          sizeof(rx_data_) - sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }

    // CRC校验
    auto crc_calculated = tools::get_crc16(
      reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_) - sizeof(rx_data_.crc16));

    uint16_t rx_crc = rx_data_.crc16;  // 复制到临时变量以避免packed字段引用问题
    if (crc_calculated != rx_crc) {
      error_count++;
      tools::logger()->warn(
        "[SimpleGimbal] CRC mismatch: expected {:04x}, got {:04x}", rx_crc, crc_calculated);
      continue;
    }

    error_count = 0;

    // 更新模式和状态
    {
      std::lock_guard<std::mutex> lock(mutex_);
      mode_ = static_cast<GimbalMode>(rx_data_.mode);
      state_.yaw = rx_data_.yaw;
      state_.pitch = rx_data_.pitch;
      state_.yaw_vel = rx_data_.yaw_vel;
      state_.pitch_vel = rx_data_.pitch_vel;
      state_.bullet_speed = rx_data_.bullet_speed;
      state_.bullet_count = rx_data_.bullet_count;
    }

    // 存储角度数据到队列
    AngleData angle_data;
    angle_data.yaw = rx_data_.yaw;
    angle_data.pitch = rx_data_.pitch;
    angle_data.timestamp = t;
    queue_.push(angle_data);

    // 调试输出
    // tools::logger()->debug(
    //   "[SimpleGimbal] Received: yaw={:.3f}rad ({:.1f}°), pitch={:.3f}rad ({:.1f}°), "
    //   "yaw_vel={:.3f}, pitch_vel={:.3f}",
    //   rx_data_.yaw, rx_data_.yaw * 57.3,
    //   rx_data_.pitch, rx_data_.pitch * 57.3,
    //   rx_data_.yaw_vel, rx_data_.pitch_vel);
  }
}

}  // namespace io
