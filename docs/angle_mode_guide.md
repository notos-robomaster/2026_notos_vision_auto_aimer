# 下位机角度模式发送示例

当使用 `angle_mode: true` 配置时，下位机需要发送yaw和pitch的角度值（弧度）。

## 数据包格式

```cpp
struct __attribute__((packed)) SimpleGimbalToVision
{
  uint8_t head[2] = {'S', 'P'};  // 帧头固定为 'S', 'P'
  uint8_t mode;                   // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
  float yaw;                      // yaw角度（弧度，相对于初始位置）
  float pitch;                    // pitch角度（弧度，相对于初始位置）
  float yaw_vel;                  // yaw角速度（弧度/秒）
  float pitch_vel;                // pitch角速度（弧度/秒）
  float bullet_speed;             // 子弹速度（m/s）
  uint16_t bullet_count;          // 子弹累计发送次数
  uint16_t crc16;                 // CRC16校验（Modbus）
};
```

## 下位机示例代码（C/C++）

```cpp
#include <stdint.h>
#include <string.h>

// CRC16 Modbus 校验函数
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

// 发送数据包
void send_gimbal_data(
  uint8_t mode,
  float yaw_rad,        // yaw角度（弧度）
  float pitch_rad,      // pitch角度（弧度）
  float yaw_vel,        // yaw角速度（rad/s）
  float pitch_vel,      // pitch角速度（rad/s）
  float bullet_speed,   // 子弹速度（m/s）
  uint16_t bullet_count // 子弹累计次数
) {
  uint8_t buffer[64];
  size_t offset = 0;
  
  // 帧头
  buffer[offset++] = 'S';
  buffer[offset++] = 'P';
  
  // 模式
  buffer[offset++] = mode;
  
  // yaw角度（弧度）
  memcpy(&buffer[offset], &yaw_rad, sizeof(float));
  offset += sizeof(float);
  
  // pitch角度（弧度）
  memcpy(&buffer[offset], &pitch_rad, sizeof(float));
  offset += sizeof(float);
  
  // yaw角速度
  memcpy(&buffer[offset], &yaw_vel, sizeof(float));
  offset += sizeof(float);
  
  // pitch角速度
  memcpy(&buffer[offset], &pitch_vel, sizeof(float));
  offset += sizeof(float);
  
  // 子弹速度
  memcpy(&buffer[offset], &bullet_speed, sizeof(float));
  offset += sizeof(float);
  
  // 子弹计数
  memcpy(&buffer[offset], &bullet_count, sizeof(uint16_t));
  offset += sizeof(uint16_t);
  
  // 计算CRC16（不包括CRC字段本身）
  uint16_t crc = crc16_modbus(buffer, offset);
  memcpy(&buffer[offset], &crc, sizeof(uint16_t));
  offset += sizeof(uint16_t);
  
  // 通过串口发送（具体实现取决于你的平台）
  uart_send(buffer, offset);
}

// 使用示例
void example_usage() {
  // 从电机编码器获取当前角度
  float current_yaw_deg = 45.0f;      // 假设当前yaw为45度
  float current_pitch_deg = -10.0f;   // 假设当前pitch为-10度
  
  // 转换为弧度
  float yaw_rad = current_yaw_deg * 3.14159265f / 180.0f;
  float pitch_rad = current_pitch_deg * 3.14159265f / 180.0f;
  
  // 获取角速度（从电机反馈或IMU）
  float yaw_vel = 0.5f;     // rad/s
  float pitch_vel = 0.2f;   // rad/s
  
  // 发送数据（建议100-200Hz频率）
  send_gimbal_data(
    1,              // mode: 1=自瞄模式
    yaw_rad,        // yaw角度（弧度）
    pitch_rad,      // pitch角度（弧度）
    yaw_vel,        // yaw角速度
    pitch_vel,      // pitch角速度
    27.0f,          // 子弹速度 27 m/s
    0               // 子弹计数
  );
}
```

## STM32 HAL库示例

```c
#include "main.h"
#include <string.h>

extern UART_HandleTypeDef huart1;  // 假设使用UART1

// 发送到上位机
void send_to_vision(void) {
  static uint16_t bullet_count = 0;
  
  // 从云台电机获取当前角度（假设已有函数）
  float yaw_motor_angle = get_yaw_motor_angle();    // 度
  float pitch_motor_angle = get_pitch_motor_angle(); // 度
  
  // 转换为弧度
  float yaw_rad = yaw_motor_angle * 0.0174532925f;
  float pitch_rad = pitch_motor_angle * 0.0174532925f;
  
  // 获取角速度
  float yaw_vel = get_yaw_velocity();      // rad/s
  float pitch_vel = get_pitch_velocity();  // rad/s
  
  // 构造数据包
  uint8_t buffer[32];
  size_t idx = 0;
  
  buffer[idx++] = 'S';
  buffer[idx++] = 'P';
  buffer[idx++] = 1;  // mode: 自瞄
  
  memcpy(&buffer[idx], &yaw_rad, 4); idx += 4;
  memcpy(&buffer[idx], &pitch_rad, 4); idx += 4;
  memcpy(&buffer[idx], &yaw_vel, 4); idx += 4;
  memcpy(&buffer[idx], &pitch_vel, 4); idx += 4;
  
  float bullet_speed = 27.0f;
  memcpy(&buffer[idx], &bullet_speed, 4); idx += 4;
  memcpy(&buffer[idx], &bullet_count, 2); idx += 2;
  
  // CRC16
  uint16_t crc = crc16_modbus(buffer, idx);
  memcpy(&buffer[idx], &crc, 2); idx += 2;
  
  // 发送
  HAL_UART_Transmit(&huart1, buffer, idx, 10);
  
  bullet_count++;
}

// 在定时器中断或主循环中以100-200Hz频率调用
void TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim6) {  // 假设使用TIM6，10ms中断（100Hz）
    send_to_vision();
  }
}
```

## 重要说明

### 1. 角度定义
- **yaw**: 相对于初始位置的水平旋转角度（弧度）
  - 向左转为正，向右转为负（或根据你的坐标系定义）
  - 建议范围：-π 到 +π
  
- **pitch**: 相对于水平面的俯仰角度（弧度）
  - 向上为正，向下为负
  - 建议范围：-π/2 到 +π/2

### 2. 发送频率
- **推荐频率**: 100-200Hz
- 必须**高于**相机帧率（至少2倍）
- 上位机会自动进行时间戳插值

### 3. 坐标系
- 确保yaw/pitch的正负方向与上位机坐标系一致
- 可通过配置文件中的 `R_gimbal2imubody` 调整坐标系对齐

### 4. 优势
- 下位机无需计算四元数，降低计算负担
- 只需发送角度值，数据量小
- 上位机自动处理四元数转换和插值
- 适合全向轮底盘（roll=0的场景）

### 5. 接收上位机控制指令

上位机会发送以下格式的控制指令：

```cpp
struct VisionToGimbal {
  uint8_t head[2];  // 'S', 'P'
  uint8_t mode;     // 0: 不控制, 1: 控制但不开火, 2: 控制且开火
  float yaw;        // 目标yaw（rad）
  float yaw_vel;    // 目标yaw速度
  float yaw_acc;    // 目标yaw加速度
  float pitch;      // 目标pitch（rad）
  float pitch_vel;  // 目标pitch速度
  float pitch_acc;  // 目标pitch加速度
  uint16_t crc16;
};
```

下位机接收后，根据mode和目标角度控制云台电机。
