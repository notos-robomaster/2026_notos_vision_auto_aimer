# Simple Serial 简单串口模式指南

`simple_serial` 模式使用**字符串格式**进行双向通信，适合简单的下位机实现。

## 数据格式

### 📥 下位机 → 上位机（发送当前姿态）

**格式**: `ARP<pitch>Y<yaw>C<crc>%\n`

- `AR`: 帧头标识
- `P<pitch>`: Pitch角度（度数，可以是负数）
- `Y<yaw>`: Yaw角度（度数，可以是负数）
- `C<crc>`: CRC16校验（4位十六进制，Modbus格式）
- `%\n`: 帧尾

**示例**:
```
ARP-10.50Y45.30C1a2b%\n
```
表示：Pitch = -10.50°, Yaw = 45.30°

### 📤 上位机 → 下位机（发送目标位置）

**格式**: `AAP<pitch>Y<yaw>C<crc>%\n`

- `AA`: 帧头标识
- `P<pitch>`: 目标Pitch角度（度数）
- `Y<yaw>`: 目标Yaw角度（度数）
- `C<crc>`: CRC16校验（4位十六进制，Modbus格式）
- `%\n`: 帧尾

**示例**:
```
AAP-12.34Y56.78C3c4d%\n
```
表示：控制云台移动到 Pitch = -12.34°, Yaw = 56.78°

## 下位机实现示例（C语言）

### 发送当前姿态

```c
#include <stdio.h>
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

// 发送当前姿态给上位机
void send_gimbal_pose(float pitch_deg, float yaw_deg) {
  char frame[64];
  
  // 1. 构造数据部分
  int len = snprintf(frame, sizeof(frame), "ARP%.2fY%.2f", pitch_deg, yaw_deg);
  
  // 2. 计算CRC16
  uint16_t crc = crc16_modbus((uint8_t*)frame, len);
  
  // 3. 添加CRC和帧尾
  len += snprintf(frame + len, sizeof(frame) - len, "C%04x%%\n", crc);
  
  // 4. 通过串口发送
  uart_send((uint8_t*)frame, len);
}

// 使用示例
void example_send(void) {
  // 从云台电机获取当前角度
  float current_pitch = get_pitch_angle();  // 假设这个函数返回当前pitch角度（度）
  float current_yaw = get_yaw_angle();      // 假设这个函数返回当前yaw角度（度）
  
  // 发送给上位机（建议100-200Hz频率）
  send_gimbal_pose(current_pitch, current_yaw);
}
```

### 接收目标指令

```c
#include <stdlib.h>
#include <string.h>

// 解析上位机发送的控制指令
int parse_vision_command(const char* frame, float* pitch_deg, float* yaw_deg) {
  // 检查帧头
  if (strncmp(frame, "AA", 2) != 0) {
    return -1;  // 帧头错误
  }
  
  // 查找P、Y、C位置
  const char* p_pos = strchr(frame, 'P');
  const char* y_pos = strchr(frame, 'Y');
  const char* c_pos = strchr(frame, 'C');
  
  if (!p_pos || !y_pos || !c_pos) {
    return -2;  // 格式错误
  }
  
  // 提取CRC
  char crc_str[5];
  strncpy(crc_str, c_pos + 1, 4);
  crc_str[4] = '\0';
  uint16_t received_crc = (uint16_t)strtol(crc_str, NULL, 16);
  
  // 验证CRC
  size_t data_len = c_pos - frame;
  uint16_t calculated_crc = crc16_modbus((uint8_t*)frame, data_len);
  
  if (received_crc != calculated_crc) {
    return -3;  // CRC校验失败
  }
  
  // 解析pitch和yaw
  *pitch_deg = atof(p_pos + 1);
  *yaw_deg = atof(y_pos + 1);
  
  return 0;  // 成功
}

// 串口接收处理
char rx_buffer[256];
int rx_index = 0;

void uart_rx_callback(uint8_t byte) {
  rx_buffer[rx_index++] = byte;
  
  // 查找完整帧（以 %\n 结尾）
  if (rx_index >= 2 && rx_buffer[rx_index-2] == '%' && rx_buffer[rx_index-1] == '\n') {
    rx_buffer[rx_index] = '\0';
    
    // 解析指令
    float target_pitch, target_yaw;
    if (parse_vision_command(rx_buffer, &target_pitch, &target_yaw) == 0) {
      // 控制云台移动到目标位置
      set_gimbal_target(target_pitch, target_yaw);
    }
    
    // 清空缓冲区
    rx_index = 0;
  }
  
  // 防止缓冲区溢出
  if (rx_index >= sizeof(rx_buffer) - 1) {
    rx_index = 0;
  }
}
```

### STM32 HAL库完整示例

```c
#include "main.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart1;

// 全局变量
char rx_buffer[256];
int rx_index = 0;

// 发送当前姿态（在定时器中断中调用，100Hz）
void send_current_pose(void) {
  char frame[64];
  
  // 获取当前云台角度
  float pitch = get_pitch_encoder() / 8192.0f * 360.0f;  // 示例：将编码器值转为角度
  float yaw = get_yaw_encoder() / 8192.0f * 360.0f;
  
  // 构造帧
  int len = snprintf(frame, sizeof(frame), "ARP%.2fY%.2f", pitch, yaw);
  uint16_t crc = crc16_modbus((uint8_t*)frame, len);
  len += snprintf(frame + len, sizeof(frame) - len, "C%04x%%\n", crc);
  
  // 发送
  HAL_UART_Transmit(&huart1, (uint8_t*)frame, len, 10);
}

// 接收回调（使用HAL_UART_Receive_IT）
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart1) {
    uint8_t byte;
    HAL_UART_Receive_IT(&huart1, &byte, 1);  // 重新启动接收
    
    rx_buffer[rx_index++] = byte;
    
    // 检查完整帧
    if (rx_index >= 2 && rx_buffer[rx_index-2] == '%' && rx_buffer[rx_index-1] == '\n') {
      rx_buffer[rx_index] = '\0';
      
      float target_pitch, target_yaw;
      if (parse_vision_command(rx_buffer, &target_pitch, &target_yaw) == 0) {
        // 设置PID目标
        pid_set_target(PID_PITCH, target_pitch);
        pid_set_target(PID_YAW, target_yaw);
      }
      
      rx_index = 0;
    }
    
    if (rx_index >= sizeof(rx_buffer) - 1) {
      rx_index = 0;
    }
  }
}

// 在main函数中初始化
int main(void) {
  // ... HAL初始化 ...
  
  // 启动串口接收中断
  uint8_t dummy;
  HAL_UART_Receive_IT(&huart1, &dummy, 1);
  
  // ... 其他初始化 ...
  
  while (1) {
    // 主循环
  }
}

// 在定时器中断中发送（100Hz = 10ms）
void TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim6) {  // 假设使用TIM6
    send_current_pose();
  }
}
```

## 配置文件设置

在 `configs/demo.yaml` 中：

```yaml
use_mpc: true           # 使用MPC控制器
use_serial: false       # 不使用Gimbal类（因为用simple_serial）
simple_serial: true     # 启用简单串口模式
angle_mode: false       # 不使用angle_mode（因为simple_serial已包含角度处理）
com_port: "/dev/ttyUSB0"
```

## 数据流程

```
下位机发送姿态 (100-200Hz)
    ↓
  ARP-10.5Y45.3C1a2b%\n
    ↓
上位机接收 → 解析 → 转四元数 → 目标检测 → 计算控制
    ↓
  AAP-12.3Y56.7C3c4d%\n
    ↓
下位机接收 → 控制云台电机
```

## 优缺点

### ✅ 优点
- 实现简单，下位机不需要复杂的结构体处理
- 易于调试，可以直接在串口助手中查看数据
- 对下位机要求低，任何支持字符串的MCU都能用

### ❌ 缺点
- 数据量较大（字符串比二进制大）
- 解析速度较慢（字符串解析比结构体慢）
- 不支持速度和加速度反馈（只有位置）

## 注意事项

1. **发送频率**: 建议100-200Hz，确保上位机有足够的姿态更新
2. **CRC校验**: 必须实现，防止数据错误导致云台失控
3. **角度单位**: 统一使用**度数**（上位机内部会转换为弧度）
4. **缓冲区**: 注意防止缓冲区溢出
5. **波特率**: 建议115200或更高
