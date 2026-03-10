# ROS2 云台控制数据发布说明

## 功能介绍

`auto_aim_test` 程序现在支持通过 ROS2 发布云台控制指令（pitch/yaw）数据，方便在 PlotJuggler 等工具中实时查看和分析。

## 编译选项

### 1. 无 ROS2 支持（当前默认）
如果系统没有安装 ROS2，程序会自动编译为无 ROS2 支持版本，所有功能正常，只是不会发布 ROS2 消息。

```bash
make -C build/ -j$(nproc)
./build/auto_aim_test
```

### 2. 启用 ROS2 支持
如果系统已安装 ROS2 Humble/Foxy，需要在编译前 source ROS2 环境：

```bash
source /opt/ros/humble/setup.bash  # 或 /opt/ros/foxy/setup.bash
cd /home/oconnor/Downloads/sp_vision_25-main
rm -rf build  # 清除旧的编译缓存
cmake -B build
make -C build/ -j$(nproc)
```

如果编译成功，终端会显示：
```
-- ROS2 support enabled for auto_aim_test
```

## ROS2 话题说明

### 话题名称
`/gimbal_command`

### 消息类型
`std_msgs/msg/Float64MultiArray`

### 数据格式

#### MPC 模式（use_mpc: true）
```yaml
data: [pitch, yaw, fire, pitch_vel, yaw_vel, pitch_acc, yaw_acc]
```
- `pitch`: pitch 角度（弧度）
- `yaw`: yaw 角度（弧度）
- `fire`: 开火标志（0/1）
- `pitch_vel`: pitch 角速度（弧度/秒）
- `yaw_vel`: yaw 角速度（弧度/秒）
- `pitch_acc`: pitch 角加速度（弧度/秒²）
- `yaw_acc`: yaw 角加速度（弧度/秒²）

#### Aimer 模式（use_mpc: false）
```yaml
data: [pitch, yaw, shoot]
```
- `pitch`: pitch 角度（弧度）
- `yaw`: yaw 角度（弧度）
- `shoot`: 射击标志（0/1）

## 使用 PlotJuggler 实时查看数据

### 1. 安装 PlotJuggler
```bash
# 如果已经下载 AppImage
cd ~/Downloads
chmod +x PlotJuggler-latest.AppImage
./PlotJuggler-latest.AppImage &
```

### 2. 运行 auto_aim_test
```bash
# 在新终端
source /opt/ros/humble/setup.bash
cd /home/oconnor/Downloads/sp_vision_25-main
./build/auto_aim_test
```

### 3. 在 PlotJuggler 中订阅话题

1. 点击 **Streaming** → **Start ROS2 Topic Subscriber**
2. 在弹出的话题列表中勾选 `/gimbal_command`
3. 点击 **OK**
4. 在左侧数据列表中，将 `gimbal_command/data[0]` (pitch) 和 `gimbal_command/data[1]` (yaw) 拖到右侧绘图区
5. 实时查看 pitch/yaw 变化曲线

### 4. 查看角速度和加速度（MPC 模式）
- `data[3]`: pitch 角速度
- `data[4]`: yaw 角速度
- `data[5]`: pitch 角加速度
- `data[6]`: yaw 角加速度

## 命令行查看话题

### 查看话题列表
```bash
ros2 topic list
```

### 实时显示数据
```bash
ros2 topic echo /gimbal_command
```

### 查看话题频率
```bash
ros2 topic hz /gimbal_command
```

## 调试建议

1. **数据单位转换**：话题中的角度为弧度，如需要度数请乘以 57.3（180/π）
2. **PlotJuggler 保存布局**：配置好绘图后，可以保存为 `.xml` 布局文件，下次直接加载
3. **录制数据包**：可以使用 `ros2 bag record /gimbal_command` 录制数据用于离线分析

## 故障排查

### 编译时提示找不到 rclcpp
说明 ROS2 环境未正确配置，解决方法：
```bash
source /opt/ros/humble/setup.bash
rm -rf build
cmake -B build
make -C build/ -j$(nproc)
```

### 运行时无 ROS2 话题
1. 检查程序启动日志是否显示：`ROS2 gimbal_command publisher initialized`
2. 如果没有，说明编译时未启用 ROS2 支持，需要重新编译（见上述编译步骤）
3. 确认 `ros2 topic list` 能显示 `/gimbal_command`

### PlotJuggler 看不到话题
1. 确保 PlotJuggler 和 auto_aim_test 使用相同的 ROS2 环境
2. 检查 `ros2 topic list` 是否能看到话题
3. 重启 PlotJuggler 的 ROS2 Topic Subscriber 插件
