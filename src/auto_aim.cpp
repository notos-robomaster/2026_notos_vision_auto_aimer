#include <fmt/core.h>
#include <fstream>  
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include "tasks/auto_aim/solver.hpp"
#include "io/camera.hpp"
#include "io/gimbal/simple_gimbal.hpp"
#include "io/simple_serial.hpp"
#include "tasks/auto_aim/aim_filter.hpp"  
#include "io/command.hpp"

#ifdef HAS_ROS2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#endif
#include "serial/serial.h"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/thread_safe_queue.hpp"
#include "yaml-cpp/yaml.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <mutex>
#include <thread>
#include <atomic>
#include <regex>
#include <deque>

using namespace std::chrono_literals;


const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{config-path c  | configs/demo.yaml | yaml配置文件路径}"
  "{d display      |                     | 显示视频流       }";


int main(int argc, char * argv[])
{
#ifdef HAS_ROS2
  // 初始化 ROS2
  rclcpp::init(argc, argv);
#endif
  
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
#ifdef HAS_ROS2
    rclcpp::shutdown();
#endif
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;

  auto config_path = cli.get<std::string>("config-path");
  auto display = cli.has("display");

#ifdef HAS_ROS2
  // 创建 ROS2 发布器用于发布 pitch/yaw 数据
  auto ros2_node = std::make_shared<rclcpp::Node>("auto_aim_gimbal_publisher");
  auto gimbal_cmd_pub = ros2_node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "gimbal_command", 10);
  
  // 在单独线程中 spin ROS2 节点
  std::thread ros2_spin_thread([ros2_node]() {
    rclcpp::spin(ros2_node);
  });
  ros2_spin_thread.detach();
  
  tools::logger()->info("ROS2 gimbal_command publisher initialized on topic: gimbal_command");
#endif

  // 变量声明区，保证主循环和初始化都能访问
  std::unique_ptr<auto_aim::Aimer> aimer;
  std::unique_ptr<auto_aim::Planner> planner;
  std::unique_ptr<io::Gimbal> gimbal;
  std::unique_ptr<io::SimpleGimbal> simple_gimbal;
  std::unique_ptr<io::SimpleSerial> simple_serial_inst;
  std::shared_ptr<io::SimpleSerialState> simple_state = nullptr;
  bool use_mpc = false;
  bool use_serial = false;
  bool simple_serial = false;
  bool angle_mode = false;

  try {
    std::cout << "All string fields are valid!" << std::endl;
    tools::logger()->info("Starting auto_aim with config: {}", config_path);
    // 检查配置文件
    std::ifstream config_file(config_path);
    if (!config_file.good()) {
      tools::logger()->error("Config file not found: {}", config_path);
      return -1;
    }
    config_file.close();

    tools::logger()->info("Step 1: Initializing camera...");
    io::Camera camera(config_path);

    tools::logger()->info("Step 2: Initializing YOLO...");
    auto_aim::YOLO yolo(config_path, display);

    tools::logger()->info("Step 3: Initializing Solver...");
    auto_aim::Solver solver(config_path);

    tools::logger()->info("Step 4: Initializing Tracker...");
    auto_aim::Tracker tracker(config_path, solver);

    tools::logger()->info("Step 4.5: Initializing Shooter...");
    auto_aim::Shooter shooter(config_path);

    // 读取控制模式配置
    YAML::Node cfg = YAML::LoadFile(config_path);
    if (cfg["use_mpc"]) {
      use_mpc = cfg["use_mpc"].as<bool>();
    }
    if (cfg["use_serial"]) {
      use_serial = cfg["use_serial"].as<bool>();
    }
    if (cfg["simple_serial"]) {
      simple_serial = cfg["simple_serial"].as<bool>();
    }
    if (cfg["angle_mode"]) {
      angle_mode = cfg["angle_mode"].as<bool>();
    }
    double bullet_speed = cfg["bullet_speed"] ? cfg["bullet_speed"].as<double>() : 27.0;

    // Step 5: 初始化控制器（MPC或Aimer）
    if (use_mpc) {
      tools::logger()->info("Step 5: Initializing Planner (MPC)...");
      planner = std::make_unique<auto_aim::Planner>(config_path);
    } else {
      tools::logger()->info("Step 5: Initializing Aimer (Traditional)...");
      aimer = std::make_unique<auto_aim::Aimer>(config_path);
    }

    // Step 6: 初始化串口通信（无论MPC还是Aimer都需要）
    if (use_serial && !simple_serial && !angle_mode) {
      tools::logger()->info("Step 6: Initializing Gimbal (Full Serial with Quaternion)...");
      gimbal = std::make_unique<io::Gimbal>(config_path);
    } else if (use_serial && angle_mode) {
      tools::logger()->info("Step 6: Initializing SimpleGimbal (Yaw/Pitch Angle Mode)...");
      simple_gimbal = std::make_unique<io::SimpleGimbal>(config_path);
    } else if (simple_serial) {
      tools::logger()->info("Step 6: Initializing Simple Serial (String format with CRC16)...");
      std::string com_port = cfg["com_port"] ? cfg["com_port"].as<std::string>() : "/dev/ttyUSB0";
      simple_serial_inst = std::make_unique<io::SimpleSerial>(com_port);
      simple_state = simple_serial_inst->get_state();
    } else {
      tools::logger()->info("Step 6: Serial disabled, will output to console");
    }

    tools::logger()->info("All components initialized successfully!");

    // 从这里开始是原有的主循环代码
    // 从配置文件读取相机类型并设置期望分辨率（优先判断相机，再判断分辨率）
    std::string cam_name = "mindvision";
    if (cfg["camera_name"]) {
      try { cam_name = cfg["camera_name"].as<std::string>(); } catch (...) { cam_name = "mindvision"; }
    }
    std::string lc; lc.resize(cam_name.size());
    std::transform(cam_name.begin(), cam_name.end(), lc.begin(), ::tolower);
    int desired_w = 640, desired_h = 480; // 默认迈德
    if (lc.find("hik") != std::string::npos || lc.find("hikrobot") != std::string::npos ||
        lc.find("hikvision") != std::string::npos) {
      desired_w = 720; desired_h = 540;
    }

    // img 初始化为空，按相机实际帧填充；首次成功读取视为相机已初始化
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
    auto last_stamp = std::chrono::steady_clock::now();
    auto t0 = std::chrono::steady_clock::now(); // 添加程序开始时间
    // 自瞄相关的状态变量
    auto_aim::Target last_target;
    io::Command last_command;
    int frame_count = 0;
    double last_t = -1; // 定义 last_t
    
    // 初始化滤波器
    auto_aim::AimFilter aim_filter;

    // 开火判断：连续稳定帧计数
    int stable_aim_frames = 0;
    constexpr int REQUIRED_STABLE_FRAMES = 5;  // 需要连续5帧满足条件才开火（更严格）
    constexpr double AIMER_FIRE_THRESH = 0.015;  // 开火阈值：0.015弧度 ≈ 0.86度（更严格，接近MPC）

    // 帧率计算变量
    auto fps_time_start = std::chrono::steady_clock::now();
    int fps_frame_count = 0;
    double current_fps = 0.0;

    // 固定云台姿态（在没有真实IMU数据的情况下使用）
    Eigen::Quaterniond fixed_quat(1.0, 0.0, 0.0, 0.0); // w=1, x=0, y=0, z=0
    Eigen::Quaterniond gimbal_q(1.0, 0.0, 0.0, 0.0);
    Eigen::Quaterniond last_gimbal_q(1.0, 0.0, 0.0, 0.0);
    double last_gimbal_time = 0.0;
    bool gimbal_stable = true;
    constexpr double GIMBAL_ANGULAR_VEL_THRESHOLD = 30.0 * M_PI / 180.0; // 30度/秒阈值
    
    while (!exiter.exit()) {
      // simple_serial 的接收已由后台线程处理，不在主循环中读取以避免数据竞争
      
      // 保存当前帧的控制指令（用于PlotJuggler）
      double current_cmd_pitch = 0.0;
      double current_cmd_yaw = 0.0;
      double current_cmd_fire = 0.0;
      double current_cmd_control = 0.0;
      
      // 从相机读取一帧
      camera.read(img, timestamp);
      // 只有在相机已初始化并且帧非空时才检查并调整分辨率
      if (!img.empty()) {
        if (img.cols != desired_w || img.rows != desired_h) {
          cv::resize(img, img, cv::Size(desired_w, desired_h));
        }
      }
      // std::cout<<img.cols<<"*"<<img.rows<<std::endl;  // 注释掉避免拖慢帧率

      auto current_frame_count = frame_count++;

      // 计算帧率
      fps_frame_count++;
      auto fps_time_now = std::chrono::steady_clock::now();
      auto fps_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        fps_time_now - fps_time_start).count();
      if (fps_elapsed >= 1000) {  // 每秒更新一次FPS
        current_fps = fps_frame_count * 1000.0 / fps_elapsed;
        fps_frame_count = 0;
        fps_time_start = fps_time_now;
      }

      // 计算当前时间戳（相对于程序开始）
      double current_time = std::chrono::duration<double>(timestamp - t0).count();
      auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(
        timestamp - t0);
      double t = elapsed_time.count() / 1e6; // 定义t

      /// 自瞄核心逻辑开始

      // 获取云台姿态：优先使用 simple_state（下位机上报的绝对角度），否则使用 gimbal / simple_gimbal / 固定
      bool simple_state_valid = false;
      std::chrono::steady_clock::time_point simple_state_timestamp;
      double yaw_deg = 0.0, pitch_deg = 0.0, roll_deg = 0.0;
      if (simple_state) {
        std::lock_guard<std::mutex> lk(simple_state->m);
        simple_state_valid = simple_state->valid;
        simple_state_timestamp = simple_state->timestamp;
        pitch_deg = simple_state->pitch_deg;
        yaw_deg = simple_state->yaw_deg;
        roll_deg = simple_state->roll_deg;
      }

      if (simple_state && simple_state_valid) {
        // yaw_deg = -yaw_deg;  // 反转 yaw 以匹配上位机坐标系定义
        
        // 调试输出：接收到的原始姿态（每帧都输出）
        // std::cout << "\n=== 云台姿态调试 Frame " << frame_count << " ===" << std::endl;  // 注释掉避免拖慢帧率
        // std::cout << "接收: Pitch=" << pitch_deg << "°, Yaw=" << yaw_deg << "°, Roll=" << roll_deg << "°" << std::endl;
        
        // 检查姿态数据的时效性
        double imu_age = std::chrono::duration<double>(timestamp - simple_state_timestamp).count();
        if (imu_age > 0.05) {  // 超过50ms认为数据过时
          tools::logger()->warn("[Frame {}] IMU数据过时: 延迟={:.1f}ms", frame_count, imu_age * 1000);
        }
        
        const double DEG2RAD = M_PI / 180.0;
        double yaw_rad = yaw_deg * DEG2RAD;
        double pitch_rad = pitch_deg * DEG2RAD;
        double roll_rad = roll_deg * DEG2RAD;
        
        // 尝试不同的旋转顺序（默认使用Z-Y-X外旋）
        // 如果效果不对，可以尝试注释掉当前的，启用下面的其他方式
        
        // 方式1：Z-Y-X 外旋（当前）
        Eigen::AngleAxisd Ra(yaw_rad, Eigen::Vector3d::UnitZ());   // yaw (Z)
        Eigen::AngleAxisd Rb(pitch_rad, Eigen::Vector3d::UnitY()); // pitch (Y)
        Eigen::AngleAxisd Rc(roll_rad, Eigen::Vector3d::UnitX());  // roll (X)
        Eigen::Quaterniond q = Ra * Rb * Rc;
        
        // 方式2：X-Y-Z 外旋（内旋Z-Y-X的逆序）
        // Eigen::Quaterniond q = Rc * Rb * Ra;
        
        // 方式3：直接用欧拉角（Eigen默认Z-Y-X内旋）
        // Eigen::Matrix3d R;
        // R = Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ())
        //   * Eigen::AngleAxisd(pitch_rad, Eigen::Vector3d::UnitY())
        //   * Eigen::AngleAxisd(roll_rad, Eigen::Vector3d::UnitX());
        // Eigen::Quaterniond q(R);
        
        gimbal_q = q;
        
        // 调试输出：计算出的四元数和旋转矩阵（每帧都输出）
        Eigen::Vector3d euler_back = q.toRotationMatrix().eulerAngles(2, 1, 0) * 180.0 / M_PI;
        // std::cout << "四元数: w=" << q.w() << ", x=" << q.x() << ", y=" << q.y() << ", z=" << q.z() << std::endl;  // 注释掉避免拖慢帧率
        // std::cout << "回算欧拉角: Yaw=" << euler_back[0] << "°, Pitch=" << euler_back[1] << "°, Roll=" << euler_back[2] << "°" << std::endl;
      } else if (gimbal) {
        // 完整四元数模式：从 gimbal 获取
        gimbal_q = gimbal->q(timestamp);
      } else if (simple_gimbal) {
        // 角度模式：从 simple_gimbal 获取（内部会将 yaw/pitch 转为四元数）
        gimbal_q = simple_gimbal->q(timestamp);
      } else {
        // 无串口模式：使用固定姿态
        gimbal_q = fixed_quat;
      }

      // 计算云台角速度以检测快速运动
      double dt_gimbal = current_time - last_gimbal_time;
      if (dt_gimbal > 0.001 && last_gimbal_time > 0) {  // 避免除零
        // 计算四元数角度差
        Eigen::Quaterniond q_diff = last_gimbal_q.inverse() * gimbal_q;
        double angle_diff = 2.0 * acos(std::min(1.0, std::abs(q_diff.w())));
        double angular_vel = angle_diff / dt_gimbal;
        
        gimbal_stable = (angular_vel < GIMBAL_ANGULAR_VEL_THRESHOLD);
        
        if (!gimbal_stable) {
          tools::logger()->warn(
            "[Frame {}] 云台快速运动检测: 角速度={:.2f}°/s (阈值={:.2f}°/s)", 
            frame_count, angular_vel * 57.3, GIMBAL_ANGULAR_VEL_THRESHOLD * 57.3);
        }
      }
      last_gimbal_q = gimbal_q;
      last_gimbal_time = current_time;

      // 设置云台姿态到solver
      solver.set_R_gimbal2world({
        gimbal_q.w(),
        gimbal_q.x(),
        gimbal_q.y(),
        gimbal_q.z()
      });

      // 第一步：YOLO目标检测
      auto yolo_start = std::chrono::steady_clock::now();//
      auto armors = yolo.detect(img, current_frame_count);

      auto tracker_start = std::chrono::steady_clock::now();
      
      // 云台快速运动时的特殊处理
      std::list<auto_aim::Target> targets;
      if (!gimbal_stable && !armors.empty()) {
        tools::logger()->warn("[Frame {}] 云台不稳定，跳过tracker更新", frame_count);
        // 云台快速运动时，不更新tracker，使用上一帧的预测
        // 或者可以选择直接清空targets让系统重新锁定
        targets.clear();
      } else {
        targets = tracker.track(armors, timestamp);
      }

      auto plan_start = std::chrono::steady_clock::now();
      
      // 根据模式选择控制策略
      if (use_mpc && planner) {
        // MPC模式 - 传入第一个目标和子弹速度
        std::optional<auto_aim::Target> target_opt;
        if (!targets.empty()) {
          target_opt = targets.front();
        }
        auto plan = planner->plan(target_opt, bullet_speed);
        
        // 保存控制指令到变量
        current_cmd_pitch = plan.pitch * 57.3;  // 转为度
        current_cmd_yaw = plan.yaw * 57.3;
        current_cmd_fire = plan.fire ? 1.0 : 0.0;
        current_cmd_control = plan.control ? 1.0 : 0.0;
        
// #ifdef HAS_ROS2
//         // 发布 ROS2 消息
//         if (plan.control) {
//           auto msg = std_msgs::msg::Float64MultiArray();
//           msg.data = {plan.pitch, plan.yaw, (double)plan.fire, 
//                       plan.pitch_vel, plan.yaw_vel,
//                       plan.pitch_acc, plan.yaw_acc};
//           gimbal_cmd_pub->publish(msg);
//         }
// #endif
        
        // 发送或打印MPC控制指令
        if (use_serial && gimbal && !simple_serial && !angle_mode) {
          // 完整四元数模式：发送位置+速度+加速度
          gimbal->send(
            plan.control, plan.fire,
            plan.yaw, plan.yaw_vel, plan.yaw_acc,
            plan.pitch, plan.pitch_vel, plan.pitch_acc
          );
        } else if (use_serial && simple_gimbal && angle_mode) {
          // 角度模式：发送位置+速度+加速度（SimpleGimbal自动处理）
          simple_gimbal->send(
            plan.control, plan.fire,
            plan.yaw, plan.yaw_vel, plan.yaw_acc,
            plan.pitch, plan.pitch_vel, plan.pitch_acc
          );
        } else if (simple_serial && simple_serial_inst && simple_serial_inst->is_open()) {
          float send_pitch = plan.control ? plan.pitch * 57.3 : 0.0f;
          float send_yaw = plan.control ? plan.yaw * 57.3 : 0.0f;
          int send_fire = plan.control && plan.fire ? 1 : 0;
          simple_serial_inst->send_command(send_pitch, send_yaw, send_fire, current_frame_count);
          if (current_frame_count % 30 == 0) {
            std::cout << fmt::format(
              "[Simple Serial] 发送 (Pitch: {:.2f}°, Yaw: {:.2f}°, Fire: {}, Frame: {}, Control: {})\n",
              send_pitch, send_yaw, send_fire, current_frame_count, plan.control
            );
          }
        } else if (plan.control) {
          // 控制台输出
          std::cout << fmt::format(
            "[MPC] Control: {}, Fire: {}, Yaw: {:.3f}rad ({:.2f}°), YawVel: {:.3f}, YawAcc: {:.3f}, "
            "Pitch: {:.3f}rad ({:.2f}°), PitchVel: {:.3f}, PitchAcc: {:.3f}\n",
            plan.control, plan.fire,
            plan.yaw, plan.yaw * 57.3, plan.yaw_vel, plan.yaw_acc,
            plan.pitch, plan.pitch * 57.3, plan.pitch_vel, plan.pitch_acc
          );
        }
        
      } else if (aimer.get()) {
        // 传统Aimer模式
        auto command = aimer->aim(targets, timestamp, bullet_speed, false);

        // 使用Shooter的专业开火判断（类似sentry.cpp）
        Eigen::Vector3d gimbal_pos = tools::eulers(gimbal_q.toRotationMatrix(), 2, 1, 0);
        command.shoot = shooter.shoot(command, *aimer, targets, gimbal_pos);

        // 应用三层滤波处理，减少静止和高速移动时的指令抖动
        if (command.control) {
          double target_speed = 0.0;
          if (!targets.empty()) {
            auto& target = targets.front();
            Eigen::VectorXd x = target.ekf_x();
            double vx = x[1], vy = x[3], vz = x[5];
            target_speed = std::sqrt(vx*vx + vy*vy + vz*vz);
          }
          aim_filter.filter(command.pitch * 57.3, command.yaw * 57.3, target_speed, last_command, command, frame_count);
        }

        // 保存控制指令到变量
        current_cmd_pitch = command.pitch * 57.3;  // 转为度
        current_cmd_yaw = command.yaw * 57.3;
        current_cmd_fire = command.shoot ? 1.0 : 0.0;
        current_cmd_control = command.control ? 1.0 : 0.0;
        
        // 发送控制指令（无论是否有目标都发送，无目标时发送心跳包）
        if (command.control) {
          last_command = command;
          
// #ifdef HAS_ROS2
//           // 发布 ROS2 消息
//           auto msg = std_msgs::msg::Float64MultiArray();
//           msg.data = {command.pitch, command.yaw, (double)command.shoot};
//           gimbal_cmd_pub->publish(msg);
// #endif
        }
        
        if (use_serial && gimbal && !simple_serial && !angle_mode && command.control) {
          // 完整四元数模式（Aimer只有位置，速度加速度为0）
          gimbal->send(
            command.control, command.shoot,
            command.yaw, 0.0f, 0.0f,
            command.pitch, 0.0f, 0.0f
          );
        } else if (use_serial && simple_gimbal && angle_mode && command.control) {
          // 角度模式
          simple_gimbal->send(
            command.control, command.shoot,
            command.yaw, 0.0f, 0.0f,
            command.pitch, 0.0f, 0.0f
          );
        } else if (simple_serial && simple_serial_inst && simple_serial_inst->is_open()) {
          Eigen::Vector3d current_euler = gimbal_q.toRotationMatrix().eulerAngles(2, 1, 0);
          float current_yaw_rad = current_euler[0];
          float current_pitch_rad = current_euler[1];
          float send_pitch = command.control ? (command.pitch - current_pitch_rad) * 57.3 : 0.0f;
          float send_yaw = command.control ? (command.yaw - current_yaw_rad) * 57.3 : 0.0f;
          
          constexpr double AIM_PRECISION_THRESH = 1.0;
          double aim_error = std::hypot(send_pitch, send_yaw);
          bool aim_precise = (aim_error < AIM_PRECISION_THRESH);
          int send_fire = command.control && command.shoot && aim_precise ? 1 : 0;
          
          if (command.control && frame_count % 30 == 0) {
            tools::logger()->info(
              "[瞄准精度] 增量P={:.2f}°, Y={:.2f}°, 总误差={:.2f}°, 阈值={:.2f}°, 精确={}, 允许开火={}",
              send_pitch, send_yaw, aim_error, AIM_PRECISION_THRESH, aim_precise, send_fire
            );
          }
          simple_serial_inst->send_command(send_pitch, send_yaw, send_fire, current_frame_count);
          if (current_frame_count % 30 == 0) {
            std::cout << fmt::format(
              "[Aimer Simple Serial] 发送 (增量 Pitch: {:.2f}°, Yaw: {:.2f}°, Fire: {}, Frame: {}, Control: {})\n",
              send_pitch, send_yaw, send_fire, current_frame_count, command.control
            );
          }
        } else if (command.control) {
          // 控制台输出
          std::cout << fmt::format(
            "[Aimer] Control: {}, Shoot: {}, Yaw: {:.3f}rad ({:.2f}°), Pitch: {:.3f}rad ({:.2f}°)\n",
            command.control, command.shoot,
            command.yaw, command.yaw * 57.3,
            command.pitch, command.pitch * 57.3
          );
        }
      }

      // 调试输出（每30帧打印一次，避免拖慢帧率）
      auto finish = std::chrono::steady_clock::now();
      if (frame_count % 30 == 0) {
        tools::logger()->info(
          "[{}] yolo: {:.1f}ms, tracker: {:.1f}ms, plan/aim: {:.1f}ms", frame_count,
          tools::delta_time(tracker_start, yolo_start) * 1e3,
          tools::delta_time(plan_start, tracker_start) * 1e3,
          tools::delta_time(finish, plan_start) * 1e3);
      }

      // 在图像上绘制调试信息
      std::string mode_info = use_mpc ? "MPC" : "Aimer";
      tools::draw_text(
        img,
        fmt::format("{} mode - Frame: {}", mode_info, frame_count),
        {10, 60}, {154, 50, 205});

      // 显示帧率
      tools::draw_text(
        img,
        fmt::format("FPS: {:.1f}", current_fps),
        {10, 30}, {0, 255, 0});

      tools::draw_text(
        img,
        fmt::format(
          "gimbal yaw{:.2f}", (tools::eulers(gimbal_q.toRotationMatrix(), 2, 1, 0) * 57.3)[0]),
        {10, 90}, {255, 255, 255});

      // 收集数据用于绘图
      nlohmann::json data;

      // 控制指令数据（发送给下位机的指令）
      data["cmd_pitch"] = current_cmd_pitch;
      data["cmd_yaw"] = current_cmd_yaw;
      data["cmd_fire"] = current_cmd_fire;
      data["cmd_control"] = current_cmd_control;

      // 装甲板原始观测数据
      data["armor_num"] = armors.size();
      if (!armors.empty()) {
        const auto & armor = armors.front();
        data["armor_x"] = armor.xyz_in_world[0];
        data["armor_y"] = armor.xyz_in_world[1];
        data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
        data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
        data["armor_center_x"] = armor.center_norm.x;
        data["armor_center_y"] = armor.center_norm.y;
      }

      auto yaw = tools::eulers(gimbal_q, 2, 1, 0)[0];
      data["gimbal_yaw"] = yaw * 57.3;

      if (!targets.empty()) {
        auto target = targets.front();

        if (last_t == -1) {
          last_target = target;
          last_t = t;
          frame_count++;
          continue;
        }

        // 绘制装甲板轮廓（灰色：EKF估算的全部装甲板位置，包括相机看不到的）
        std::vector<Eigen::Vector4d> armor_xyza_list;
        armor_xyza_list = target.armor_xyza_list();
        for (const Eigen::Vector4d & xyza : armor_xyza_list) {
          auto image_points =
            solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
          tools::draw_points(img, image_points, {150, 150, 150});  // 灰色细框=EKF推算位置
        }

        // 绘制瞄准目标（红色粗框）：Aimer选中的装甲板（已包含子弹飞行时间预测offset）
        if (aimer.get()) {
          auto aim_point = aimer->debug_aim_point;
          if (aim_point.valid) {
            auto image_points =
              solver.reproject_armor(aim_point.xyza.head(3), aim_point.xyza[3], target.armor_type, target.name);
            // 红色粗框 = 当前选中的瞄准目标
            tools::draw_points(img, image_points, {0, 0, 255}, 3);
            if (image_points.size() >= 4) {
              cv::Point2f center(0, 0);
              for (const auto & pt : image_points) { center.x += pt.x; center.y += pt.y; }
              center.x /= image_points.size();
              center.y /= image_points.size();
              cv::drawMarker(img, center, {0, 0, 255}, cv::MARKER_CROSS, 24, 2);
              cv::putText(img, "AIM", cv::Point(center.x + 10, center.y - 10),
                         cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 0, 255}, 2);
            }
          }
        } else if (planner.get() && !target.armor_xyza_list().empty()) {
          // MPC模式：用第一个装甲板作为目标
          auto predict_xyza = target.armor_xyza_list().front();
          auto image_points =
            solver.reproject_armor(predict_xyza.head(3), predict_xyza[3], target.armor_type, target.name);
          tools::draw_points(img, image_points, {0, 0, 255}, 3);
          if (image_points.size() >= 4) {
            cv::Point2f center(0, 0);
            for (const auto & pt : image_points) { center.x += pt.x; center.y += pt.y; }
            center.x /= image_points.size();
            center.y /= image_points.size();
            cv::drawMarker(img, center, {0, 0, 255}, cv::MARKER_CROSS, 24, 2);
            cv::putText(img, "MPC TARGET", cv::Point(center.x + 10, center.y - 10),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 0, 255}, 2);
          }
        }

        // 收集观测器数据
        Eigen::VectorXd x = target.ekf_x();
        data["x"] = x[0];
        data["vx"] = x[1];
        data["y"] = x[2];
        data["vy"] = x[3];
        data["z"] = x[4];
        data["vz"] = x[5];
        data["a"] = x[6] * 57.3;
        data["w"] = x[7];
        data["r"] = x[8];
        data["l"] = x[9];
        data["h"] = x[10];
        data["last_id"] = target.last_id;

        // 收集卡方检验数据
        data["residual_yaw"] = target.ekf().data.at("residual_yaw");
        data["residual_pitch"] = target.ekf().data.at("residual_pitch");
        data["residual_distance"] = target.ekf().data.at("residual_distance");
        data["residual_angle"] = target.ekf().data.at("residual_angle");
        data["nis"] = target.ekf().data.at("nis");
        data["nees"] = target.ekf().data.at("nees");
        data["nis_fail"] = target.ekf().data.at("nis_fail");
        data["nees_fail"] = target.ekf().data.at("nees_fail");
        data["recent_nis_failures"] = target.ekf().data.at("recent_nis_failures");
      }

      // 发送数据给绘图器
      plotter.plot(data);

      // 降低显示频率：每3帧更新一次画面（imshow+waitKey是最大瓶颈，~15-20ms）
      if (display && frame_count % 3 == 0) {
        cv::imshow("reprojection", img);
        cv::waitKey(1);
      }
    }  // end of while

    // Simple Serial will auto cleanup via smart pointer.


  } catch (const YAML::Exception& e) {
    std::cerr << "YAML Exception: " << e.what() << std::endl;
    return 1;
  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
#ifdef HAS_ROS2
    rclcpp::shutdown();
#endif
    return 1;
  }

#ifdef HAS_ROS2
  rclcpp::shutdown();
#endif
  return 0;
}

/*
cmake -B build
make -C build/ -j`nproc`


./build/auto_aim


看进程是否在跑
ps -ef | grep auto_aim | grep -v grep


ros2 run plotjuggler plotjuggler


查看日志：
ls -lt /home/oconnor/Downloads/sp_vision_25-main/logs | head


pkill -f /home/oconnor/Downloads/sp_vision_25-main/build/auto_aim
 */
