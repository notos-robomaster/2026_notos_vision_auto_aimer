// #include <fmt/core.h>
// #include <fstream>  // 添加这个头文件
// #include <chrono>
// #include <nlohmann/json.hpp>
// #include <opencv2/opencv.hpp>
// #include "tasks/auto_aim/solver.hpp"
// #include "io/camera.hpp"
// #include "tasks/auto_aim/aimer.hpp"
// #include "tasks/auto_aim/solver.hpp"
// #include "tasks/auto_aim/tracker.hpp"
// #include "tasks/auto_aim/yolo.hpp"
// #include "tools/exiter.hpp"
// #include "tools/img_tools.hpp"
// #include "tools/logger.hpp"
// #include "tools/math_tools.hpp"
// #include "tools/plotter.hpp"
// #include "yaml-cpp/yaml.h"
//
// const std::string keys =
//   "{help h usage ? |                     | 输出命令行参数说明}"
//   "{config-path c  | configs/camera.yaml | yaml配置文件路径}"
//   "{d display      |                     | 显示视频流       }";
//
//
// int main(int argc, char * argv[])
// {
//   cv::CommandLineParser cli(argc, argv, keys);
//   if (cli.has("help")) {
//     cli.printMessage();
//     return 0;
//   }
//
//   tools::Exiter exiter;
//   tools::Plotter plotter;
//
//   auto config_path = cli.get<std::string>("config-path");
//   auto display = cli.has("display");
//
//   // 在初始化组件之前添加详细的错误处理
//   try {
//
//     std::cout << "All string fields are valid!" << std::endl;
//
//     tools::logger()->info("Starting auto_aim_test with config: {}", config_path);
//
//     // 检查配置文件
//     std::ifstream config_file(config_path);
//     if (!config_file.good()) {
//       tools::logger()->error("Config file not found: {}", config_path);
//       return -1;
//     }
//     config_file.close();
//
//     tools::logger()->info("Step 1: Initializing camera...");
//     io::Camera camera(config_path);
//
//     tools::logger()->info("Step 2: Initializing YOLO...");
//     auto_aim::YOLO yolo(config_path);
//
//     tools::logger()->info("Step 3: Initializing Solver...");
//     auto_aim::Solver solver(config_path);
//
//     tools::logger()->info("Step 4: Initializing Tracker...");
//     auto_aim::Tracker tracker(config_path, solver);
//
//     tools::logger()->info("Step 5: Initializing Aimer...");
//     auto_aim::Aimer aimer(config_path);
//
//     tools::logger()->info("All components initialized successfully!");
//
//     // 从这里开始是原有的主循环代码
//     cv::Mat img(720, 540, CV_8UC3); // 注意：参数顺序是(行, 列, 类型)
//     std::chrono::steady_clock::time_point timestamp;
//     auto last_stamp = std::chrono::steady_clock::now();
//
//     // 自瞄相关的状态变量
//     auto_aim::Target last_target;
//     io::Command last_command;
//     int frame_count = 0;
//
//     // 固定云台姿态（在没有真实IMU数据的情况下使用）
//     Eigen::Quaterniond fixed_quat(1.0, 0.0, 0.0, 0.0); // w=1, x=0, y=0, z=0
//
//     while (!exiter.exit()) {
//       // 从相机读取一帧
//       camera.read(img, timestamp);
//       if (img.cols != 720 || img.rows != 540) {
//         cv::resize(img, img, cv::Size(720,540 ));
//       }
//
//       // 检查图像是否为空来判断读取是否成功
//       if (img.empty()) {
//         tools::logger()->warn("Failed to read from camera - empty image");
//         std::this_thread::sleep_for(std::chrono::milliseconds(10));
//         continue;
//       }
//
//       auto current_frame_count = frame_count++;
//
//       /// 自瞄核心逻辑开始
//
//       // 设置云台姿态（这里使用固定值，在实际应用中应从IMU获取）
//       solver.set_R_gimbal2world({
//         fixed_quat.w(),
//         fixed_quat.x(),
//         fixed_quat.y(),
//         fixed_quat.z()
//       });
//
//       // 第一步：YOLO目标检测
//       auto yolo_start = std::chrono::steady_clock::now();//
//       auto armors = yolo.detect(img, current_frame_count);
//
//       auto detection1 = img.clone();
//       if (!armors.empty()) {
//         armors.sort([](const auto & a, const auto & b) {
//         cv::Point2f img_center(720 / 2, 540 / 2);  // TODO
//         auto distance_1 = cv::norm(a.center - img_center);
//         auto distance_2 = cv::norm(b.center - img_center);
//         return distance_1 < distance_2;
//       });
//
//       // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
//       armors.sort(
//         [](const auto_aim::Armor & a, const auto_aim::Armor & b) { return a.priority < b.priority; });
//       auto & armor = armors.front();  // <-- 直接选择列表的第一个（优先级最高的）
//       // 绘制装甲板四边形
//       for (int i = 0; i < 4; i++) {
//         int j = (i + 1) % 4;
//         std::cout<<"第"<<i<<"个点:"<<armor.points[i]<<std::endl;
//         cv::line(detection1, armor.points[i], armor.points[j],
//                  cv::Scalar(0, 0, 255), 2);
//
//         // 绘制角点
//         cv::circle(detection1, armor.points[i], 5, cv::Scalar(0, 0, 255), -1);
//
//         // 标注点序号
//         cv::putText(detection1, std::to_string(i),
//                    cv::Point(armor.points[i].x + 5, armor.points[i].y - 5),
//                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
//       }
//       imshow("detection1",detection1);
//
//       // 首先确认检测到的点的实际位置
//       // 打印每个点的坐标来判断顺序
//       std::cout << "=== 检测到的角点坐标 ===" << std::endl;
//       for (int i = 0; i < 4; i++) {
//         std::cout << "points[" << i << "]: (" << armor.points[i].x << ", " << armor.points[i].y << ")" << std::endl;
//       }
//
//       // 分析点的位置，找出正确的对应关系
//       // 根据x坐标判断左右，根据y坐标判断上下
//       std::vector<cv::Point2f> sorted_points(4);
//       std::vector<cv::Point2f> pts = armor.points;  // 直接复制vector
//
//       // 找出左侧两个点和右侧两个点
//       std::sort(pts.begin(), pts.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
//         return a.x < b.x;
//       });
//
//       // 左侧两个点
//       cv::Point2f left_top, left_bottom;
//       if (pts[0].y < pts[1].y) {
//         left_top = pts[0];
//         left_bottom = pts[1];
//       } else {
//         left_top = pts[1];
//         left_bottom = pts[0];
//       }
//
//       // 右侧两个点
//       cv::Point2f right_top, right_bottom;
//       if (pts[2].y < pts[3].y) {
//         right_top = pts[2];
//         right_bottom = pts[3];
//       } else {
//         right_top = pts[3];
//         right_bottom = pts[2];
//       }
//
//       // 3D点顺序：左上、右上、右下、左下
//       std::vector<cv::Point3f> object_points;
//       float armor_half_width = 67.5f;   // 小装甲板半宽 mm
//       float armor_half_height = 28.0f;  // 小装甲板半高 mm
//
//       object_points.push_back(cv::Point3f(-armor_half_width, -armor_half_height, 0.f));  // 左上
//       object_points.push_back(cv::Point3f(armor_half_width, -armor_half_height, 0.f));   // 右上
//       object_points.push_back(cv::Point3f(armor_half_width, armor_half_height, 0.f));    // 右下
//       object_points.push_back(cv::Point3f(-armor_half_width, armor_half_height, 0.f));   // 左下
//
//       // 2D点按照相同顺序：左上、右上、右下、左下
//       std::vector<cv::Point2f> image_points;
//       image_points.push_back(left_top);      // 左上
//       image_points.push_back(right_top);     // 右上
//       image_points.push_back(right_bottom);  // 右下
//       image_points.push_back(left_bottom);   // 左下
//
//
//       cv::Mat rvec, tvec;
//
//       cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
//             901.628836608727,    0.0,                 368.99147138139285,
//             0.0,                 903.1485486250545,   281.9078681333959,
//             0.0,                 0.0,                 1.0);
//
//       cv::Mat distortion_coeffs = (cv::Mat_<double>(5, 1) <<
//             -0.447569535375274,
//             -0.000446903384808,
//              0.000278897924377,
//              0.001563208302172,
//              0.0);
//
//         std::cout << "图像分辨率: " << img.cols << "x" << img.rows << std::endl;
//
//
//       cv::solvePnP(object_points, image_points, camera_matrix, distortion_coeffs,
//                    rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
//
//       double distance = cv::norm(tvec);
//       std::cout << "距离: " << distance << " mm" << std::endl;
//
//       }  // end of if (!armors.empty())
//
//       cv::waitKey(1);
//     }  // end of while
//
//   } catch (const YAML::Exception& e) {
//     std::cerr << "YAML Exception: " << e.what() << std::endl;
//     return 1;
//   } catch (const std::exception& e) {
//     std::cerr << "Exception: " << e.what() << std::endl;
//     return 1;
//   }
//
//   return 0;
// }

#include <fmt/core.h>
#include <fstream>  // 添加这个头文件
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include "tasks/auto_aim/solver.hpp"
#include "io/camera.hpp"
#include "io/gimbal/simple_gimbal.hpp"  // 这个会自动包含 gimbal.hpp
#include "io/command.hpp"
#include "serial/serial.h"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
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

using namespace std::chrono_literals;

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

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{config-path c  | configs/demo.yaml | yaml配置文件路径}"
  "{d display      |                     | 显示视频流       }";


int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;

  auto config_path = cli.get<std::string>("config-path");
  auto display = cli.has("display");

  // 变量声明区，保证主循环和初始化都能访问
  std::unique_ptr<auto_aim::Aimer> aimer;
  std::unique_ptr<auto_aim::Planner> planner;
  std::unique_ptr<io::Gimbal> gimbal;
  std::unique_ptr<io::SimpleGimbal> simple_gimbal;
  int simple_serial_fd = -1;
  std::string serial_buffer;
  float received_yaw = 0.0f;
  float received_pitch = 0.0f;
  bool use_mpc = false;
  bool use_serial = false;
  bool simple_serial = false;
  bool angle_mode = false;

  try {
    std::cout << "All string fields are valid!" << std::endl;
    tools::logger()->info("Starting auto_aim_test with config: {}", config_path);
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
    auto_aim::YOLO yolo(config_path);

    tools::logger()->info("Step 3: Initializing Solver...");
    auto_aim::Solver solver(config_path);

    tools::logger()->info("Step 4: Initializing Tracker...");
    auto_aim::Tracker tracker(config_path, solver);

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

    if (use_mpc) {
      tools::logger()->info("Step 5: Initializing Planner (MPC)...");
      planner = std::make_unique<auto_aim::Planner>(config_path);
      if (use_serial && !simple_serial && !angle_mode) {
        tools::logger()->info("Step 6: Initializing Gimbal (Full Serial with Quaternion)...");
        gimbal = std::make_unique<io::Gimbal>(config_path);
      } else if (use_serial && angle_mode) {
        tools::logger()->info("Step 6: Initializing SimpleGimbal (Yaw/Pitch Angle Mode)...");
        simple_gimbal = std::make_unique<io::SimpleGimbal>(config_path);
      } else if (simple_serial) {
        tools::logger()->info("Step 6: Initializing Simple Serial (String format with CRC16)...");
        std::string com_port = cfg["com_port"] ? cfg["com_port"].as<std::string>() : "/dev/ttyUSB0";
        simple_serial_fd = open(com_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (simple_serial_fd == -1) {
          tools::logger()->error("Failed to open serial port: {}", com_port);
          throw std::runtime_error("Serial port open failed");
        }
        struct termios options;
        tcgetattr(simple_serial_fd, &options);
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
        tcsetattr(simple_serial_fd, TCSANOW, &options);
        tools::logger()->info("Simple serial opened on {} (fd={})", com_port, simple_serial_fd);
        std::cout << "[串口调试] 打开串口 fd=" << simple_serial_fd << std::endl;
      } else {
        tools::logger()->info("Step 6: Serial disabled, will output to console");
      }
    } else {
      tools::logger()->info("Step 5: Initializing Aimer (Traditional)...");
      aimer = std::make_unique<auto_aim::Aimer>(config_path);
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

    // 帧率计算变量
    auto fps_time_start = std::chrono::steady_clock::now();
    int fps_frame_count = 0;
    double current_fps = 0.0;

    // 固定云台姿态（在没有真实IMU数据的情况下使用）
    Eigen::Quaterniond fixed_quat(1.0, 0.0, 0.0, 0.0); // w=1, x=0, y=0, z=0
    Eigen::Quaterniond gimbal_q(1.0, 0.0, 0.0, 0.0);
    while (!exiter.exit()) {
      // 从simple_serial接收下位机数据（字符串格式）
      if (simple_serial && simple_serial_fd >= 0) {
        char recv_buf[256];
        int n = read(simple_serial_fd, recv_buf, sizeof(recv_buf) - 1);
        std::cout << "[串口调试] read n=" << n << std::endl;
        if (n > 0) {
          recv_buf[n] = '\0';
          std::cout << "[串口调试] data=" << recv_buf << std::endl;
        }
      }

      // 从相机读取一帧
      camera.read(img, timestamp);
      // 只有在相机已初始化并且帧非空时才检查并调整分辨率
      if (!img.empty()) {
        if (img.cols != desired_w || img.rows != desired_h) {
          cv::resize(img, img, cv::Size(desired_w, desired_h));
        }
      }
      std::cout<<img.cols<<"*"<<img.rows<<std::endl;

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
      auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(
        timestamp - t0);
      double t = elapsed_time.count() / 1e6; // 定义t

      /// 自瞄核心逻辑开始

      // 获取云台姿态
      if (gimbal) {
        // 完整四元数模式：从gimbal获取
        gimbal_q = gimbal->q(timestamp);
      } else if (simple_gimbal) {
        // 角度模式：从simple_gimbal获取（内部会将yaw/pitch转为四元数）
        gimbal_q = simple_gimbal->q(timestamp);
      } else {
        // 无串口模式：使用固定姿态
        gimbal_q = fixed_quat;
      }

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
      auto targets = tracker.track(armors, timestamp);

      auto plan_start = std::chrono::steady_clock::now();

      // 根据模式选择控制策略
      if (use_mpc && planner) {
        // MPC模式 - 传入第一个目标和子弹速度
        std::optional<auto_aim::Target> target_opt;
        if (!targets.empty()) {
          target_opt = targets.front();
        }
        auto plan = planner->plan(target_opt, 27.0);  // 27 m/s子弹速度

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
        } else if (simple_serial && simple_serial_fd >= 0 && plan.control) {
          // 简单模式：字符串格式发送位置和开火标志（AAP%.2fY%.2fF%d + CRC16）
          char frame[64];

          // 1. 格式化数据部分（字符串）
          int len = snprintf(frame, sizeof(frame), "AAP%.2fY%.2fF%d",
                            plan.pitch * 57.3, plan.yaw * 57.3, plan.fire ? 1 : 0);

          // 2. 计算 CRC16
          uint16_t crc = crc16_modbus((uint8_t *)frame, len);

          // 3. 拼接 CRC16 和帧尾
          len += snprintf(frame + len, sizeof(frame) - len, "C%04x%%\n", crc);

          // 4. 发送数据
          int written = write(simple_serial_fd, frame, len);
          if (written != len) {
            tools::logger()->warn("Serial write incomplete: {}/{}", written, len);
          } else {
            std::cout << fmt::format(
              "[Simple Serial] 发送: {} (Pitch: {:.2f}°, Yaw: {:.2f}°, Fire: {})\n",
              std::string(frame, len - 1),
              plan.pitch * 57.3, plan.yaw * 57.3, plan.fire
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
        auto command = aimer->aim(targets, timestamp, 27, false);

        if (
          !targets.empty() && aimer->debug_aim_point.valid &&
          std::abs(command.yaw - last_command.yaw) * 57.3 < 2)
          command.shoot = true;

        if (command.control) {
          last_command = command;

          // 发送控制指令
          if (use_serial && gimbal && !angle_mode) {
            // 完整四元数模式（Aimer只有位置，速度加速度为0）
            gimbal->send(
              command.control, command.shoot,
              command.yaw, 0.0f, 0.0f,
              command.pitch, 0.0f, 0.0f
            );
          } else if (use_serial && simple_gimbal && angle_mode) {
            // 角度模式
            simple_gimbal->send(
              command.control, command.shoot,
              command.yaw, 0.0f, 0.0f,
              command.pitch, 0.0f, 0.0f
            );
          } else {
            // 控制台输出
            std::cout << fmt::format(
              "[Aimer] Control: {}, Shoot: {}, Yaw: {:.3f}rad ({:.2f}°), Pitch: {:.3f}rad ({:.2f}°)\n",
              command.control, command.shoot,
              command.yaw, command.yaw * 57.3,
              command.pitch, command.pitch * 57.3
            );
          }
        }
      }

      // 调试输出
      auto finish = std::chrono::steady_clock::now();
      tools::logger()->info(
        "[{}] yolo: {:.1f}ms, tracker: {:.1f}ms, plan/aim: {:.1f}ms", frame_count,
        tools::delta_time(tracker_start, yolo_start) * 1e3,
        tools::delta_time(plan_start, tracker_start) * 1e3,
        tools::delta_time(finish, plan_start) * 1e3);

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

        // 绘制装甲板轮廓
        std::vector<Eigen::Vector4d> armor_xyza_list;
        armor_xyza_list = target.armor_xyza_list();
        for (const Eigen::Vector4d & xyza : armor_xyza_list) {
          auto image_points =
            solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
          tools::draw_points(img, image_points, {0, 255, 0});
        }

        // 绘制预测装甲板（红色）
        if (aimer.get()) {
          auto aim_point = aimer->debug_aim_point;
          Eigen::Vector4d aim_xyza = aim_point.xyza;
          auto image_points =
            solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
          if (aim_point.valid) {
            // 绘制红色预测装甲板轮廓（加粗线条，更明显）
            tools::draw_points(img, image_points, {0, 0, 255}, 3);

            // 在预测装甲板中心绘制标记
            if (image_points.size() >= 4) {
              cv::Point2f center(0, 0);
              for (const auto& pt : image_points) {
                center.x += pt.x;
                center.y += pt.y;
              }
              center.x /= image_points.size();
              center.y /= image_points.size();

              // 绘制中心十字标记
              cv::drawMarker(img, center, {0, 0, 255}, cv::MARKER_CROSS, 20, 2);

              // 添加文字标注
              cv::putText(img, "PREDICT", cv::Point(center.x + 10, center.y - 10),
                         cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 0, 255}, 2);
            }
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
      cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
      cv::imshow("reprojection", img);

      cv::waitKey(1);
    }  // end of while

    // 关闭简单串口
    if (simple_serial_fd >= 0) {
      close(simple_serial_fd);
      tools::logger()->info("Simple serial port closed");
    }

  } catch (const YAML::Exception& e) {
    std::cerr << "YAML Exception: " << e.what() << std::endl;
    return 1;
  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}


/*
cmake -B build
make -C build/ -j`nproc`
./build/auto_aim_test
 */
//ros2 run plotjuggler plotjuggler