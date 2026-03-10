#include "mindvision.hpp"

#include <libusb-1.0/libusb.h>

#include <stdexcept>

#include "tools/logger.hpp"

using namespace std::chrono_literals;

namespace io
{
MindVision::MindVision(double exposure_ms, double gamma, const std::string & vid_pid)
: exposure_ms_(exposure_ms),
  gamma_(gamma),
  handle_(-1),
  quit_(false),
  ok_(false),
  queue_(1),
  vid_(-1),
  pid_(-1)
{
  set_vid_pid(vid_pid);
  if (libusb_init(NULL)) tools::logger()->warn("Unable to init libusb!");

  try_open();

  // 守护线程
  daemon_thread_ = std::thread{[this] {
    while (!quit_) {
      std::this_thread::sleep_for(100ms);

      if (ok_) continue;

      if (capture_thread_.joinable()) capture_thread_.join();

      close();
      reset_usb();
      try_open();
    }
  }};
}

MindVision::~MindVision()
{
  quit_ = true;
  if (daemon_thread_.joinable()) daemon_thread_.join();
  if (capture_thread_.joinable()) capture_thread_.join();
  close();
  tools::logger()->trace("Mindvision destructed.");
}

void MindVision::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  CameraData data;
  queue_.pop(data);

  img = data.img;
  timestamp = data.timestamp;
}

void MindVision::open()
{
  int camera_num = 1;
  tSdkCameraDevInfo camera_info_list;
  tSdkCameraCapbility camera_capbility;
  CameraSdkInit(1);
  CameraEnumerateDevice(&camera_info_list, &camera_num);

  if (camera_num == 0) throw std::runtime_error("Not found camera!");

  if (CameraInit(&camera_info_list, -1, -1, &handle_) != CAMERA_STATUS_SUCCESS)
    throw std::runtime_error("Failed to init camera!");

  CameraGetCapability(handle_, &camera_capbility);
  width_ = camera_capbility.sResolutionRange.iWidthMax;
  height_ = camera_capbility.sResolutionRange.iHeightMax;

  CameraSetAeState(handle_, TRUE);   // 开启自动曝光（优先）
  // 如需强制手动曝光，可注释上一行并用下面更大值测试：
  // CameraSetAeState(handle_, FALSE);

  CameraSetGamma(handle_, gamma_ * 1e2);                   // 设置伽马
  CameraSetTriggerMode(handle_, 0);                        // 连续采集
  CameraSetFrameSpeed(handle_, 1);                         // 低帧率/稳定模式
  // CameraSetExposureTime(handle_, 10000); // 单位视 SDK 文档（us）
  // CameraSetGain(handle_, 10,10,10);
  CameraPlay(handle_);
  // 要求 SDK 输出彩色 BGR8（放在 CameraPlay 之后）
  CameraSetIspOutFormat(handle_, CAMERA_MEDIA_TYPE_BGR8);

  // 取图线程（循环获取帧）
  capture_thread_ = std::thread{[this] {
    while (!quit_) {
      tSdkFrameHead head;
      BYTE * raw = nullptr;
      auto status = CameraGetImageBuffer(handle_, &head, &raw, 100);
      auto timestamp = std::chrono::steady_clock::now();

      if (status != CAMERA_STATUS_SUCCESS) {
        tools::logger()->warn("CameraGetImageBuffer failed: {}", status);
        std::this_thread::sleep_for(10ms);
        continue;
      }


      // 1) 使用独立字节缓冲让 SDK 写入，BGR8 为 3 字节每像素
      size_t expects = static_cast<size_t>(head.iWidth) * static_cast<size_t>(head.iHeight) * 3;
      std::vector<BYTE> buf(expects);
      int proc_status = CameraImageProcess(handle_, raw, buf.data(), &head);
      CameraReleaseImageBuffer(handle_, raw);
      if (proc_status != CAMERA_STATUS_SUCCESS) {
        tools::logger()->error("CameraImageProcess failed: {}", proc_status);
        std::this_thread::sleep_for(10ms);
        continue;
      }
      cv::Mat img(head.iHeight, head.iWidth, CV_8UC3, buf.data());
      cv::Mat img_cloned = img.clone();
      // 可选诊断
      cv::Mat gray; cv::cvtColor(img_cloned, gray, cv::COLOR_BGR2GRAY);
      double minv, maxv; cv::minMaxLoc(gray, &minv, &maxv);
      double meanv = cv::mean(gray)[0];
      // 改为 trace，避免控制台被刷屏
      tools::logger()->trace("proc ok, mean={}, min={}, max={}", meanv, minv, maxv);
      queue_.push({img_cloned, timestamp});
    }
  }};

  tools::logger()->debug("Mindvision opened.");
}

void MindVision::try_open()
{
  try {
    open();
  } catch (const std::exception & e) {
    // tools::logger()->warn("{}", e.what());
  }
}

void MindVision::close()
{
  if (handle_ == -1) return;
  CameraUnInit(handle_);
}

void MindVision::set_vid_pid(const std::string & vid_pid)
{
  auto index = vid_pid.find(':');
  if (index == std::string::npos) {
    // tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
    return;
  }

  auto vid_str = vid_pid.substr(0, index);
  auto pid_str = vid_pid.substr(index + 1);

  try {
    vid_ = std::stoi(vid_str, 0, 16);
    pid_ = std::stoi(pid_str, 0, 16);
  } catch (const std::exception &) {
    // tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
  }
}

void MindVision::reset_usb() const
{
  if (vid_ == -1 || pid_ == -1) return;

  // https://github.com/ralight/usb-reset/blob/master/usb-reset.c
  auto handle = libusb_open_device_with_vid_pid(NULL, vid_, pid_);
  if (!handle) {
    // tools::logger()->warn("Unable to open usb!");
    return;
  }

  int ret = libusb_reset_device(handle);
  if (ret != 0) {
    // tools::logger()->warn("Unable to reset usb! ret={}", ret);
  } else {
    // tools::logger()->info("Reset usb successfully :)");
  }

  libusb_close(handle);
}
}  // namespace io