#pragma once 
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <chrono>
namespace ImgSetting {
enum class ImgFormat {
  NV12 = 0,
  BGR = 1,
};

struct ImgResolution {
  size_t w{};
  size_t h{};
  size_t len{};
};
struct ImgTopicConfig {
  std::string name;
  std::string topic_name;
  ImgSetting::ImgFormat format;
  ImgSetting::ImgResolution resolution;
  std::chrono::milliseconds cycle_ms;
  bool dump_img;
  bool show_img;
};
} // namespace ImgSetting
