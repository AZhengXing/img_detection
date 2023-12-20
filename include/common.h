#pragma once 
#include "rclcpp/rclcpp.hpp"
#include <string>
namespace ImgSetting {
enum class ImgFormat {
  NV12 = 0,
  BGR = 1,
};

struct ImgResolution {
  size_t w{};
  size_t h{};
};
struct ImgTopicConfig {
  std::string name;
  std::string topic_name;
  ImgSetting::ImgFormat format;
  ImgSetting::ImgResolution resolution;
  size_t freq;
  bool dump_pic;
};
} // namespace ImgSetting
