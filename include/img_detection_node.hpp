#include <vector>
#include "img_detection.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "common.h"
#include <vector>
class ImgDetectionNode : public rclcpp::Node  {
  // using namespace ImgSetting;
public:
private:
  /* data */
  using UPtrImgDetction = std::unique_ptr<ImgDetection>;
  std::vector<UPtrImgDetction> detctors_list;
  std::vector<ImgSetting::ImgTopicConfig> detectors_config_list;
  rclcpp::TimerBase::SharedPtr img_detection_timer{nullptr};
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client;
public:
  ImgSetting::ImgTopicConfig config;
  ImgDetectionNode(const std::string &node_name, rclcpp::NodeOptions opts);
  virtual ~ImgDetectionNode();
};


ImgDetectionNode::ImgDetectionNode(
    const std::string &node_name,
    rclcpp::NodeOptions opts = rclcpp::NodeOptions{})
    : rclcpp::Node{node_name, opts} {
  // return;
  ImgSetting::ImgTopicConfig config_1;
  config_1.name = "cam2";
  config_1.show_img = true;
  config_1.resolution.h = 720;
  config_1.resolution.w = 1280;
  config_1.topic_name = "/pic/rtsp_cam2_nv12";
  config_1.cycle_ms = std::chrono::milliseconds{1000};
  config_1.format = ImgSetting::ImgFormat::NV12;
  ImgSetting::ImgTopicConfig config_2;
  config_2.name = "cam1";
  config_2.resolution.w = 1920;
  config_2.resolution.h = 1080;
  config_2.show_img = true;
  config_2.topic_name = "/pic/rtsp_cam1_nv12";
  config_2.cycle_ms = std::chrono::milliseconds{1000};
  config_2.format = ImgSetting::ImgFormat::NV12;
  detectors_config_list.emplace_back(config_1);
  detectors_config_list.emplace_back(config_2);
  for (const auto &config : detectors_config_list) {
    auto sub_node = this->create_sub_node(config.name);
    UPtrImgDetction detector =
        std::make_unique<ImgDetection>(sub_node, config);
    detctors_list.emplace_back(std::move(detector));
  }
}

ImgDetectionNode::~ImgDetectionNode() {}
