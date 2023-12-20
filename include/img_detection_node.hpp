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

public:
  ImgSetting::ImgTopicConfig config;

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_main_node_base_interface()  {
    return this->get_node_base_interface();
  }

  auto get_sub_node_base_interface() {
    std::vector<rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>
        sub_nodes;
    for (const auto &detecor : detctors_list) {
      sub_nodes.push_back(detecor->get_node_base_interface());
    }
    return sub_nodes;
  }

  ImgDetectionNode(const std::string &node_name, rclcpp::NodeOptions opts);
  virtual ~ImgDetectionNode();
};

ImgDetectionNode::ImgDetectionNode(
    const std::string &node_name,
    rclcpp::NodeOptions opts = rclcpp::NodeOptions{})
    : rclcpp::Node{node_name, opts} {
  ImgSetting::ImgTopicConfig config_1;
  config.name = "cam2";
  config.topic_name = "/pic/rtsp_cam2_nv12";
  config.freq = 1;
  config.format = ImgSetting::ImgFormat::NV12;
  ImgSetting::ImgTopicConfig config_2;
  config.name = "cam1";
  config.topic_name = "/pic/rtsp_cam1_nv12";
  config.freq = 1;
  config.format = ImgSetting::ImgFormat::NV12;
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
