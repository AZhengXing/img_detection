#include "common.h"
#include <functional>
#include <queue>
#include <mutex>
#include "std_msgs/msg/byte_multi_array.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
class ImgDetection {
public:
  ImgDetection(rclcpp::Node::SharedPtr sub_node_,
               const ImgSetting::ImgTopicConfig &config_);
  ~ImgDetection() {}

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface() const;

private:
  using Msg = std_msgs::msg::UInt8MultiArray;
  using ConstSharedPtr = std_msgs::msg::UInt8MultiArray::ConstSharedPtr;
  rclcpp::Node::SharedPtr sub_node;
  std::string sub_node_name;
  ImgSetting::ImgTopicConfig config;
  rclcpp::TimerBase::SharedPtr img_detection_timer{nullptr};
  rclcpp::Subscription<Msg>::ConstSharedPtr ros_img_sub{nullptr};
  rclcpp::SubscriptionHbmem<Msg>::ConstSharedPtr hbm_img_sub{nullptr};

  void img_detection_cb();
  void img_get_cb(ConstSharedPtr data);

  std::mutex mx;
  const size_t max_msg_q_size{4};
  std::queue<ConstSharedPtr> msg_q;
};

void ImgDetection::img_detection_cb() {
  RCLCPP_ERROR(sub_node->get_logger(), "[%s]timer get data", sub_node_name);
  ConstSharedPtr msg;
  {
    std::lock_guard<std::mutex> lg{mx};
    if (msg_q.empty() == false) {
      msg = msg_q.front();
      msg_q.pop();
    }
  }
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
ImgDetection::get_node_base_interface() const {
  return this->sub_node->get_node_base_interface();
}

void ImgDetection::img_get_cb(ConstSharedPtr data) {
  RCLCPP_ERROR(sub_node->get_logger(), "[%s]get data", sub_node_name);
  std::lock_guard<std::mutex> lg{mx};
  if (max_msg_q_size > msg_q.size()) {
    msg_q.emplace(data);
  }
}

ImgDetection::ImgDetection(rclcpp::Node::SharedPtr sub_node_,
                           const ImgSetting::ImgTopicConfig &config_)
    : sub_node{sub_node_}, 
      sub_node_name{sub_node->get_name()}, 
      config{config_} {
  if (config_.freq == 0 || config_.freq >= 10) {
    return;
  }

  auto dt = 1.0 / config_.freq;
  img_detection_timer = sub_node->create_wall_timer(
      std::chrono::milliseconds(static_cast<int64_t>(dt * 1000)),
      std::bind(&ImgDetection::img_detection_cb, this));

  ros_img_sub = sub_node->create_subscription<Msg>(
      config.topic_name, 1,
      std::bind(&ImgDetection::img_get_cb, this, std::placeholders::_1));
}
