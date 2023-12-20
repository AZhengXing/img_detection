#include "common.h"
#include <functional>
#include <queue>
#include <mutex>
#include "std_msgs/msg/byte_multi_array.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "ai_service_msg/srv/img_detection_srv.hpp"
#include "ai_service_msg/msg/img_detection_output.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include <atomic>
#include "video_utils.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "cv_bridge/cv_bridge.h"
class ImgDetection {
public:
  ImgDetection(rclcpp::Node::SharedPtr sub_node_,
               const ImgSetting::ImgTopicConfig &config_);
  ~ImgDetection() {}

private:
  using Msg = std_msgs::msg::UInt8MultiArray;
  using IMGDetectionSrv = ai_service_msg::srv::IMGDetectionSrv;
  using ConstSharedPtr = std_msgs::msg::UInt8MultiArray::ConstSharedPtr;
  rclcpp::Node::SharedPtr sub_node;
  rclcpp::CallbackGroup::SharedPtr service_cb_group;
  std::string sub_node_name;
  ImgSetting::ImgTopicConfig config;
  rclcpp::Subscription<Msg>::ConstSharedPtr ros_img_sub{nullptr};

  rclcpp::Client<IMGDetectionSrv>::SharedPtr pClient;
  std::atomic_bool has_finish_request{true};
  std::atomic_int pic_number{0};
  rclcpp::TimerBase::SharedPtr img_detection_timer{nullptr};
  
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr bgr_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr nv12_image_pub_;
  sensor_msgs::msg::Image ros_brg_img_msg;
  sensor_msgs::msg::Image ros_nv12_img_msg;
  
  rclcpp::SubscriptionHbmem<Msg>::ConstSharedPtr hbm_img_sub{nullptr};

  void check_param();
  void img_detection_cb();
  void img_get_cb(ConstSharedPtr data);

  std::mutex mx;
  const size_t max_msg_q_size{4};
  std::queue<ConstSharedPtr> msg_q;
};

void ImgDetection::img_detection_cb() {
  
  ConstSharedPtr msg;
  {
    std::lock_guard<std::mutex> lg{mx};
    if (msg_q.empty() == false) {
      msg = msg_q.front();
      msg_q.pop();
    }
  }
  if (msg != nullptr) {
    const auto &raw_img = msg->data;
    // bgr_mat.
    if (config.format != ImgSetting::ImgFormat::NV12) {
      return;
    }
    if (!pClient->service_is_ready()) {
      RCLCPP_WARN(sub_node->get_logger(), "server is not available");
      has_finish_request = true;
      return;
    }
    if (has_finish_request == false) {
      return;
    }
    pic_number++;
    RCLCPP_DEBUG(sub_node->get_logger(), "Cam %s Detection start! pic num: %d",
                sub_node_name.c_str(), pic_number.load());
    has_finish_request = false;
    auto detection_msg = sensor_msgs::msg::Image();
    auto data_len = config.resolution.h * config.resolution.w * 3 / 2;
    detection_msg.height = config.resolution.h;
    detection_msg.width = config.resolution.w;
    detection_msg.encoding = "nv12";
    detection_msg.data.resize(data_len);
    memcpy(&detection_msg.data[0], raw_img.data(), data_len);
    auto requ = std::make_shared<IMGDetectionSrv::Request>();
    requ->img = detection_msg;
    RCLCPP_DEBUG(sub_node->get_logger(), "Cam %s call server. pic num: %d",
                sub_node_name.c_str(), pic_number.load());
    (void)pClient->async_send_request(
        requ,
        [msg, this](rclcpp::Client<IMGDetectionSrv>::SharedFuture result) {
          auto ret = result.get();
          auto goals = ret->goals;
          RCLCPP_DEBUG(sub_node->get_logger(),
                      "Cam %s Detection end!  pic num: %d",
                      sub_node_name.c_str(), pic_number.load());
          const auto &raw_img = msg->data;
          if (config.show_img) {
            cv::Mat bgr_mat(config.resolution.h, config.resolution.w, CV_8UC3);
            video_utils::NV12_TO_BGR24((unsigned char *)raw_img.data(), nullptr,
                                       bgr_mat.ptr<unsigned char>(),
                                       config.resolution.w,
                                       config.resolution.h);
            for (auto &goal : goals) {
              video_utils::draw_object(bgr_mat, goal);
            }
            cv_bridge::CvImage(std_msgs::msg::Header{}, "bgr8", bgr_mat)
                .toImageMsg(ros_brg_img_msg);
            bgr_image_pub_->publish(ros_brg_img_msg);
          }
          for (auto &goal : goals) {
            RCLCPP_DEBUG(sub_node->get_logger(),
                        "Cam %s Detection end!  pic num: %d. class %s",
                        sub_node_name.c_str(), pic_number.load(),
                        goal.name.c_str());
          }
          has_finish_request = true;
        });
  }
}

void ImgDetection::img_get_cb(ConstSharedPtr data) {
  std::lock_guard<std::mutex> lg{mx};
  if (max_msg_q_size > msg_q.size()) {
    msg_q.push(data);
  } else {
    msg_q.pop();
  }
}

ImgDetection::ImgDetection(rclcpp::Node::SharedPtr sub_node_,
                           const ImgSetting::ImgTopicConfig &config_)
    : sub_node{sub_node_}, 
      sub_node_name{config_.name}, 
      config{config_} {

  service_cb_group =
      sub_node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    
  ros_img_sub = sub_node->create_subscription<Msg>(
      config.topic_name, 1,
      std::bind(&ImgDetection::img_get_cb, this, std::placeholders::_1));
  // auto qos_setting = rmw_qos_profile_services_default; 
  // qos_setting.depth = 1;

  pClient = sub_node->create_client<IMGDetectionSrv>(
      "/ai_server", rmw_qos_profile_services_default, service_cb_group);

  img_detection_timer = sub_node->create_wall_timer(
      config.cycle_ms, std::bind(&ImgDetection::img_detection_cb, this),
      service_cb_group);

  bgr_image_pub_ =
      sub_node->create_publisher<sensor_msgs::msg::Image>("bgr8", 3);

  // nv12_image_pub_ =
  //     sub_node->create_publisher<sensor_msgs::msg::Image>("nv12", 2);
}
