#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "img_detection_node.hpp"
#include "common.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::string node_name{"img_detection_node"};
  RCLCPP_WARN(rclcpp::get_logger("Detection"), "Img Detection Start: %s.",
              node_name.data());
  rclcpp::executors::MultiThreadedExecutor exec;
  ImgDetectionNode main_node{"main_node"};
  exec.add_node(main_node.get_node_base_interface());
  exec.spin();
  RCLCPP_WARN(rclcpp::get_logger("Detection"), "Img Detection End: %s.",
              node_name.data());
  return 0;
}
