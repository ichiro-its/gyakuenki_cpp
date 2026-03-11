// File: src/ekf_test_main.cpp
#include <memory>
#include <string>

#include "gyakuenki_cpp/node/ekf_test_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("ekf_test_node");

  std::string config_path = "";

  if (argc > 1) {
    config_path = argv[1];
    RCLCPP_INFO(node->get_logger(), "load config from: %s", config_path.c_str());
  } else {
    RCLCPP_ERROR(node->get_logger(), "config not found!");
    rclcpp::shutdown();
    return 1;
  }

  try {
    auto ekf_test = std::make_shared<gyakuenki_cpp::EkfTestNode>(node, config_path);

    RCLCPP_INFO(node->get_logger(), "EKF Ball Tracking Node has been started.");
    RCLCPP_INFO(node->get_logger(), "Press Ctrl+C to stop...");

    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();

  return 0;
}