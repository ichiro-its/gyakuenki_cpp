// Copyright (c) 2024 Ichiro ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <iostream>
#include <memory>

#include "gyakuenki_cpp/gyakuenki_cpp.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "Usage: ros2 run gyakuenki_cpp gyakuenki_cpp_node <config_path>" << std::endl;
    return 1;
  }

  const std::string & path = argv[1];

  auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("gyakuenki_cpp");

  try {
    auto gyakuenki_cpp_node = std::make_shared<gyakuenki_cpp::GyakuenkiCppNode>(node, path);

    RCLCPP_INFO(node->get_logger(), "GyakuenkiCpp Node has been started.");
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