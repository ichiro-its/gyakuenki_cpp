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

#ifndef GYAKUENKI_CPP__NODE__GYAKUENKI_CPP_NODE_HPP_
#define GYAKUENKI_CPP__NODE__GYAKUENKI_CPP_NODE_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "gyakuenki_cpp/projections/ipm.hpp"
#include "gyakuenki_interfaces/msg/projected_object.hpp"
#include "gyakuenki_interfaces/msg/projected_objects.hpp"
#include "ninshiki_interfaces/msg/detected_objects.hpp"

namespace gyakuenki_cpp
{

class GyakuenkiCppNode
{
public:
  using DetectedObjects = ninshiki_interfaces::msg::DetectedObjects;
  using ProjectedObjects = gyakuenki_interfaces::msg::ProjectedObjects;
  using ProjectedObject = gyakuenki_interfaces::msg::ProjectedObject;

  GyakuenkiCppNode(const std::shared_ptr<rclcpp::Node> & node, const std::string & path);

private:
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::shared_ptr<gyakuenki_cpp::IPM> ipm;

  rclcpp::Publisher<ProjectedObjects>::SharedPtr projected_objects_publisher;
  rclcpp::Subscription<DetectedObjects>::SharedPtr dnn_detection_subscriber;
};

}  // namespace gyakuenki_cpp

#endif  // GYAKUENKI_CPP__NODE__GYAKUENKI_CPP_NODE_HPP_
