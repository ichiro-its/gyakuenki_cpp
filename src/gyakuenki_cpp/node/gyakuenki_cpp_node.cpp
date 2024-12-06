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

#include "gyakuenki_cpp/node/gyakuenki_cpp_node.hpp"

namespace gyakuenki_cpp
{

GyakuenkiCppNode::GyakuenkiCppNode(
  const std::shared_ptr<rclcpp::Node> & node, const std::string & config_path)
: node(node)
{
  tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node, false);

  ipm = std::make_shared<IPM>(node, tf_buffer, tf_listener, config_path);

  dnn_detection_subscriber = node->create_subscription<DetectedObjects>(
    "ninshiki_cpp/dnn_detection", 10, [this](const DetectedObjects::SharedPtr message) {
      ProjectedObjects protected_objects;

      for (const auto & detected_object : message->detected_objects) {
        ProjectedObject projected_object =
          this->ipm->map_object(detected_object, IPM::TYPE_DNN, "base_footprint");
        projected_objects.push_back(projected_object);
      }

      projected_objects_publisher->publish(projected_objects);
    });
}

}  // namespace gyakuenki_cpp
