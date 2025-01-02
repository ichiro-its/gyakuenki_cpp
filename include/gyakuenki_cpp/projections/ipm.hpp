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

#ifndef GYAKUENKI_CPP__IPM__GYAKUENKI_CPP_IPM_HPP_
#define GYAKUENKI_CPP__IPM__GYAKUENKI_CPP_IPM_HPP_

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/quaternion.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include "gyakuenki_cpp/utils/camera_info.hpp"
#include "gyakuenki_interfaces/msg/projected_object.hpp"
#include "keisan/matrix.hpp"
#include "ninshiki_interfaces/msg/detected_object.hpp"

namespace gyakuenki_cpp
{

class IPM
{
public:
  using DetectedObject = ninshiki_interfaces::msg::DetectedObject;
  using Quaternion = geometry_msgs::msg::Quaternion;

  enum { TYPE_DNN, TYPE_COLOR };

  IPM(
    const std::shared_ptr<rclcpp::Node> & node, const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
    const std::shared_ptr<tf2_ros::TransformListener> & tf_listener, const std::string & path);

  bool object_at_bottom_of_image(const DetectedObject & detected_object, int detection_type);
  void normalize_pixel(cv::Point & pixel);
  void denormalize_pixel(cv::Point & pixel);
  void undistort_pixel(cv::Point & pixel);

  keisan::Matrix<4, 4> quat_to_rotation_matrix(const Quaternion & q);

  keisan::Matrix<4, 1> point_in_camera_frame(
    const cv::Point & pixel, const keisan::Matrix<4, 4> & T, const keisan::Matrix<4, 4> & R,
    const std::string & object_label);

  cv::Point get_target_pixel(const DetectedObject & detected_object, int detection_type);
  cv::Point get_normalized_target_pixel(const DetectedObject & detected_object, int detection_type);

  gyakuenki_interfaces::msg::ProjectedObject map_object(
    const DetectedObject & detected_object, int detection_type, const std::string & output_frame);

private:
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;

  utils::CameraInfo camera_info;
};

}  // namespace gyakuenki_cpp

#endif  // GYAKUENKI_CPP__IPM__GYAKUENKI_CPP_IPM_HPP_
