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

#include "keisan/geometry/point_3.hpp"
#include "keisan/matrix.hpp"
#include "ninshiki_interfaces/msg/detected_object.hpp"

namespace gyakuenki_cpp
{

class IPM
{
public:
  enum { TYPE_DNN, TYPE_COLOR };

  struct ProjectedObject
  {
    keisan::Point3d position;
    double confidence;
    std::string label;
  };

  IPM(
    const std::shared_ptr<rclcpp::Node> & node, const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
    const std::shared_ptr<tf2_ros::TransformListener> & tf_listener);

  keisan::rotation_matrix get_rotation_matrix(const geometry_msgs::msg::Quaternion & q);
  const cv::Point & get_object_projection_point();
  const ProjectedObject & map_object(
    const ninshiki_interfaces::msg::DetectedObject & detected_object, int object_type);

  void load_camera_info_configuration(const std::string & path);

private:
  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;

  keisan::Matrix<3, 3> camera_matrix;           // Camera intrinsic matrix
  std::vector<double> distortion_coefficients;  // Camera distortion cofficients
};

}  // namespace gyakuenki_cpp

#endif  // GYAKUENKI_CPP__IPM__GYAKUENKI_CPP_IPM_HPP_
