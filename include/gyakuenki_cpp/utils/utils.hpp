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

#ifndef GYAKUENKI_CPP__UTILS__GYAKUENKI_CPP_UTILS_HPP_
#define GYAKUENKI_CPP__UTILS__GYAKUENKI_CPP_UTILS_HPP_

#include <geometry_msgs/msg/quaternion.hpp>
#include <opencv2/opencv.hpp>

#include "ninshiki_interfaces/msg/detected_object.hpp"

namespace gyakuenki_cpp::utils
{

using DetectedObject = ninshiki_interfaces::msg::DetectedObject;
using Quaternion = geometry_msgs::msg::Quaternion;

struct CameraInfo
{
  keisan::Matrix<3, 3> K = keisan::Matrix::zero();  // Intrinsic matrix
  keisan::Matrix<5, 1> D = keisan::Matrix::zero();  // Distortion coefficients
  std::string frame_id = "camera";                  // Camera frame id
  int image_width = 320;                            // Image width
  int image_height = 240;                           // Image height
  bool use_distortion = false;                      // Use distortion
};

enum { TYPE_DNN, TYPE_COLOR };

const CameraInfo & load_camera_info(const std::string & config_path);
const keisan::rotation_matrix & quat_to_rotation_matrix(const geometry_msgs::msg::Quaternion & q);
const cv::Point & get_projection_pixels(const DetectedObject & detected_object, int object_type);
double get_height_offset(const std::string & label, int object_type);
bool object_at_bottom_of_image(const cv::Point & point, int image_height);

}  // namespace gyakuenki_cpp::utils

#endif  // GYAKUENKI_CPP__UTILS__GYAKUENKI_CPP_UTILS_HPP_
