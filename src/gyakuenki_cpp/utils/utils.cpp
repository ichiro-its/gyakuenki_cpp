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

#include "gyakuenki_cpp/utils/utils.hpp"

#include "jitsuyo/config.hpp"

namespace gyakuenki_cpp::utils
{

const CameraInfo & load_camera_info(const std::string & config_path)
{
  nlohmann::json config;

  if (!jitsuyo::load_config(config_path, "camera_info.json", config)) {
    throw std::runtime_error("Failed to load config file `" + config_path + "camera_info.json`");
  }

  CameraInfo camera_info;
  bool valid_config = true;

  nlohmann::json camera_matrix_section;
  if (jitsuyo::assign_val(config, "camera_matrix", camera_matrix_section)) {
    bool valid_section = jitsuyo::assign_val(camera_matrix_section, "fx", camera_info.K(0, 0));
    valid_section &= jitsuyo::assign_val(camera_matrix_section, "fy", camera_info.K(1, 1));
    valid_section &= jitsuyo::assign_val(camera_matrix_section, "cx", camera_info.K(0, 2));
    valid_section &= jitsuyo::assign_val(camera_matrix_section, "cy", camera_info.K(1, 2));

    if (!valid_section) {
      std::cout << "Error found at section `camera_matrix`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  nlohmann::json distortion_section;
  if (jitsuyo::assign_val(config, "distortion", distortion_section)) {
    bool valid_section = jitsuyo::assign_val(distortion_section, "k1", camera_info.D(0, 0));
    valid_section &= jitsuyo::assign_val(distortion_section, "k2", camera_info.D(1, 0));
    valid_section &= jitsuyo::assign_val(distortion_section, "p1", camera_info.D(2, 0));
    valid_section &= jitsuyo::assign_val(distortion_section, "p2", camera_info.D(3, 0));
    valid_section &= jitsuyo::assign_val(distortion_section, "k3", camera_info.D(4, 0));

    if (!valid_section) {
      std::cout << "Error found at section `distortion`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  return camera_info;
}

const keisan::rotation_matrix & quat_to_rotation_matrix(const Quaternion & q)
{
  keisan::Quaternion<double> quat(q.x, q.y, q.z, q.w);

  keisan::Euler<double> euler = quat.euler();

  keisan::rotation_matrix R(euler);

  return R;
}

const cv::Point & get_projection_pixels(const DetectedObject & detected_object, int object_type)
{
  cv::Point point;

  switch (object_type) {
    case TYPE_DNN:
      // x is the center of the detection
      point.x = detected_object.left + detected_object.right / 2;

      // For goalpost and robot, y is the bottom of the detection
      // Ball and field marks, y is the center of the detection
      if (detected_object.label == "goalpost" || detected_object.label == "robot") {
        point.y = detected_object.bottom;
      } else {
        point.y = detected_object.top + detected_object.bottom / 2;
      }

      break;
    case TYPE_COLOR:
      if (detected_object.label == "ball") {
        // TODO: This should be received from soccer node
      }

      break;
    default:
      throw std::runtime_error("Invalid object type");
  }

  return point;
}

double get_height_offset(const std::string & label, int object_type)
{
  if (label == "ball") {
    return 0.153  // Diameter of the ball
  }

  return 0.0;
}  // namespace gyakuenki_cpp::utils
