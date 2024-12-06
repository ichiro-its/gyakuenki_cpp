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

#include "gyakuenki_cpp/utils/camera_info.hpp"

#include "jitsuyo/config.hpp"

namespace gyakuenki_cpp::utils
{

CameraInfo::CameraInfo()
: frame_id("camera"),
  fx(0.0),
  fy(0.0),
  cx(0.0),
  cy(0.0),
  D(5),
  image_width(0),
  image_height(0),
  use_distortion(false)
{
}

void CameraInfo::load_configuration(const std::string & config_path)
{
  nlohmann::json config;

  if (!jitsuyo::load_config(config_path, "camera_info.json", config)) {
    throw std::runtime_error("Failed to load config file `" + config_path + "camera_info.json`");
  }

  bool valid_config = true;

  nlohmann::json camera_matrix_section;
  if (jitsuyo::assign_val(config, "camera_matrix", camera_matrix_section)) {
    bool valid_section = jitsuyo::assign_val(camera_matrix_section, "fx", fx);
    valid_section &= jitsuyo::assign_val(camera_matrix_section, "fy", fy);
    valid_section &= jitsuyo::assign_val(camera_matrix_section, "cx", cx);
    valid_section &= jitsuyo::assign_val(camera_matrix_section, "cy", cy);

    if (!valid_section) {
      std::cout << "Error found at section `camera_matrix`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  nlohmann::json distortion_section;
  if (jitsuyo::assign_val(config, "distortion", distortion_section)) {
    bool valid_section = jitsuyo::assign_val(distortion_section, "use_distortion", use_distortion);

    if (use_distortion) {
      double k1;
      double k2;
      double p1;
      double p2;
      double k3;

      valid_section &= jitsuyo::assign_val(distortion_section, "k1", k1);
      valid_section &= jitsuyo::assign_val(distortion_section, "k2", k2);
      valid_section &= jitsuyo::assign_val(distortion_section, "p1", p1);
      valid_section &= jitsuyo::assign_val(distortion_section, "p2", p2);
      valid_section &= jitsuyo::assign_val(distortion_section, "k3", k3);

      D.at(0) = k1;
      D.at(1) = k2;
      D.at(2) = p1;
      D.at(3) = p2;
      D.at(4) = k3;
    }

    if (!valid_section) {
      std::cout << "Error found at section `distortion`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  nlohmann::json image_section;
  if (jitsuyo::assign_val(config, "image", image_section)) {
    bool valid_section = jitsuyo::assign_val(image_section, "width", image_width);
    valid_section &= jitsuyo::assign_val(image_section, "height", image_height);

    if (!valid_section) {
      std::cout << "Error found at section `image`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  if (!valid_config) {
    throw std::runtime_error("Invalid camera configuration");
  }
}

}  // namespace gyakuenki_cpp::utils
