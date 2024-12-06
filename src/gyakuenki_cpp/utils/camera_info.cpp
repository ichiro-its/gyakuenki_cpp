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

namespace gyakuenki_cpp
{

CameraInfo::CameraInfo()
: frame_id("camera"),
  K(keisan::Matrix<3, 3>::zero()),
  D(keisan::Matrix<1, 5>::zero()),
  image_width(0),
  image_height(0),
  use_distortion(false)
{
}

CameraInfo::load_configuration(const std::string & path)
{
  nlohmann::json config;

  if (!jitsuyo::load_config(config_path, "camera_info.json", config)) {
    throw std::runtime_error("Failed to load config file `" + config_path + "camera_info.json`");
  }

  bool valid_config = true;

  nlohmann::json camera_matrix_section;
  if (jitsuyo::assign_val(config, "camera_matrix", camera_matrix_section)) {
    bool valid_section = jitsuyo::assign_val(camera_matrix_section, "fx", K(0, 0));
    valid_section &= jitsuyo::assign_val(camera_matrix_section, "fy", K(1, 1));
    valid_section &= jitsuyo::assign_val(camera_matrix_section, "cx", K(0, 2));
    valid_section &= jitsuyo::assign_val(camera_matrix_section, "cy", K(1, 2));

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
      valid_section &= jitsuyo::assign_val(distortion_section, "k1", D(0, 0));
      valid_section &= jitsuyo::assign_val(distortion_section, "k2", D(0, 1));
      valid_section &= jitsuyo::assign_val(distortion_section, "p1", D(0, 2));
      valid_section &= jitsuyo::assign_val(distortion_section, "p2", D(0, 3));
      valid_section &= jitsuyo::assign_val(distortion_section, "k3", D(0, 4));
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

}  // namespace gyakuenki_cpp