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

#ifndef GYAKUENKI_CPP__UTILS__CAMERA_INFO_HPP_
#define GYAKUENKI_CPP__UTILS__CAMERA_INFO_HPP_

#include <opencv2/opencv.hpp>

#include "keisan/matrix.hpp"

namespace gyakuenki_cpp
{

struct CameraInfo
{
  std::string frame_id;    // Camera frame id
  keisan::Matrix<3, 3> K;  // Intrinsic matrix
  keisan::Matrix<1, 5> D;  // Distortion coefficients
  int image_width;         // Image width
  int image_height;        // Image height
  bool use_distortion;     // Use distortion coefficients

  CameraInfo();
  load_configuration(const std::string & config_path);
};

}  // namespace gyakuenki_cpp

#endif  // GYAKUENKI_CPP__UTILS__CAMERA_INFO_HPP_
