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
#include <string>
#include <vector>

namespace gyakuenki_cpp::utils
{

class CameraInfo
{
public:
  CameraInfo();
  void load_configuration(const std::string & config_path);
  cv::Point2d normalize_pixel(cv::Point2d pixel);
  const std::string & get_frame_id() { return frame_id; }

  int image_width() { return width; }
  int image_height() { return height; }

  double fx() { return K.at<double>(0, 0); }
  double fy() { return K.at<double>(1, 1); }
  double cx() { return K.at<double>(0, 2); }
  double cy() { return K.at<double>(1, 2); }

  double k1() { return D.at<double>(0); }
  double k2() { return D.at<double>(1); }
  double p1() { return D.at<double>(2); }
  double p2() { return D.at<double>(3); }
  double k3() { return D.at<double>(4); }
  double k4() { return D.at<double>(5); }
  double k5() { return D.at<double>(6); }
  double k6() { return D.at<double>(7); }

private:
  std::string frame_id;
  cv::Mat K;  // intrinsic camera matrix
  cv::Mat D;  // distortion coefficients

  bool use_distortion;
  int width;
  int height;
};

}  // namespace gyakuenki_cpp::utils

#endif  // GYAKUENKI_CPP__UTILS__CAMERA_INFO_HPP_
