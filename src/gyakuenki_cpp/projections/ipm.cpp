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

#include "gyakuenki_cpp/projections/ipm.hpp"

namespace gyakuenki_cpp
{

IPM::IPM(
  const std::shared_ptr<rclcpp::Node> & node, const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
  const std::shared_ptr<tf2_ros::TransformListener> & tf_listener, const std::string & path)
: node(node), tf_buffer(tf_buffer), tf_listener(tf_listener)
{
  // Load camera info
  // camera_info.K = [ fx 0 cx]
  //                 [ 0 fy cy]
  //                 [ 0  0  1]
  // camera_info.D = [k1 k2 p1 p2 k3]
  // camera_info.image_width = width
  // camera_info.image_height = height
  camera_info = utils::load_camera_info(path);
}

// Get the object's normalized pixel coordinates that are going to be projected
const cv::Point & IPM::get_projection_point(const DetectedObject & detected_object, int object_type)
{
  // Get the object's pixel coordinates that are going to be projected
  cv::Point point = utils::get_projection_pixels(detected_object, object_type);

  // Un-distort the pixel coordinates if distortion is used
  if (camera_info.use_distortion) {
    point = utils::undistort(point, camera_info, camera_info.use_distortion);
  }

  // Normalize the undistorted pixels
  // u = (u - cx) / fx
  // v = (v - cy) / fy
  point.x = (point.x - camera_info.K(0, 2)) / camera_info.K(0, 0);
  point.y = (point.y - camera_info.K(1, 2)) / camera_info.K(1, 1);

  return point;
}

// Map the detected object to the 3D world relative to param output_frame using pinhole camera model
const ProjectedObject & IPM::map_object(
  const DetectedObject & detected_object, int object_type, const std::string & output_frame)
{
  // The relationship between 3D world points Pw = [Xw, Yw, Zw, 1] and 2D image points p = [u, v, 1] is given by:
  // p = K * [R | T] * Pw
  // where K is the camera intrinsic matrix, R is the rotation matrix, T is the translation vector
  // The idea is to reverse this process to get the 3D world points from the 2D image points

  // Get u and v coordinates
  cv::Point point = get_projection_point(detected_object, object_type);

  // Get the latest transform (Rotation and Translation) from the camera to the output frame which
  // we will use later to transform the 3D points from camera frame to output frame
  // According to ROS2 documentation, rclcpp::Node::get_clock()->now() can be used here but it will be converted to tf2::TimePoint anyways
  geometry_msgs::msg::TransformedStamped t;
  t = tf_buffer->lookupTransform(output_frame, camera_info.frame_id, tf2::TimePointZero);

  // Convert the quaternion to a rotation matrix R
  keisan::rotation_matrix R = utils::quat_to_rotation_matrix(t.transform.rotation);

  // Get the translation matrix
  keisan::Matrix<3, 1> T;
  T << t.transform.translation.x, t.transform.translation.y, t.transform.translation.z;

  // Get the height offset of the detected object
  double height_offset = utils::get_height_offset(detected_object.label);

  // Calculate the depth (Z) of the object in the camera frame.
  // Since the object always lies on ground plane, the Z can be calculated by comparing
  // the height offset and the height difference between camera frame and output frame (i. e. T(2))
  double denominator = R(2, 0) * point.x + R(2, 1) * point.y + R(2, 2);
  if (std::abs(denominator) < 1e-6) {
    throw std::runtime_error("Denominator is too close to zero, cannot compute Z.");
  }
  Z = height_offset / denominator;

  // Using the pinhole camera model, we can calculate the 3D points relative to camera frame
  keisan::Matrix<3, 1> Pw;
  Pw << point.x * Z, point.y * Z, Z;

  // But we want the 3D points relative to the output frame
  // Therefore, transform using spatial transformation
  Pw = R * Pw + T;

  // Create the ProjectedObject instance
  ProjectedObject projected_object;

  projected_object.label = detected_object.label;
  projected_object.x = Pw(0);
  projected_object.y = Pw(1);
  projected_object.z = Pw(2);

  return projected_object;
}

}  // namespace gyakuenki_cpp
