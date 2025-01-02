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
  camera_info.load_configuration(path);
}

// Check if the bottom bounding box is at the bottom of the image
bool IPM::object_at_bottom_of_image(const DetectedObject & detected_object, int detection_type)
{
  // TODO: Handle for color detection
  return detected_object.bottom >
         camera_info.image_height - 5;  // TODO: Change 5 to a threshold variable
}

void IPM::normalize_pixel(cv::Point & pixel)
{
  // x = (u - cx) / fx
  // y = (v - cy) / fy
  pixel.x = (pixel.x - camera_info.cx) / camera_info.fx;
  pixel.y = (pixel.y - camera_info.cy) / camera_info.fy;
}

void IPM::denormalize_pixel(cv::Point & pixel)
{
  // u = x * fx + cx
  // v = y * fy + cy
  pixel.x = pixel.x * camera_info.fx + camera_info.cx;
  pixel.y = pixel.y * camera_info.fy + camera_info.cy;
}

void IPM::undistort_pixel(cv::Point & pixel)
{
  double k1 = camera_info.D[0];
  double k2 = camera_info.D[1];
  double p1 = camera_info.D[2];
  double p2 = camera_info.D[3];
  double k3 = camera_info.D[4];

  // Normalize the pixel coordinates
  normalize_pixel(pixel);

  // Undistort the pixel coordinates
  double x = pixel.x;
  double y = pixel.y;

  double r2 = x * x + y * y;
  double r4 = r2 * r2;
  double r6 = r4 * r2;

  double x_corr = x * (1 + k1 * r2 + k2 * r4 + k3 * r6) + 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
  double y_corr = y * (1 + k1 * r2 + k2 * r4 + k3 * r6) + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;

  // Denormalize the pixel coordinates
  denormalize_pixel(pixel);
}

// Get the target pixel (i. e. u and v) that are going to be projected depending on the object and detection type
cv::Point IPM::get_target_pixel(const DetectedObject & detected_object, int detection_type)
{
  cv::Point point;

  switch (detection_type) {
    case TYPE_DNN:
      // u is the center of the detection
      point.x = detected_object.left + detected_object.right / 2;

      // For goalpost and robot, v is the bottom of the detection
      // Ball and field marks, v is the center of the detection
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
      throw std::runtime_error("Invalid detection type");
  }

  return point;
}

// Get the object's normalized XY coordinates in image plane
cv::Point IPM::get_normalized_target_pixel(
  const DetectedObject & detected_object, int detection_type)
{
  // Get the target pixel that are going to be projected (i. e. u and v)
  cv::Point pixel = get_target_pixel(detected_object, detection_type);

  // Un-distort the pixel coordinates if distortion is used
  if (camera_info.use_distortion) {
    undistort_pixel(pixel);
  }

  normalize_pixel(pixel);

  // Return the normalized pixel coordinates
  return pixel;
}

// Convert quaternion to rotation matrix
keisan::Matrix<4, 4> IPM::quat_to_rotation_matrix(const Quaternion & q)
{
  keisan::Quaternion<double> quat(q.x, q.y, q.z, q.w);

  keisan::Euler<double> euler = quat.euler();

  return keisan::rotation_matrix(euler);
}

// Find Pc (3D point in camera frame) using normalized pixel
// We will use general plane equation in camera frame: nc . Pc + d = 0
// nc = plane normal in camera frame, Pc = 3D point in camera frame, d = distance from origin to plane
// Since the object lies on the ground plane, the normal vector of base frame is [0, 0, 1]
// Therefore, nc = R . [0, 0, 1] and d is the height offset between camera frame and object height
keisan::Matrix<4, 1> IPM::point_in_camera_frame(
  const cv::Point & pixel, const keisan::Matrix<4, 4> & T, const keisan::Matrix<4, 4> & R,
  const std::string & object_label)
{
  // Get object height
  double object_height =
    object_label == "ball" ? 0.153 / 2 : 0.0;  // For ball, the height is the radius of the ball

  // Calculate the Z coordinate in camera frame
  double denominator = R[2][0] * pixel.x + R[2][1] * pixel.y + R[2][2];
  if (std::abs(denominator) < 1e-6) {
    throw std::runtime_error("No intersection with base plane!");
  }
  double Zc = object_height - T[2][3] / denominator;

  // Calculate the X and Y coordinates in camera frame
  double Xc = Zc * pixel.x;
  double Yc = Zc * pixel.y;

  // Create the 3D point in camera frame
  keisan::Matrix<4, 1> Pc(Xc, Yc, Zc, 1.0);

  return Pc;
}

// Map the detected object to the 3D world relative to param output_frame (e. g. base_footprint) using pinhole camera model
gyakuenki_interfaces::msg::ProjectedObject IPM::map_object(
  const DetectedObject & detected_object, int detection_type, const std::string & output_frame)
{
  // The relationship between 3D world points Pw = [Xw, Yw, Zw, 1] and 2D image pixels p = [u, v, 1] is given by:
  // p = K * [R | T] * Pw
  // where K is the camera intrinsic matrix, R is the rotation matrix and T is the translation matrix of the camera frame
  //
  // The idea is to reverse this process to get the 3D world points from the 2D image points

  // We can not map the object if the bounding box touches the bottom of the image
  if (object_at_bottom_of_image(detected_object, detection_type)) {
    throw std::runtime_error("Bounding box touches the bottom of the image, can not map object!");
  }

  // First, get the normalized target pixel
  cv::Point norm_pixel = get_normalized_target_pixel(detected_object, detection_type);

  // Get the latest transform (Rotation and Translation) from the camera to the output frame which
  geometry_msgs::msg::TransformStamped t;
  try {
    t = tf_buffer->lookupTransform(output_frame, camera_info.frame_id, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    throw std::runtime_error(ex.what());
  }

  // Convert the quaternion to rotation matrix R
  keisan::Matrix<4, 4> R = quat_to_rotation_matrix(t.transform.rotation);

  // Get the translation matrix
  keisan::Matrix<4, 4> T = keisan::translation_matrix(keisan::Point3(
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z));

  // Now, we have the 3D point in camera frame
  keisan::Matrix<4, 1> Pc = point_in_camera_frame(norm_pixel, T, R, detected_object.label);

  // But we want the 3D points relative to the output frame
  // Therefore, transform using spatial transformation
  keisan::Matrix<4, 1> Pw = (R * T) * Pc;

  // Create the ProjectedObject instance
  gyakuenki_interfaces::msg::ProjectedObject projected_object;

  projected_object.label = detected_object.label;
  projected_object.position.x = Pw[0][0];
  projected_object.position.y = Pw[1][0];
  projected_object.position.z = Pw[2][0];
  projected_object.confidence = detected_object.score;

  return projected_object;
}

}  // namespace gyakuenki_cpp
