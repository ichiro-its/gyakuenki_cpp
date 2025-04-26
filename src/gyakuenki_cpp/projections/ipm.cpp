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

#include "jitsuyo/config.hpp"

namespace gyakuenki_cpp
{

IPM::IPM(
  const std::shared_ptr<rclcpp::Node> & node, const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
  const std::shared_ptr<tf2_ros::TransformListener> & tf_listener, const std::string & path)
: node(node), tf_buffer(tf_buffer), tf_listener(tf_listener), config_path(path)
{
  // Load camera info
  camera_info.load_configuration(path);
  load_config(path);
}

void IPM::load_config(const std::string & path)
{
  nlohmann::json config;
  if (!jitsuyo::load_config(path, "camera_offset.json", config)) {
    throw std::runtime_error("Failed to load configuration file `camera_offset.json`");
  }

  bool valid_config = true;
  double roll_double;
  double pitch_double;
  double yaw_double;

  nlohmann::json rotation_offset_section;
  if (jitsuyo::assign_val(config, "rotation_offset", rotation_offset_section)) {
    bool valid_section = true;
    valid_section &= jitsuyo::assign_val(rotation_offset_section, "roll", roll_double);
    valid_section &= jitsuyo::assign_val(rotation_offset_section, "pitch", pitch_double);
    valid_section &= jitsuyo::assign_val(rotation_offset_section, "yaw", yaw_double);
    if (!valid_section) {
      std::cout << "Error found at section `rotation_offset`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  camera_offset.roll = keisan::make_degree(roll_double);
  camera_offset.pitch = keisan::make_degree(pitch_double);
  camera_offset.yaw = keisan::make_degree(yaw_double);

  rotation_offset.setRPY(
    camera_offset.roll.radian(), camera_offset.pitch.radian(), camera_offset.yaw.radian());

  nlohmann::json position_offset_section;
  if (jitsuyo::assign_val(config, "position_offset", position_offset_section)) {
    bool valid_section = true;
    valid_section &= jitsuyo::assign_val(position_offset_section, "x", camera_offset.position.x);
    valid_section &= jitsuyo::assign_val(position_offset_section, "y", camera_offset.position.y);
    valid_section &= jitsuyo::assign_val(position_offset_section, "z", camera_offset.position.z);
    if (!valid_section) {
      std::cout << "Error found at section `position_offset`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  if (!valid_config) {
    throw std::runtime_error("Failed to set configuration file `camera_offset.json`");
  }
}

void IPM::set_config(double x, double y, double z, double roll, double pitch, double yaw)
{
  camera_offset.position.x = x;
  camera_offset.position.y = y;
  camera_offset.position.z = z;
  camera_offset.roll = keisan::make_degree(roll);
  camera_offset.pitch = keisan::make_degree(pitch);
  camera_offset.yaw = keisan::make_degree(yaw);
  rotation_offset.setRPY(
    camera_offset.roll.radian(), camera_offset.pitch.radian(), camera_offset.yaw.radian());
}

void IPM::save_config()
{
  nlohmann::json config;

  config["rotation_offset"]["roll"] = camera_offset.roll.degree();
  config["rotation_offset"]["pitch"] = camera_offset.pitch.degree();
  config["rotation_offset"]["yaw"] = camera_offset.yaw.degree();

  config["position_offset"]["x"] = camera_offset.position.x;
  config["position_offset"]["y"] = camera_offset.position.y;
  config["position_offset"]["z"] = camera_offset.position.z;

  jitsuyo::save_config(config_path, "camera_offset.json", config);
}

// Check if the bottom bounding box is at the bottom of the image
bool IPM::object_at_bottom_of_image(const DetectedObject & detected_object, int detection_type)
{
  // TODO: Handle for color detection
  return detected_object.bottom >
         camera_info.image_height - 5;  // TODO: Change 5 to a threshold variable
}

void IPM::normalize_pixel(cv::Point2d & pixel)
{
  // x = (u - cx) / fx
  // y = (v - cy) / fy
  pixel.x = (pixel.x - camera_info.cx) / camera_info.fx;
  pixel.y = (pixel.y - camera_info.cy) / camera_info.fy;
}

void IPM::undistort_pixel(cv::Point2d & pixel)
{
  double k1 = camera_info.D[0];
  double k2 = camera_info.D[1];
  double p1 = camera_info.D[2];
  double p2 = camera_info.D[3];
  double k3 = camera_info.D[4];
  double k4 = camera_info.D[5];
  double k5 = camera_info.D[6];
  double k6 = camera_info.D[7];

  // Undistort the pixel coordinates
  double x = pixel.x;
  double y = pixel.y;

  double r2 = x * x + y * y;
  double r4 = r2 * r2;
  double r6 = r4 * r2;
  double r8 = r4 * r4;
  double r10 = r6 * r4;
  double r12 = r6 * r6;

  // Apply radial distortion
  double radial_distortion = 1 + k1 * r2 + k2 * r4 + k3 * r6 + k4 * r8 + k5 * r10 + k6 * r12;

  // Apply tangential distortion
  pixel.x = x * radial_distortion + 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
  pixel.y = y * radial_distortion + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
}

// Get the target pixel (i. e. u and v) that are going to be projected depending on the object and detection type
cv::Point2d IPM::get_target_pixel(const DetectedObject & detected_object, int detection_type)
{
  cv::Point2d point;

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
        point.x = detected_object.left + detected_object.right / 2;
        point.y = detected_object.top + detected_object.bottom / 2;
      }

      break;
    default:
      throw std::runtime_error("Invalid detection type");
  }

  return point;
}

// Get the object's normalized XY coordinates in image plane
cv::Point2d IPM::get_normalized_target_pixel(
  const DetectedObject & detected_object, int detection_type)
{
  // Get the target pixel that are going to be projected (i. e. u and v)
  cv::Point2d pixel = get_target_pixel(detected_object, detection_type);

  normalize_pixel(pixel);

  // Un-distort the pixel coordinates if distortion is used
  if (camera_info.use_distortion) {
    undistort_pixel(pixel);
  }

  // Return the normalized pixel coordinates
  return pixel;
}

// Convert tf2::Quaternion to msg::Quaternion
IPM::Quaternion IPM::tf2_to_msg(const tf2::Quaternion & tf2_quat)
{
  Quaternion msg_quat;
  msg_quat.x = tf2_quat.x();
  msg_quat.y = tf2_quat.y();
  msg_quat.z = tf2_quat.z();
  msg_quat.w = tf2_quat.w();

  return msg_quat;
}

// Convert msg::Quaternion to tf2::Quaternion
tf2::Quaternion IPM::msg_to_tf2(const Quaternion & msg_quat)
{
  return tf2::Quaternion(msg_quat.x, msg_quat.y, msg_quat.z, msg_quat.w);
}

// Convert quaternion to rotation matrix
keisan::Matrix<4, 4> IPM::quat_to_rotation_matrix(const Quaternion & q)
{
  // Normalize the quaternion
  double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  keisan::Quaternion<double> quat(q.x / norm, q.y / norm, q.z / norm, q.w / norm);

  return keisan::rotation_matrix(quat);
}

// Find Pc (3D point in camera frame) using normalized pixel
// We will use general plane equation in camera frame: nc . Pc + d = 0
// nc = plane normal in camera frame, Pc = 3D point in camera frame, d = distance from origin to plane
// Since the object lies on the ground plane, the normal vector of base frame is [0, 0, 1]
// Therefore, nc = R . [0, 0, 1] and d is the height offset between camera frame and object height
keisan::Matrix<4, 1> IPM::point_in_camera_frame(
  const cv::Point2d & pixel, const keisan::Matrix<4, 4> & T, const keisan::Matrix<4, 4> & R,
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
  double Zc = (object_height - T[2][3]) / denominator;

  if (Zc < 0) {
    throw std::runtime_error("Object is behind the camera frame!");
  }

  // Calculate the X and Y coordinates in camera frame
  double Xc = Zc * pixel.x;
  double Yc = Zc * pixel.y;

  // Create the 3D point in camera frame
  keisan::Matrix<4, 1> Pc(Xc, Yc, Zc, 1.0);

  return Pc;
}

// Map the detected object to the 3D world relative to param output_frame (e. g. base_footprint) using pinhole camera model
gyakuenki_interfaces::msg::ProjectedObject IPM::map_object(
  const DetectedObject & detected_object, int detection_type, const std::string & output_frame,
  keisan::Matrix<4, 1> & Pc)
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
  cv::Point2d norm_pixel = get_normalized_target_pixel(detected_object, detection_type);

  // Get the latest transform (Rotation and Translation) from the camera to the output frame
  geometry_msgs::msg::TransformStamped t;
  try {
    t = tf_buffer->lookupTransform(output_frame, camera_info.frame_id, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    throw std::runtime_error(ex.what());
  }

  // Convert the quaternion to rotation matrix R
  keisan::Matrix<4, 4> R =
    quat_to_rotation_matrix(tf2_to_msg(msg_to_tf2(t.transform.rotation) * rotation_offset));

  // Get the translation matrix
  keisan::Matrix<4, 4> T = keisan::translation_matrix(keisan::Point3(
    t.transform.translation.x + camera_offset.position.x,
    t.transform.translation.y + camera_offset.position.y,
    t.transform.translation.z + camera_offset.position.z));

  // Now, we have the 3D point in camera frame
  Pc = point_in_camera_frame(norm_pixel, T, R, detected_object.label);

  // But we want the 3D points relative to the output frame
  // Therefore, transform using spatial transformation
  keisan::Matrix<4, 4> M = R;
  M[0][3] = T[0][3];
  M[1][3] = T[1][3];
  M[2][3] = T[2][3];

  keisan::Matrix<4, 1> Pw = M * Pc;

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
