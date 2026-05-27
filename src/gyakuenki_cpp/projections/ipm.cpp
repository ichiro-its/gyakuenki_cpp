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
  camera_info = utils::CameraInfo(path);
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
  double x_double;
  double y_double;
  double z_double;

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

  nlohmann::json position_offset_section;
  if (jitsuyo::assign_val(config, "position_offset", position_offset_section)) {
    bool valid_section = true;
    valid_section &= jitsuyo::assign_val(position_offset_section, "x", x_double);
    valid_section &= jitsuyo::assign_val(position_offset_section, "y", y_double);
    valid_section &= jitsuyo::assign_val(position_offset_section, "z", z_double);
    if (!valid_section) {
      std::cout << "Error found at section `position_offset`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  set_config(x_double, y_double, z_double, roll_double, pitch_double, yaw_double);

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

  translation_offset.setValue(x, y, z);
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
bool IPM::object_at_bottom_of_image(const DetectedObject & detected_object)
{
  // TODO: Handle for color detection
  return detected_object.top + detected_object.bottom > camera_info.image_height() - 5;
}

// Get the target pixel that are going to be projected depending on the object
cv::Point2d IPM::get_target_pixel(const DetectedObject & detected_object)
{
  cv::Point2d point;

  // Goalpost and robot uses bottom-center of bounding box, other object uses center of bounding box
  point.x = detected_object.left + detected_object.right / 2;
  if (detected_object.label == "goalpost" || detected_object.label == "robot") {
    point.y = detected_object.top + detected_object.bottom;
  } else {
    point.y = detected_object.top + (detected_object.bottom / 2.0);
  }

  return point;
}

// Get the object's normalized XY coordinates in image plane
cv::Point2d IPM::get_normalized_target_pixel(const DetectedObject & detected_object)
{
  cv::Point2d pixel = get_target_pixel(detected_object);
  return camera_info.normalize_pixel(pixel);
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
keisan::Matrix<4, 1> IPM::point_in_camera_frame(
  const cv::Point2d & pixel, const keisan::Matrix<4, 4> & T, const keisan::Matrix<4, 4> & R,
  const std::string & object_label)
{
  // Get object height
  double object_height =
    object_label == "ball" ? 0.135 / 2 : 0.0;  // For ball, the height is the radius of the ball

  // Calculate depth (Z)
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

  keisan::Matrix<4, 1> Pc(Xc, Yc, Zc, 1.0);

  return Pc;
}

// Apply the camera translation and rotation offset to the transform from camera frame
// to output frame (e. g. base_footprint) and return the corrected transform
tf2::Transform IPM::get_corrected_camera_transform(
  const std::string & output_frame, const rclcpp::Time & timestamp)
{
  // Get the transform from the camera frame to the output frame at the timestamp when
  // the image was captured
  geometry_msgs::msg::TransformStamped t;
  try {
    if (timestamp.nanoseconds() == 0) {
      t = tf_buffer->lookupTransform(output_frame, camera_info.get_frame_id(), tf2::TimePointZero);
    } else {
      t = tf_buffer->lookupTransform(
        output_frame, camera_info.get_frame_id(), timestamp, tf2::Duration::zero());
    }
  } catch (tf2::TransformException &) {
    try {
      t = tf_buffer->lookupTransform(output_frame, camera_info.get_frame_id(), tf2::TimePointZero);
      RCLCPP_WARN(node->get_logger(), "TF not available for capture timestamp, using latest TF");
    } catch (tf2::TransformException & ex) {
      throw std::runtime_error(ex.what());
    }
  }

  // Build TF from base to camera
  tf2::Transform tf_base_to_cam;

  tf2::Quaternion q_base_cam = msg_to_tf2(t.transform.rotation);

  tf_base_to_cam.setOrigin(
    tf2::Vector3(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z));

  tf_base_to_cam.setRotation(q_base_cam);

  // Build offset transform
  tf2::Transform tf_offset;
  tf_offset.setOrigin(translation_offset);
  tf_offset.setRotation(rotation_offset);

  // Apply offset: T_final = T_base_cam * T_offset
  tf2::Transform tf_final = tf_base_to_cam * tf_offset;

  return tf_final;
}

// Map the detected object to the 3D world relative to param output_frame (e. g. base_footprint) using pinhole camera model
gyakuenki_interfaces::msg::Point3 IPM::map_object(
  const DetectedObject & detected_object, const rclcpp::Time & timestamp,
  const std::string & output_frame, keisan::Matrix<4, 1> & Pc)
{
  // Ignore object if the bounding box touches the bottom of the image
  if (object_at_bottom_of_image(detected_object)) {
    throw std::runtime_error("Bounding box touches the bottom of the image, can not map object!");
  }

  cv::Point2d norm_pixel = get_normalized_target_pixel(detected_object);

  // Get the camera transform with offset applied expressed in output_frame
  tf2::Transform tf_final = get_corrected_camera_transform(output_frame, timestamp);

  tf2::Quaternion q_final = tf_final.getRotation();
  tf2::Vector3 t_final = tf_final.getOrigin();

  // Convert the quaternion to rotation matrix R
  keisan::Matrix<4, 4> R = quat_to_rotation_matrix(tf2_to_msg(q_final));

  // Get the translation matrix
  keisan::Matrix<4, 4> T =
    keisan::translation_matrix(keisan::Point3(t_final.x(), t_final.y(), t_final.z()));

  // 3D point in camera frame
  Pc = point_in_camera_frame(norm_pixel, T, R, detected_object.label);

  // Transform to output_frame
  keisan::Matrix<4, 4> M = R;
  M[0][3] = T[0][3];
  M[1][3] = T[1][3];
  M[2][3] = T[2][3];

  keisan::Matrix<4, 1> Pw = M * Pc;

  // Create the ProjectedObject instance
  gyakuenki_interfaces::msg::Point3 position;
  position.x = Pw[0][0];
  position.y = Pw[1][0];
  position.z = Pw[2][0];

  return position;
}

}  // namespace gyakuenki_cpp
