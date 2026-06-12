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
  double roll_center_double;
  double pitch_center_double;
  double yaw_center_double;
  double roll_side_double;
  double pitch_side_double;
  double yaw_side_double;
  double x_double;
  double y_double;
  double z_double;

  nlohmann::json rotation_offset_center_section;
  if (jitsuyo::assign_val(config, "rotation_offset_center", rotation_offset_center_section)) {
    bool valid_section = true;
    valid_section &= jitsuyo::assign_val(rotation_offset_center_section, "roll", roll_center_double);
    valid_section &= jitsuyo::assign_val(rotation_offset_center_section, "pitch", pitch_center_double);
    valid_section &= jitsuyo::assign_val(rotation_offset_center_section, "yaw", yaw_center_double);
    if (!valid_section) {
      std::cout << "Error found at section `rotation_offset_center`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  nlohmann::json rotation_offset_side_section;
  if (jitsuyo::assign_val(config, "rotation_offset_side", rotation_offset_side_section)) {
    bool valid_section = true;
    valid_section &= jitsuyo::assign_val(rotation_offset_side_section, "roll", roll_side_double);
    valid_section &= jitsuyo::assign_val(rotation_offset_side_section, "pitch", pitch_side_double);
    valid_section &= jitsuyo::assign_val(rotation_offset_side_section, "yaw", yaw_side_double);
    if (!valid_section) {
      std::cout << "Error found at section `rotation_offset_side`" << std::endl;
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

  nlohmann::json confidence_section;
  if (jitsuyo::assign_val(config, "confidence", confidence_section)) {
    bool valid_section = jitsuyo::assign_val(confidence_section, "horizon_scale", horizon_scale);

    if (!valid_section) {
      std::cerr << "WARN: Error found at section `confidence`, using default values" << std::endl;
    }

  } else {
    std::cerr << "WARN: Error found at section `confidence`, using default values" << std::endl;
  }

  set_config(
    x_double, y_double, z_double,
    roll_center_double, pitch_center_double, yaw_center_double,
    roll_side_double, pitch_side_double, yaw_side_double);

  if (!valid_config) {
    throw std::runtime_error("Failed to set configuration file `camera_offset.json`");
  }

  horizon_scale = keisan::clamp(horizon_scale, 15.0, 35.0);
}

void IPM::set_config(
  double x, double y, double z,
  double roll_center, double pitch_center, double yaw_center,
  double roll_side, double pitch_side, double yaw_side)
{
  camera_offset.position.x = x;
  camera_offset.position.y = y;
  camera_offset.position.z = z;
  camera_offset.roll_center = keisan::make_degree(roll_center);
  camera_offset.pitch_center = keisan::make_degree(pitch_center);
  camera_offset.yaw_center = keisan::make_degree(yaw_center);
  camera_offset.roll_side = keisan::make_degree(roll_side);
  camera_offset.pitch_side = keisan::make_degree(pitch_side);
  camera_offset.yaw_side = keisan::make_degree(yaw_side);

  translation_offset.setValue(x, y, z);

  rotation_offset_center.setRPY(
    camera_offset.roll_center.radian(),
    camera_offset.pitch_center.radian(),
    camera_offset.yaw_center.radian());
  rotation_offset_center.normalize();

  rotation_offset_side.setRPY(
    camera_offset.roll_side.radian(),
    camera_offset.pitch_side.radian(),
    camera_offset.yaw_side.radian());
  rotation_offset_side.normalize();
}

void IPM::save_config()
{
  nlohmann::json config;

  config["rotation_offset_center"]["roll"] = camera_offset.roll_center.degree();
  config["rotation_offset_center"]["pitch"] = camera_offset.pitch_center.degree();
  config["rotation_offset_center"]["yaw"] = camera_offset.yaw_center.degree();

  config["rotation_offset_side"]["roll"] = camera_offset.roll_side.degree();
  config["rotation_offset_side"]["pitch"] = camera_offset.pitch_side.degree();
  config["rotation_offset_side"]["yaw"] = camera_offset.yaw_side.degree();

  config["position_offset"]["x"] = camera_offset.position.x;
  config["position_offset"]["y"] = camera_offset.position.y;
  config["position_offset"]["z"] = camera_offset.position.z;

  jitsuyo::save_config(config_path, "camera_offset.json", config);
}

// Check if the bottom bounding box is at the bottom of the image
bool IPM::object_at_bottom_of_image(const DetectedObject & detected_object)
{
  // TODO: Handle for color detection
  return detected_object.top + detected_object.bottom > camera_info.image_height() - 2;
}

// Get the target pixel that are going to be projected depending on the object
cv::Point2d IPM::get_target_pixel(const DetectedObject & detected_object)
{
  cv::Point2d point;

  // Goalpost and robot uses bottom-center of bounding box, other object uses center of bounding box
  point.x = detected_object.left + detected_object.right / 2;
  if (detected_object.label == "robot") {
    point.y = detected_object.top + detected_object.bottom;
  } else if (detected_object.label == "goalpost") {
    point.y = detected_object.top + detected_object.bottom + 2.0;
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
keisan::Matrix<4, 4> IPM::quat_to_rotation_matrix(const tf2::Quaternion & q)
{
  // Normalize the quaternion
  double norm = std::sqrt(q.x() * q.x() + q.y() * q.y() + q.z() * q.z() + q.w() * q.w());
  keisan::Quaternion<double> quat(q.x() / norm, q.y() / norm, q.z() / norm, q.w() / norm);

  return keisan::rotation_matrix(quat);
}

double IPM::compute_confidence(const keisan::Matrix<4, 4> & R, const double D)
{
  double A = R[2][0] / this->camera_info.fx();
  double B = R[2][1] / this->camera_info.fy();
  double distance = std::fabs(D) / std::hypot(A, B);

  return 1.0 - std::exp(-distance / horizon_scale);
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
  if (denominator >= 0) {
    throw std::runtime_error("No intersection with base plane!");
  }

  double confidence = compute_confidence(R, denominator);
  if (confidence < 0.5) {
    throw std::runtime_error("Confidence is too low");
  }

  double Zc = (object_height - T[2][3]) / denominator;

  // Calculate the X and Y coordinates in camera frame
  double Xc = Zc * pixel.x;
  double Yc = Zc * pixel.y;

  keisan::Matrix<4, 1> Pc(Xc, Yc, Zc, 1.0);

  return Pc;
}

// Extract head pan angle (yaw) from TF at the given timestamp
// by looking up transform from base to head_pan_link
double IPM::get_head_pan_from_tf(const rclcpp::Time & timestamp)
{
  geometry_msgs::msg::TransformStamped t_head;
  try {
    if (timestamp.nanoseconds() == 0) {
      t_head = tf_buffer->lookupTransform(
        "base_footprint", "neck", tf2::TimePointZero);
    } else {
      t_head = tf_buffer->lookupTransform(
        "base_footprint", "neck", timestamp, tf2::Duration::zero());
    }
  } catch (tf2::TransformException &) {
    try {
      t_head = tf_buffer->lookupTransform(
        "base_footprint", "neck", tf2::TimePointZero);
      RCLCPP_WARN(
        node->get_logger(),
        "Head pan TF not available for capture timestamp, using latest TF");
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(
        node->get_logger(),
        "Failed to get head pan from TF: %s. Defaulting to 0.0", ex.what());
      return 0.0;
    }
  }

  // Extract yaw (pan) from the rotation quaternion
  tf2::Quaternion q_head = msg_to_tf2(t_head.transform.rotation);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q_head).getRPY(roll, pitch, yaw);

  return yaw;
}

tf2::Quaternion IPM::interpolate_rotation_offset(double pan_rad)
{
  double pan_abs = std::abs(pan_rad);

  double x = std::clamp(pan_abs / (M_PI * 0.5), 0.0, 1.0);
  double t = std::pow(x, 3.0);

  return rotation_offset_center.slerp(rotation_offset_side, t);
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

  // Get head pan angle from TF synchronized with the same timestamp
  double pan_rad = get_head_pan_from_tf(timestamp);

  // Interpolate rotation offset using TF-derived pan angle
  tf2::Quaternion q_offset = interpolate_rotation_offset(pan_rad);

  // Build offset transform
  tf2::Transform tf_offset;
  tf_offset.setOrigin(translation_offset);
  tf_offset.setRotation(q_offset);

  // Apply offset: T_final = T_base_cam * T_offset
  tf2::Transform tf_final = tf_base_to_cam * tf_offset;

  return tf_final;
}

// Map the detected object to the 3D world relative to param output_frame (e. g. base_footprint) using pinhole camera model
gyakuenki_interfaces::msg::Point3 IPM::map_object(
  const DetectedObject & detected_object, const keisan::Matrix<4, 4> & R,
  const keisan::Matrix<4, 4> t)
{
  // Ignore object if the bounding box touches the bottom of the image
  if (object_at_bottom_of_image(detected_object)) {
    throw std::runtime_error("Bounding box touches the bottom of the image, can not map object!");
  }

  cv::Point2d norm_pixel = get_normalized_target_pixel(detected_object);

  // 3D point in camera frame
  auto Pc = point_in_camera_frame(norm_pixel, t, R, detected_object.label);

  // Transform to output_frame
  keisan::Matrix<4, 4> M = R;
  M[0][3] = t[0][3];
  M[1][3] = t[1][3];
  M[2][3] = t[2][3];

  keisan::Matrix<4, 1> Pw = M * Pc;

  gyakuenki_interfaces::msg::Point3 position;
  position.x = Pw[0][0];
  position.y = Pw[1][0];
  position.z = Pw[2][0];

  return position;
}

gyakuenki_interfaces::msg::ProjectedObjects IPM::map_objects(
  const DetectedObjects::SharedPtr & message)
{
  ProjectedObjects projected_objects;
  projected_objects.header = message->header;

  // Get the camera transform with offset applied expressed in output_frame
  tf2::Transform tf_final;
  try {
    tf_final = get_corrected_camera_transform("base_footprint", message->header.stamp);
  } catch (const std::exception & ex) {
    RCLCPP_WARN(
      this->node->get_logger(), "Could not get corrected camera transform: %s", ex.what());
    return projected_objects;
  }

  tf2::Quaternion q_final = tf_final.getRotation();
  tf2::Vector3 t_final = tf_final.getOrigin();

  // Convert the quaternion to rotation matrix R
  keisan::Matrix<4, 4> R = quat_to_rotation_matrix(q_final);

  // Get the translation matrix
  keisan::Matrix<4, 4> t =
    keisan::translation_matrix(keisan::Point3(t_final.x(), t_final.y(), t_final.z()));

  for (const auto & detected_object : message->detected_objects) {
    ProjectedObject projected_object;

    projected_object.label = detected_object.label;
    projected_object.confidence = detected_object.score;
    projected_object.left = detected_object.left;
    projected_object.top = detected_object.top;
    projected_object.right = detected_object.right;
    projected_object.bottom = detected_object.bottom;
    projected_object.has_projection = false;

    try {
      projected_object.position = map_object(detected_object, R, t);
      projected_object.has_projection = true;
    } catch (std::exception & e) {
      RCLCPP_WARN(this->node->get_logger(), e.what());
    }

    projected_objects.projected_objects.push_back(projected_object);
  }

  return projected_objects;
}

}  // namespace gyakuenki_cpp
