#include "gyakuenki_cpp/node/ekf_test_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <nlohmann/json.hpp>

#include "gyakuenki_interfaces/srv/get_camera_offset.hpp"
#include "gyakuenki_interfaces/srv/update_camera_offset.hpp"

namespace gyakuenki_cpp
{

EkfTestNode::EkfTestNode(
  const std::shared_ptr<rclcpp::Node> & node, const std::string & config_path)
: node(node), ball_initialized_(false), lost_ball_duration(0.0)
{
  std::string home_dir = getenv("HOME");
  ekf_config_path_ = home_dir + "/ichiro-ws/src/gyakuenki_cpp/config/ekf.json";

  load_config();

  tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node, false);

  ipm = std::make_shared<IPM>(node, tf_buffer, tf_listener, config_path);

  projected_objects_publisher = node->create_publisher<gyakuenki_interfaces::msg::ProjectedObjects>(
    "gyakuenki_cpp/projected_objects", 10);

  markers_publisher =
    node->create_publisher<visualization_msgs::msg::MarkerArray>("ekf_test/markers", 10);

  dnn_subscriber = node->create_subscription<ninshiki_interfaces::msg::DetectedObjects>(
    "ninshiki_cpp/dnn_detection", 10,
    [this](const ninshiki_interfaces::msg::DetectedObjects::SharedPtr message) {
      this->dnn_detection_callback(message);
    });

  // Camera Offset Services
  get_camera_offset_service = node->create_service<GetCameraOffset>(
    "camera/get_camera_offset", [this, config_path](
                                  const GetCameraOffset::Request::SharedPtr request,
                                  GetCameraOffset::Response::SharedPtr response) {
      this->ipm->load_config(config_path);
      auto camera_offset = this->ipm->get_camera_offset();
      response->position_x = camera_offset.position.x;
      response->position_y = camera_offset.position.y;
      response->position_z = camera_offset.position.z;

      response->roll = camera_offset.roll.degree();
      response->pitch = camera_offset.pitch.degree();
      response->yaw = camera_offset.yaw.degree();

      response->status = true;
    });

  update_camera_offset_service = node->create_service<UpdateCameraOffset>(
    "camera/update_camera_offset", [this](
                                     const UpdateCameraOffset::Request::SharedPtr request,
                                     UpdateCameraOffset::Response::SharedPtr response) {
      this->ipm->set_config(
        request->position_x, request->position_y, request->position_z, request->roll,
        request->pitch, request->yaw);

      if (request->save) {
        this->ipm->save_config();
      }

      response->status = true;
    });
}

void EkfTestNode::load_config()
{
  try {
    if (!std::filesystem::exists(ekf_config_path_)) {
      std::ofstream file(ekf_config_path_);
      if (file.is_open()) {
        nlohmann::json j;
        j["grass_friction"] = 0.05;
        j["ekf_q_pos"] = 0.001;
        j["ekf_q_vel"] = 0.001;
        j["ekf_r_pos"] = 0.01;
        file << j;
        file.close();
      }
      return;
    }

    auto curr_time = std::filesystem::last_write_time(ekf_config_path_);

    if (curr_time != last_modified_time_) {
      std::ifstream file(ekf_config_path_);

      if (file.is_open()) {
        nlohmann::json j;
        file >> j;

        if (j.contains("grass_friction")) grass_friction_ = j["grass_friction"];
        if (j.contains("ekf_q_pos")) q_pos_ = j["ekf_q_pos"];
        if (j.contains("ekf_q_vel")) q_vel_ = j["ekf_q_vel"];
        if (j.contains("ekf_r_pos")) r_pos_ = j["ekf_r_pos"];
        if (j.contains("enable_testing")) is_testing = j["enable_testing"];

        curr_time = last_modified_time_;
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(node->get_logger(), "error load config: %s", e.what());
  }
}

void EkfTestNode::dnn_detection_callback(
  const ninshiki_interfaces::msg::DetectedObjects::SharedPtr message)
{
  load_config();

  rclcpp::Time now = node->now();
  double delta_sec = ball_initialized_ ? (now - last_ball_time_).seconds() : 0.0;
  double max_confidence = -1.0;
  ninshiki_interfaces::msg::DetectedObject best_ball;
  bool ball_found = false;

  for (const auto & obj : message->detected_objects) {
    if (obj.label == "ball" && obj.score > max_confidence) {
      max_confidence = obj.score;
      best_ball = obj;
      ball_found = true;
    }
  }

  if (ball_found) {
    last_ball_time_ = now;
  }

  double raw_x = 0.0;
  double raw_y = 0.0;

  ball_ekf_.setQ(q_pos_, q_vel_);
  ball_ekf_.setR(r_pos_);
  ball_ekf_.setFriction(grass_friction_);

  if (!ball_found) {
    if (!ball_initialized_) return;

    lost_ball_duration += delta_sec;
    ball_ekf_.predict(delta_sec);
  } else {
    try {
      if (lost_ball_duration > 10.0) ball_initialized_ = false;
      lost_ball_duration = 0.0;

      keisan::Matrix<4, 1> Pc;
      ProjectedObject projected_ball =
        this->ipm->map_object(best_ball, message->header.stamp, "base_footprint", Pc);

      raw_x = projected_ball.position.x;
      raw_y = projected_ball.position.y;

      if (!ball_initialized_) {
        ball_ekf_.init(raw_x, raw_y, 0.0, 0.0);
        last_ball_time_ = now;
        ball_initialized_ = true;
      } else {
        if (delta_sec > 0.0) {
          ball_ekf_.predict(delta_sec);

          keisan::Matrix<2, 1> z;
          z[0][0] = projected_ball.position.x;
          z[1][0] = projected_ball.position.y;
          ball_ekf_.update(z);
        }
      }
    } catch (std::exception & e) {
      RCLCPP_ERROR(this->node->get_logger(), e.what());
    }
  }

  auto pos = ball_ekf_.getPosition();
  auto vel = ball_ekf_.getVelocity();

  double x_curr = pos[0][0];
  double y_curr = pos[1][0];
  double vx = vel[0][0];
  double vy = vel[1][0];
  double v_mag = std::sqrt(vx * vx + vy * vy);

  double target_time = 5.0;
  auto future_state_vector = ball_ekf_.predictFuture(target_time);

  if (is_testing == true) {
    if (v_mag > 0.5 && is_validating == false) {
      frozen_x = future_state_vector.back()[0][0];
      frozen_y = future_state_vector.back()[1][0];
      is_validating = true;
    } else if (v_mag < 0.005 && is_validating == true) {
      double error_prediction = hypot(x_curr - frozen_x, y_curr - frozen_y);
      RCLCPP_INFO(node->get_logger(), "EKF prediction error: %.2f", error_prediction);
      is_validating = false;
    }
  } else {
    is_validating = false;
  }

  double shadow_x = x_curr;
  double shadow_y = y_curr;

  if (!future_state_vector.empty()) {
    shadow_x = future_state_vector.back()[0][0];
    shadow_y = future_state_vector.back()[1][0];
  }

  double predicted_distance = std::sqrt(
    (shadow_x - x_curr) * (shadow_x - x_curr) + (shadow_y - y_curr) * (shadow_y - y_curr));

  RCLCPP_INFO(node->get_logger(), "\n--- [EKF BALL] ---");
  RCLCPP_INFO(node->get_logger(), "Status        : %s", (ball_found ? "BALL FOUND" : "BALL LOST"));
  if (ball_found) {
    RCLCPP_INFO(node->get_logger(), "1. Raw (IPM) : X=%.3f, Y=%.3f", raw_x, raw_y);
  } else {
    RCLCPP_INFO(node->get_logger(), "Lost Duration   : %.2f secs", lost_ball_duration);
  }
  RCLCPP_INFO(
    node->get_logger(), "2. Filtered (EKF)  : X=%.3f, Y=%.3f | velocity = %.3f m/s", x_curr, y_curr,
    v_mag);
  RCLCPP_INFO(
    node->get_logger(), "3. Shadow (%.3fs)  : X=%.3f, Y=%.3f | distance = %.3f m", target_time,
    shadow_x, shadow_y, predicted_distance);
  RCLCPP_INFO(node->get_logger(), "---------------------------------");

  gyakuenki_interfaces::msg::ProjectedObjects published_objects;

  gyakuenki_interfaces::msg::ProjectedObject filtered_ball;
  filtered_ball.label = "ball";
  filtered_ball.position.x = x_curr;
  filtered_ball.position.y = y_curr;
  filtered_ball.position.z = 0.0;
  filtered_ball.confidence = max_confidence;
  published_objects.projected_objects.push_back(filtered_ball);

  gyakuenki_interfaces::msg::ProjectedObject shadow_ball;
  shadow_ball.label = "shadow_ball";
  shadow_ball.position.x = shadow_x;
  shadow_ball.position.y = shadow_y;
  shadow_ball.position.z = 0.0;
  shadow_ball.confidence = max_confidence;
  published_objects.projected_objects.push_back(shadow_ball);

  if (is_validating) {
    gyakuenki_interfaces::msg::ProjectedObject frozen_ball;
    frozen_ball.label = "frozen_ball";
    frozen_ball.position.x = frozen_x;
    frozen_ball.position.y = frozen_y;
    frozen_ball.position.z = 0.0;
    frozen_ball.confidence = max_confidence;
    published_objects.projected_objects.push_back(frozen_ball);
  }

  projected_objects_publisher->publish(published_objects);

  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker m_ekf;
  m_ekf.header.frame_id = "base_footprint";
  m_ekf.header.stamp = now;
  m_ekf.ns = "ekf_ball";
  m_ekf.id = 0;
  m_ekf.type = visualization_msgs::msg::Marker::SPHERE;
  m_ekf.action = visualization_msgs::msg::Marker::ADD;
  m_ekf.pose.position.x = x_curr;
  m_ekf.pose.position.y = y_curr;
  m_ekf.pose.position.z = 0.0765;
  m_ekf.scale.x = 0.153;
  m_ekf.scale.y = 0.153;
  m_ekf.scale.z = 0.153;
  m_ekf.color.a = 1.0;
  m_ekf.color.r = 1.0;
  m_ekf.color.g = 0.0;
  m_ekf.color.b = 0.0;
  markers.markers.push_back(m_ekf);

  int shadow_id = 1;

  for (const auto& pt : future_state_vector) {
    visualization_msgs::msg::Marker m_shadow;
    m_shadow.header.frame_id = "base_footprint";
    m_shadow.header.stamp = now;
    m_shadow.ns = "shadow_ball";
    m_shadow.id = shadow_id++;
    m_shadow.type = visualization_msgs::msg::Marker::SPHERE;
    m_shadow.action = visualization_msgs::msg::Marker::ADD;
    
    // Set posisi dari data matriks EKF
    m_shadow.pose.position.x = pt[0][0];
    m_shadow.pose.position.y = pt[1][0];
    m_shadow.pose.position.z = 0.0765;
    
    m_shadow.scale.x = 0.153;
    m_shadow.scale.y = 0.153;
    m_shadow.scale.z = 0.153;
    
    m_shadow.color.a = 0.5;
    m_shadow.color.r = 0.0;
    m_shadow.color.g = 1.0;
    m_shadow.color.b = 0.0;
    
    m_shadow.lifetime = rclcpp::Duration::from_seconds(0.1);

    markers.markers.push_back(m_shadow);
  }

  static bool delete_frozen = false;
  if (is_validating) {
    visualization_msgs::msg::Marker m_frozen;
    m_frozen.header.frame_id = "base_footprint";
    m_frozen.header.stamp = now;
    m_frozen.ns = "frozen_ball";
    m_frozen.id = 999;
    m_frozen.type = visualization_msgs::msg::Marker::SPHERE;
    m_frozen.action = visualization_msgs::msg::Marker::ADD;
    
    m_frozen.pose.position.x = frozen_x;
    m_frozen.pose.position.y = frozen_y;
    m_frozen.pose.position.z = 0.0765;
    
    m_frozen.scale.x = 0.16;
    m_frozen.scale.y = 0.16;
    m_frozen.scale.z = 0.16;
    
    m_frozen.color.a = 1.0;
    m_frozen.color.r = 0.0;
    m_frozen.color.g = 0.0;
    m_frozen.color.b = 1.0;
    
    markers.markers.push_back(m_frozen);
    delete_frozen = true;
  } else if (delete_frozen) {
      visualization_msgs::msg::Marker m_frozen_delete;
      m_frozen_delete.header.frame_id = "base_footprint";
      m_frozen_delete.header.stamp = now;
      m_frozen_delete.ns = "frozen_ball";
      m_frozen_delete.id = 999;
      m_frozen_delete.action = visualization_msgs::msg::Marker::DELETE;
      
      markers.markers.push_back(m_frozen_delete);
      delete_frozen = false;
  }

  markers_publisher->publish(markers);
}

}  // namespace gyakuenki_cpp