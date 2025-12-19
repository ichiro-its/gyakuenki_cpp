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

#include "gyakuenki_cpp/node/gyakuenki_cpp_node.hpp"

namespace gyakuenki_cpp
{

GyakuenkiCppNode::GyakuenkiCppNode(
  const std::shared_ptr<rclcpp::Node> & node, const std::string & config_path)
: node(node)
{
  using GetCameraOffset = gyakuenki_interfaces::srv::GetCameraOffset;
  using UpdateCameraOffset = gyakuenki_interfaces::srv::UpdateCameraOffset;

  tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node, false);

  ipm = std::make_shared<IPM>(node, tf_buffer, tf_listener, config_path);

  projected_objects_publisher =
    node->create_publisher<ProjectedObjects>("gyakuenki_cpp/projected_objects", 10);

  projected_ball_publisher =
    node->create_publisher<ProjectedObject>("gyakuenki_cpp/projected_ball", 10);

  markers_publisher = node->create_publisher<MarkerArray>("gyakuenki_cpp/markers", 10);

  dnn_detection_subscriber = node->create_subscription<DetectedObjects>(
    "ninshiki_cpp/dnn_detection", 10,
    [this](const DetectedObjects::SharedPtr message) { this->publish(message); });

  ball_detection_subscriber = node->create_subscription<DetectedObject>(
    "soccer/ball_detection", 10,
    [this](const DetectedObject::SharedPtr message) {
      try {
        keisan::Matrix<4, 1> Pc;
        ProjectedObject projected_ball =
          this->ipm->map_object(*message, "base_footprint", Pc);

        rclcpp::Time now = this->node->now();

        if (!ball_initialized_) {
          ball_ekf_.init(Pc[0][0], Pc[1][0]);
          last_ball_time_ = now;
          ball_initialized_ = true;
          return;
        }

        double dt = (now - last_ball_time_).seconds();
        last_ball_time_ = now;

        if (dt <= 0.0) {
          return;
        }

        ball_ekf_.predict(dt);

        keisan::Matrix<2,1> z;
        z[0][0] = Pc[0][0];
        z[1][0] = Pc[1][0];

        ball_ekf_.update(z);

        auto pos = ball_ekf_.getPosition();

        projected_ball.position.x = pos(0,0);
        projected_ball.position.y = pos(1,0);

        projected_ball_publisher->publish(projected_ball);
      } catch (std::exception & e) {
        RCLCPP_ERROR(this->node->get_logger(), e.what());
      }
    }
  );

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

void GyakuenkiCppNode::publish(const DetectedObjects::SharedPtr & message)
{
  ProjectedObjects projected_objects;
  MarkerArray markers;

  uint8_t id = 0;
  for (const auto & detected_object : message->detected_objects) {
    try {
      keisan::Matrix<4, 1> Pc;
      ProjectedObject projected_object =
        this->ipm->map_object(detected_object, "base_footprint", Pc);

      projected_objects.projected_objects.push_back(projected_object);

      Marker marker;
      marker.header.frame_id = "camera";
      marker.header.stamp = node->now();
      marker.ns = projected_object.label;
      marker.id = id++;

      if (projected_object.label == "ball") {
        marker.type = Marker::SPHERE;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
      } else if (projected_object.label == "goalpost") {
        marker.type = Marker::CUBE;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
      } else if (projected_object.label == "robot") {
        marker.type = Marker::CYLINDER;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
      } else if (projected_object.label == "L-intersection") {
        marker.type = Marker::LINE_LIST;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
      } else if (projected_object.label == "T-intersection") {
        marker.type = Marker::LINE_LIST;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
      } else if (projected_object.label == "X-intersection") {
        marker.type = Marker::LINE_LIST;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
      }

      marker.action = Marker::ADD;
      marker.pose.position.x = Pc[0][0];
      marker.pose.position.y = Pc[1][0];
      marker.pose.position.z = Pc[2][0];

      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;

      marker.color.a = 1.0;

      markers.markers.push_back(marker);
    } catch (std::exception & e) {
      RCLCPP_ERROR(this->node->get_logger(), e.what());
    }
  }

  projected_objects_publisher->publish(projected_objects);
  markers_publisher->publish(markers);
}
}  // namespace gyakuenki_cpp
