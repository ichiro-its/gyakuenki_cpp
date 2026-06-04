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

using namespace std::chrono_literals;

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

  markers_publisher = node->create_publisher<MarkerArray>("gyakuenki_cpp/markers", 10);

  dnn_detection_subscriber = node->create_subscription<DetectedObjects>(
    "ninshiki_cpp/dnn_detection", 10,
    [this](const DetectedObjects::SharedPtr message) { this->publish(message); });

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

  // Publisher for visualizing the corrected camera pose in RViz
  corrected_camera_publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "gyakuenki_cpp/corrected_camera_pose", 10);

  node_timer = node->create_wall_timer(8ms, [this]() {
    try {
      tf2::Transform tf_final =
        ipm->get_corrected_camera_transform("base_footprint", rclcpp::Time(0));

      geometry_msgs::msg::PoseStamped camera_pose;
      camera_pose.header.stamp = this->node->get_clock()->now();
      camera_pose.header.frame_id = "base_footprint";
      camera_pose.pose.position.x = tf_final.getOrigin().x();
      camera_pose.pose.position.y = tf_final.getOrigin().y();
      camera_pose.pose.position.z = tf_final.getOrigin().z();
      camera_pose.pose.orientation = ipm->tf2_to_msg(tf_final.getRotation());

      corrected_camera_publisher->publish(camera_pose);

    } catch (const std::exception & ex) {
      RCLCPP_WARN(
        this->node->get_logger(), "Could not get corrected camera transform: %s", ex.what());
    }
  });
}

void GyakuenkiCppNode::publish(const DetectedObjects::SharedPtr & message)
{
  auto projected_objects = this->ipm->map_objects(message);

  projected_objects_publisher->publish(projected_objects);
  publish_markers(projected_objects, message->header.stamp);
}

void GyakuenkiCppNode::publish_markers(
  const gyakuenki_interfaces::msg::ProjectedObjects & projected_objects, const rclcpp::Time & stamp)
{
  MarkerArray markers;
  uint8_t id = 0;

  for (const auto & obj : projected_objects.projected_objects) {
    if (!obj.has_projection) {
      continue;
    }

    Marker marker;
    marker.header.frame_id = "base_footprint";
    marker.header.stamp = stamp;
    marker.ns = obj.label;
    marker.id = id++;

    if (obj.label == "ball") {
      marker.type = Marker::SPHERE;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
    } else if (obj.label == "goalpost") {
      marker.type = Marker::CUBE;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
    } else if (obj.label == "robot") {
      marker.type = Marker::CYLINDER;
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
    } else if (obj.label == "L-intersection") {
      marker.type = Marker::LINE_LIST;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    } else if (obj.label == "T-intersection") {
      marker.type = Marker::LINE_LIST;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
    } else {  // X-intersection
      marker.type = Marker::LINE_LIST;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    }

    marker.action = Marker::ADD;

    marker.pose.position.x = obj.position.x;
    marker.pose.position.y = obj.position.y;
    marker.pose.position.z = obj.position.z;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;

    markers.markers.push_back(marker);
  }

  markers_publisher->publish(markers);
}

}  // namespace gyakuenki_cpp
