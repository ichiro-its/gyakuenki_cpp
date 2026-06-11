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

#include "jitsuyo/config.hpp"
#include "nlohmann/json.hpp"

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

  // Line extraction config (optional section, defaults preserved when absent)
  max_line_points = 15;
  min_field_contour_area = 200.0;

  nlohmann::json offset_config;
  if (jitsuyo::load_config(config_path, "camera_offset.json", offset_config)) {
    nlohmann::json line_section;
    if (jitsuyo::assign_val(offset_config, "line_extraction", line_section)) {
      jitsuyo::assign_val(line_section, "max_points", max_line_points);
      jitsuyo::assign_val(line_section, "min_field_contour_area", min_field_contour_area);
    }
  }

  projected_lines_publisher =
    node->create_publisher<ProjectedObjects>("gyakuenki_cpp/projected_lines", 10);

  color_detection_subscriber = node->create_subscription<Contours>(
    "ninshiki_cpp/color_detection", 10,
    [this](const Contours::SharedPtr message) { this->process_line_contours(message); });

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
    } else if (obj.label == "line") {
      marker.type = Marker::SPHERE;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
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

void GyakuenkiCppNode::process_line_contours(const Contours::SharedPtr & message)
{
  ProjectedObjects projected_lines;
  projected_lines.header = message->header;

  int width = ipm->get_camera_info().image_width();
  int height = ipm->get_camera_info().image_height();

  std::vector<cv::Point> field_points;
  std::vector<std::vector<cv::Point>> white_polygons;

  for (const auto & contour : message->contours) {
    std::vector<cv::Point> points;
    points.reserve(contour.contour.size());
    for (const auto & point : contour.contour) {
      points.emplace_back(static_cast<int>(point.x), static_cast<int>(point.y));
    }

    if (points.size() < 3) {
      continue;
    }

    if (contour.name == "field") {
      if (cv::contourArea(points) < min_field_contour_area) {
        continue;
      }
      field_points.insert(field_points.end(), points.begin(), points.end());
    } else if (contour.name == "white") {
      white_polygons.push_back(points);
    }
  }

  if (field_points.size() >= 3 && !white_polygons.empty()) {
    // Field mask: convex hull over all field contours, same recipe basho uses
    std::vector<cv::Point> hull;
    cv::convexHull(field_points, hull);

    cv::Mat field_mask = cv::Mat::zeros(height, width, CV_8UC1);
    cv::fillConvexPoly(field_mask, hull, 255);

    cv::Mat white_mask = cv::Mat::zeros(height, width, CV_8UC1);
    cv::fillPoly(white_mask, white_polygons, 255);

    // Keep only white pixels inside the field
    cv::Mat line_mask;
    cv::bitwise_and(field_mask, white_mask, line_mask);

    std::vector<cv::Point> line_pixels;
    cv::findNonZero(line_mask, line_pixels);

    if (!line_pixels.empty()) {
      try {
        tf2::Transform tf_final =
          ipm->get_corrected_camera_transform("base_footprint", message->header.stamp);

        keisan::Matrix<4, 4> R = ipm->quat_to_rotation_matrix(tf_final.getRotation());
        keisan::Matrix<4, 4> t = keisan::translation_matrix(keisan::Point3(
          tf_final.getOrigin().x(), tf_final.getOrigin().y(), tf_final.getOrigin().z()));

        // findNonZero returns pixels row-major from the top; iterate from the
        // end so sampling favors lower rows (closer = smaller projection error)
        int step = std::max(1, static_cast<int>(line_pixels.size()) / max_line_points);
        int sampled = 0;

        for (int i = static_cast<int>(line_pixels.size()) - 1;
             i >= 0 && sampled < max_line_points; i -= step) {
          cv::Point2d pixel(line_pixels[i].x, line_pixels[i].y);

          ProjectedObject projected_line;
          projected_line.label = "line";
          projected_line.confidence = 1.0;
          projected_line.left = line_pixels[i].x;
          projected_line.top = line_pixels[i].y;
          projected_line.right = 0;
          projected_line.bottom = 0;
          projected_line.has_projection = false;

          try {
            projected_line.position = ipm->map_pixel(pixel, R, t, "line");
            projected_line.has_projection = true;
            sampled++;
            projected_lines.projected_objects.push_back(projected_line);
          } catch (const std::exception &) {
            // Horizon-confidence or plane-intersection rejection; skip this point
          }
        }
      } catch (const std::exception & ex) {
        RCLCPP_WARN(
          this->node->get_logger(), "Could not get camera transform for lines: %s", ex.what());
      }
    }
  }

  // Publish even when empty so the consumer clears stale line observations
  projected_lines_publisher->publish(projected_lines);
  publish_markers(projected_lines, message->header.stamp);
}

}  // namespace gyakuenki_cpp
