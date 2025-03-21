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

  dnn_detection_subscriber = node->create_subscription<DetectedObjects>(
    "ninshiki_cpp/dnn_detection", 10, [this](const DetectedObjects::SharedPtr message) {
      this->publish_projected_objects(message);
      this->publish_point_clouds(message);
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

void GyakuenkiCppNode::publish_projected_objects(const DetectedObjects::SharedPtr & message)
{
  ProjectedObjects projected_objects;

  for (const auto & detected_object : message->detected_objects) {
    try {
      ProjectedObject projected_object =
        this->ipm->map_object(detected_object, IPM::TYPE_DNN, "base_footprint");

      projected_objects.projected_objects.push_back(projected_object);
    } catch (std::exception & e) {
      RCLCPP_ERROR(this->node->get_logger(), e.what());
    }
  }

  projected_objects_publisher->publish(projected_objects);
}

void GyakuenkiCppNode::publish_point_clouds(const DetectedObjects::SharedPtr & message)
{
  PointCloud2 cloud_msg;
  cloud_msg.header.frame_id = "camera";
  cloud_msg.header.stamp = node->now();

  // Define the fields of PointCloud2
  cloud_msg.height = 1;
  cloud_msg.is_bigendian = false;
  cloud_msg.is_dense = true;

  // Fields: x, y, z
  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
  modifier.setPointCloud2Fields(
    3, "x", 1, PointField::FLOAT32, "y", 1, PointField::FLOAT32, "z", 1, PointField::FLOAT32);

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

  for (const auto & detected_object : message->detected_objects) {
    try {
      auto Pc = this->ipm->get_camera_frame_points(detected_object, IPM::TYPE_DNN);

      *iter_x = Pc[0][0];
      *iter_y = Pc[1][0];
      *iter_z = Pc[2][0];

      ++iter_x;
      ++iter_y;
      ++iter_z;
    } catch (std::exception & e) {
      RCLCPP_WARN(this->node->get_logger(), e.what());
    }
  }

  point_cloud_publisher->publish(cloud_msg);
}
}  // namespace gyakuenki_cpp
