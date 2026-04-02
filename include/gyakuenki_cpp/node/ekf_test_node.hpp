#ifndef GYAKUENKI_CPP__NODE__EKF_TEST_NODE_HPP_
#define GYAKUENKI_CPP__NODE__EKF_TEST_NODE_HPP_

#include <memory>
#include <string>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "ninshiki_interfaces/msg/detected_objects.hpp"
#include "ninshiki_interfaces/msg/detected_object.hpp"

#include "gyakuenki_interfaces/msg/projected_object.hpp"
#include "gyakuenki_interfaces/msg/projected_objects.hpp"
#include "gyakuenki_interfaces/srv/get_camera_offset.hpp"
#include "gyakuenki_interfaces/srv/update_camera_offset.hpp"

#include "gyakuenki_cpp/projections/ipm.hpp"
#include "keisan/ekf/ekf_ball.hpp"

namespace gyakuenki_cpp
{

class EkfTestNode
{
public:
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using Marker = visualization_msgs::msg::Marker;
  using DetectedObjects = ninshiki_interfaces::msg::DetectedObjects;
  using DetectedObject = ninshiki_interfaces::msg::DetectedObject;
  using ProjectedObjects = gyakuenki_interfaces::msg::ProjectedObjects;
  using ProjectedObject = gyakuenki_interfaces::msg::ProjectedObject;
  using GetCameraOffset = gyakuenki_interfaces::srv::GetCameraOffset;
  using UpdateCameraOffset = gyakuenki_interfaces::srv::UpdateCameraOffset;
  
  EkfTestNode(const std::shared_ptr<rclcpp::Node> & node, const std::string & config_path);

private:
  void dnn_detection_callback(const ninshiki_interfaces::msg::DetectedObjects::SharedPtr message);

  void load_config();

  std::shared_ptr<rclcpp::Node> node;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  
  rclcpp::Subscription<ninshiki_interfaces::msg::DetectedObjects>::SharedPtr dnn_subscriber;
  
  rclcpp::Publisher<gyakuenki_interfaces::msg::ProjectedObjects>::SharedPtr projected_objects_publisher;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_publisher;

  rclcpp::Service<GetCameraOffset>::SharedPtr get_camera_offset_service;
  rclcpp::Service<UpdateCameraOffset>::SharedPtr update_camera_offset_service;

  std::shared_ptr<IPM> ipm;
  keisan::ekf_ball ball_ekf_;
  rclcpp::TimerBase::SharedPtr config_timer_;
  
  bool ball_initialized_;
  rclcpp::Time last_ball_time_;

  double lost_ball_duration;

  std::string ekf_config_path_;
  std::filesystem::file_time_type last_modified_time_;
  
  double grass_friction_ = 0.15;
  double prediction_noise_pos_ = 1e-3;
  double prediction_noise_vel_ = 1e-2;
  double camera_noise_pos_ = 0.01;
  bool is_testing = false;
  bool is_validating = false;

  bool has_first_measurement = false;
  double first_raw_x = 0.0;
  double first_raw_y = 0.0;
  rclcpp::Time first_msg_time;
  rclcpp::Time last_msg_time;

  // used for testing
  double frozen_x;
  double frozen_y;
};

}  // namespace gyakuenki_cpp

#endif  // GYAKUENKI_CPP__NODE__EKF_TEST_NODE_HPP_