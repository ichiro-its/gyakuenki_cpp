#ifndef GYAKUENKI_CPP__NODE__EKF_TEST_NODE_HPP_
#define GYAKUENKI_CPP__NODE__EKF_TEST_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "ninshiki_interfaces/msg/detected_objects.hpp"
#include "ninshiki_interfaces/msg/detected_object.hpp"

#include "gyakuenki_interfaces/msg/projected_objects.hpp"
#include "gyakuenki_interfaces/msg/projected_object.hpp"

#include "gyakuenki_cpp/projections/ipm.hpp"
#include "keisan/ekf/ekf_ball.hpp"

namespace gyakuenki_cpp
{

class EkfTestNode
{
public:
  using ProjectedObject = gyakuenki_interfaces::msg::ProjectedObject;
  
  EkfTestNode(const std::shared_ptr<rclcpp::Node> & node, const std::string & config_path);

private:
  void dnn_detection_callback(const ninshiki_interfaces::msg::DetectedObjects::SharedPtr message);

  std::shared_ptr<rclcpp::Node> node;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  
  rclcpp::Subscription<ninshiki_interfaces::msg::DetectedObjects>::SharedPtr dnn_subscriber;
  
  rclcpp::Publisher<gyakuenki_interfaces::msg::ProjectedObjects>::SharedPtr projected_objects_publisher;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_publisher;

  std::shared_ptr<IPM> ipm;
  keisan::ekf_ball ball_ekf_;
  
  bool ball_initialized_;
  rclcpp::Time last_ball_time_;

  double grass_friction_;

  double lost_ball_duration = 0.0;
};

}  // namespace gyakuenki_cpp

#endif  // GYAKUENKI_CPP__NODE__EKF_TEST_NODE_HPP_