#include "gyakuenki_cpp/node/ekf_test_node.hpp"
#include <cmath>

namespace gyakuenki_cpp
{

EkfTestNode::EkfTestNode(const std::shared_ptr<rclcpp::Node> & node, const std::string & config_path)
: node(node), ball_initialized_(false)
{
  node->declare_parameter("grass_friction", 0.15);

  tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node, false);

  ipm = std::make_shared<IPM>(node, tf_buffer, tf_listener, config_path);

  projected_objects_publisher = node->create_publisher<gyakuenki_interfaces::msg::ProjectedObjects>(
    "gyakuenki_cpp/projected_objects", 10);
    
  markers_publisher = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "ekf_test/markers", 10);

  dnn_subscriber = node->create_subscription<ninshiki_interfaces::msg::DetectedObjects>(
    "ninshiki_cpp/dnn_detection", 10,
    [this](const ninshiki_interfaces::msg::DetectedObjects::SharedPtr message) {
      this->dnn_detection_callback(message);
    });
}

void EkfTestNode::dnn_detection_callback(const ninshiki_interfaces::msg::DetectedObjects::SharedPtr message)
{
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

  if (!ball_found) return;
  // todo:
  // implement prediksi letak bola 3 detik ke depan
  // init ulang kalau lost ball > 10 secs dan nemu bola
  // tuning Q & R
  // selama lost ball, tetep predict sm update dan gambar ke ichiro app

  try {
    keisan::Matrix<4, 1> Pc;
    ProjectedObject projected_ball =
        this->ipm->map_object(best_ball, "base_footprint", Pc);

    rclcpp::Time now = node->now();
    
    if (!ball_initialized_) {
      ball_ekf_.init(projected_ball.position.x, projected_ball.position.y, 0.0, 0.0); // tambah Q R
      last_ball_time_ = now;
      ball_initialized_ = true;
      return;
    }

    double dt = (now - last_ball_time_).seconds();
    last_ball_time_ = now;

    if (dt > 0.0) {
      ball_ekf_.predict(dt);

      keisan::Matrix<2, 1> z;
      z[0][0] = projected_ball.position.x;
      z[1][0] = projected_ball.position.y;
      ball_ekf_.update(z);
    }

    auto pos = ball_ekf_.getPosition();
    auto vel = ball_ekf_.getVelocity();

    double x_curr = pos[0][0];
    double y_curr = pos[1][0];
    double vx = vel[0][0];
    double vy = vel[1][0];
    double v_mag = std::sqrt(vx*vx + vy*vy);

    node->get_parameter("grass_friction", grass_friction_);
    
    double shadow_x = x_curr;
    double shadow_y = y_curr;
    double stop_distance = 0.0;
    
    if (v_mag > 0.05 && grass_friction_ > 0.0) {
      double gravity = 9.81;
      double acceleration = grass_friction_ * gravity;
      stop_distance = (v_mag * v_mag) / (2.0 * acceleration);
      
      shadow_x = x_curr + stop_distance * (vx / v_mag);
      shadow_y = y_curr + stop_distance * (vy / v_mag);
    }

    RCLCPP_INFO(node->get_logger(), "\n--- [EKF BALL TRACKING DEBUG] ---");
    RCLCPP_INFO(node->get_logger(), "1. Mentah (IPM) : X=%.3f, Y=%.3f", projected_ball.position.x, projected_ball.position.y);
    RCLCPP_INFO(node->get_logger(), "2. Halus (EKF)  : X=%.3f, Y=%.3f | Kecepatan = %.3f m/s", x_curr, y_curr, v_mag);
    RCLCPP_INFO(node->get_logger(), "3. Titik Henti  : X=%.3f, Y=%.3f | Jarak Luncur = %.3f m", shadow_x, shadow_y, stop_distance);
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

    projected_objects_publisher->publish(published_objects);

    visualization_msgs::msg::MarkerArray markers;
    
    visualization_msgs::msg::Marker m_ekf;
    m_ekf.header.frame_id = "base_footprint";
    m_ekf.header.stamp = now;
    m_ekf.ns = "ekf_ball";
    m_ekf.id = 0;
    m_ekf.type = visualization_msgs::msg::Marker::SPHERE;
    m_ekf.action = visualization_msgs::msg::Marker::ADD;
    m_ekf.pose.position.x = x_curr; m_ekf.pose.position.y = y_curr; m_ekf.pose.position.z = 0.0765; 
    m_ekf.scale.x = 0.153; m_ekf.scale.y = 0.153; m_ekf.scale.z = 0.153; 
    m_ekf.color.a = 1.0; m_ekf.color.r = 1.0; m_ekf.color.g = 0.0; m_ekf.color.b = 0.0;
    markers.markers.push_back(m_ekf);

    visualization_msgs::msg::Marker m_shadow;
    m_shadow.header.frame_id = "base_footprint";
    m_shadow.header.stamp = now;
    m_shadow.ns = "shadow_ball";
    m_shadow.id = 1;
    m_shadow.type = visualization_msgs::msg::Marker::SPHERE;
    m_shadow.action = visualization_msgs::msg::Marker::ADD;
    m_shadow.pose.position.x = shadow_x; m_shadow.pose.position.y = shadow_y; m_shadow.pose.position.z = 0.0765;
    m_shadow.scale.x = 0.153; m_shadow.scale.y = 0.153; m_shadow.scale.z = 0.153;
    m_shadow.color.a = 0.5; m_shadow.color.r = 0.0; m_shadow.color.g = 1.0; m_shadow.color.b = 0.0;
    markers.markers.push_back(m_shadow);

    markers_publisher->publish(markers);

  } catch (std::exception & e) {
    RCLCPP_ERROR(this->node->get_logger(), "EKF Test Error: %s", e.what());
  }
}

}  // namespace gyakuenki_cpp