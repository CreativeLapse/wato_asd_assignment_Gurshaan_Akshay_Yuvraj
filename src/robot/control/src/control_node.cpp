#include <chrono>
#include <cmath>
#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode(const rclcpp::NodeOptions& options)
  : Node("control", options),
    control_(robot::ControlCore(this->get_logger())),
    control_period_ms_(0),
    odom_timeout_(0.0),
    odometry_forward_offset_(0.0),
    have_odom_(false),
    robot_x_(0.0),
    robot_y_(0.0),
    robot_yaw_(0.0),
    last_odom_time_(0, 0, RCL_ROS_TIME),
    announced_arrival_(false)
{
  loadParameters();
  control_.configure(params_);

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    path_topic_, 10, std::bind(&ControlNode::onPath, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10, std::bind(&ControlNode::onOdom, this, std::placeholders::_1));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(control_period_ms_), std::bind(&ControlNode::onTimer, this));

  RCLCPP_INFO(this->get_logger(), "Control node ready: up to %.2f m/s, lookahead %.2f-%.2f m",
              params_.max_speed, params_.lookahead_min, params_.lookahead_max);
}

void ControlNode::loadParameters()
{
  path_topic_ = this->declare_parameter<std::string>("path_topic", "/path");
  odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odom/filtered");
  cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  control_period_ms_ = this->declare_parameter<int>("control_period_ms", 100);
  odom_timeout_ = this->declare_parameter<double>("odom_timeout", 0.5);
  odometry_forward_offset_ = this->declare_parameter<double>("odometry_forward_offset", 1.3);

  robot::ControlParams defaults;
  params_.max_speed = this->declare_parameter<double>("max_speed", defaults.max_speed);
  params_.min_speed = this->declare_parameter<double>("min_speed", defaults.min_speed);
  params_.max_angular_speed = this->declare_parameter<double>("max_angular_speed", defaults.max_angular_speed);
  params_.accel = this->declare_parameter<double>("accel", defaults.accel);
  params_.decel = this->declare_parameter<double>("decel", defaults.decel);
  params_.lookahead_min = this->declare_parameter<double>("lookahead_min", defaults.lookahead_min);
  params_.lookahead_max = this->declare_parameter<double>("lookahead_max", defaults.lookahead_max);
  params_.lookahead_gain = this->declare_parameter<double>("lookahead_gain", defaults.lookahead_gain);
  params_.curvature_gain = this->declare_parameter<double>("curvature_gain", defaults.curvature_gain);
  params_.turn_enter_angle = this->declare_parameter<double>("turn_enter_angle", defaults.turn_enter_angle);
  params_.turn_exit_angle = this->declare_parameter<double>("turn_exit_angle", defaults.turn_exit_angle);
  params_.turn_gain = this->declare_parameter<double>("turn_gain", defaults.turn_gain);
  params_.slowdown_distance = this->declare_parameter<double>("slowdown_distance", defaults.slowdown_distance);
  params_.goal_tolerance = this->declare_parameter<double>("goal_tolerance", defaults.goal_tolerance);
}

void ControlNode::onPath(const nav_msgs::msg::Path::SharedPtr path)
{
  control_.setPath(*path);
  announced_arrival_ = false;
  if (path->poses.empty()) {
    RCLCPP_INFO(this->get_logger(), "Received empty path; stopping.");
  } else {
    RCLCPP_INFO(this->get_logger(), "Following new path with %zu waypoints", path->poses.size());
  }
}

void ControlNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  // Odometry reports the lidar, which sits ahead of the wheel axle. The
  // axle is what actually follows the arc, and it doesn't sweep sideways
  // when the robot turns in place.
  robot_yaw_ = yawFromQuaternion(odom->pose.pose.orientation);
  robot_x_ = odom->pose.pose.position.x - odometry_forward_offset_ * std::cos(robot_yaw_);
  robot_y_ = odom->pose.pose.position.y - odometry_forward_offset_ * std::sin(robot_yaw_);
  last_odom_time_ = this->now();
  have_odom_ = true;
}

void ControlNode::onTimer()
{
  if (!have_odom_) {
    return;
  }

  // Always publish something: zeros tell the robot to stop rather than
  // leaving it coasting on its last command.
  geometry_msgs::msg::Twist cmd;
  if ((this->now() - last_odom_time_).seconds() > odom_timeout_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Odometry is stale; holding the robot still.");
  } else {
    cmd = control_.computeCommand(robot_x_, robot_y_, robot_yaw_, control_period_ms_ / 1000.0);
  }

  if (control_.hasPath() && control_.goalReached(robot_x_, robot_y_) && !announced_arrival_) {
    RCLCPP_INFO(this->get_logger(), "Reached the end of the path.");
    announced_arrival_ = true;
  }

  cmd_vel_pub_->publish(cmd);
}

double ControlNode::yawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

#ifndef WATO_NODE_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
#endif
