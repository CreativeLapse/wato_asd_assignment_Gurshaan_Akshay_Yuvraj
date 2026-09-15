#include <chrono>
#include <cmath>
#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode(const rclcpp::NodeOptions& options)
  : Node("control", options),
    control_(robot::ControlCore(this->get_logger())),
    control_period_ms_(0),
    lookahead_distance_(0.0),
    linear_speed_(0.0),
    max_angular_speed_(0.0),
    goal_tolerance_(0.0),
    turn_in_place_angle_(0.0),
    slowdown_distance_(0.0),
    odometry_forward_offset_(0.0),
    have_odom_(false),
    robot_x_(0.0),
    robot_y_(0.0),
    robot_yaw_(0.0),
    announced_arrival_(false)
{
  loadParameters();
  control_.configure(
    lookahead_distance_, linear_speed_, max_angular_speed_,
    goal_tolerance_, turn_in_place_angle_, slowdown_distance_);

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    path_topic_, 10, std::bind(&ControlNode::onPath, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10, std::bind(&ControlNode::onOdom, this, std::placeholders::_1));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(control_period_ms_), std::bind(&ControlNode::onTimer, this));

  RCLCPP_INFO(this->get_logger(), "Control node ready: %.2f m/s, lookahead %.2f m",
              linear_speed_, lookahead_distance_);
}

void ControlNode::loadParameters()
{
  path_topic_ = this->declare_parameter<std::string>("path_topic", "/path");
  odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odom/filtered");
  cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  control_period_ms_ = this->declare_parameter<int>("control_period_ms", 50);
  lookahead_distance_ = this->declare_parameter<double>("lookahead_distance", 1.0);
  linear_speed_ = this->declare_parameter<double>("linear_speed", 0.8);
  max_angular_speed_ = this->declare_parameter<double>("max_angular_speed", 1.0);
  goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.2);
  turn_in_place_angle_ = this->declare_parameter<double>("turn_in_place_angle", 0.8);
  slowdown_distance_ = this->declare_parameter<double>("slowdown_distance", 1.6);
  odometry_forward_offset_ = this->declare_parameter<double>("odometry_forward_offset", 1.3);
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
  robot_yaw_ = yawFromQuaternion(odom->pose.pose.orientation);
  // Pure pursuit controls the wheel axle, not the sensor that sweeps a
  // 1.3 m arc around it during an in-place turn.
  robot_x_ = odom->pose.pose.position.x - odometry_forward_offset_ * std::cos(robot_yaw_);
  robot_y_ = odom->pose.pose.position.y - odometry_forward_offset_ * std::sin(robot_yaw_);
  have_odom_ = true;
}

void ControlNode::onTimer()
{
  if (!have_odom_) {
    return;
  }

  // computeCommand returns all zeros when there's nothing to follow, so the
  // robot is told to stop rather than left coasting on its last command.
  geometry_msgs::msg::Twist cmd = control_.computeCommand(robot_x_, robot_y_, robot_yaw_);

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
