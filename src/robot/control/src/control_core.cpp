#include <algorithm>
#include <cmath>
#include <limits>

#include "control_core.hpp"

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger)
  : logger_(logger), speed_(0.0), turning_(false) {}

void ControlCore::configure(const ControlParams& params)
{
  params_ = params;
}

void ControlCore::setPath(const nav_msgs::msg::Path& path)
{
  path_.clear();
  path_.reserve(path.poses.size());
  for (const auto& pose : path.poses) {
    path_.push_back(pose.pose.position);
  }
  reset();
}

void ControlCore::clearPath()
{
  path_.clear();
  reset();
}

void ControlCore::reset()
{
  speed_ = 0.0;
  turning_ = false;
}

bool ControlCore::goalReached(double robot_x, double robot_y) const
{
  if (path_.empty()) {
    return true;
  }
  const auto& goal = path_.back();
  return std::hypot(goal.x - robot_x, goal.y - robot_y) <= params_.goal_tolerance;
}

bool ControlCore::findLookahead(
  double robot_x, double robot_y, double lookahead, geometry_msgs::msg::Point& out) const
{
  if (path_.empty()) {
    return false;
  }

  size_t nearest = 0;
  double nearest_dist = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < path_.size(); ++i) {
    const double d = std::hypot(path_[i].x - robot_x, path_[i].y - robot_y);
    if (d < nearest_dist) {
      nearest_dist = d;
      nearest = i;
    }
  }

  for (size_t i = nearest; i < path_.size(); ++i) {
    if (std::hypot(path_[i].x - robot_x, path_[i].y - robot_y) >= lookahead) {
      out = path_[i];
      return true;
    }
  }

  out = path_.back();
  return true;
}

geometry_msgs::msg::Twist ControlCore::computeCommand(
  double robot_x, double robot_y, double robot_yaw, double dt)
{
  geometry_msgs::msg::Twist cmd;

  if (path_.empty() || goalReached(robot_x, robot_y)) {
    reset();
    return cmd;
  }

  const double lookahead = std::clamp(
    params_.lookahead_gain * speed_, params_.lookahead_min, params_.lookahead_max);
  geometry_msgs::msg::Point target;
  findLookahead(robot_x, robot_y, lookahead, target);

  // Express the target in the robot's frame: x forward, y to the left.
  const double dx = target.x - robot_x;
  const double dy = target.y - robot_y;
  const double forward = std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
  const double left = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;
  const double heading_error = std::atan2(left, forward);

  const double turn_threshold = turning_ ? params_.turn_exit_angle : params_.turn_enter_angle;
  if (std::abs(heading_error) > turn_threshold) {
    turning_ = true;
    speed_ = 0.0;
    cmd.angular.z = std::clamp(
      params_.turn_gain * heading_error, -params_.max_angular_speed, params_.max_angular_speed);
    return cmd;
  }
  turning_ = false;

  // Pure pursuit: the arc through the robot and the target has curvature
  // 2 * y / L^2, and the angular rate that follows it is v * curvature.
  const double dist_sq = forward * forward + left * left;
  const double curvature = (dist_sq > 1e-9) ? (2.0 * left / dist_sq) : 0.0;

  double target_speed = params_.max_speed / (1.0 + params_.curvature_gain * std::abs(curvature));
  target_speed *= std::max(std::cos(heading_error), 0.0);

  const auto& goal = path_.back();
  const double to_goal = std::hypot(goal.x - robot_x, goal.y - robot_y);
  if (params_.slowdown_distance > 0.0 && to_goal < params_.slowdown_distance) {
    const double approach = params_.max_speed * to_goal / params_.slowdown_distance;
    target_speed = std::min(target_speed, std::max(approach, params_.min_speed));
  }

  if (target_speed > speed_) {
    speed_ = std::min(target_speed, speed_ + params_.accel * dt);
  } else {
    speed_ = std::max(target_speed, speed_ - params_.decel * dt);
  }

  cmd.linear.x = speed_;
  cmd.angular.z = std::clamp(speed_ * curvature, -params_.max_angular_speed, params_.max_angular_speed);
  return cmd;
}

}  // namespace robot
