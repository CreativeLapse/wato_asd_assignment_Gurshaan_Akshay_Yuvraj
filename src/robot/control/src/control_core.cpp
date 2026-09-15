#include <algorithm>
#include <cmath>
#include <limits>

#include "control_core.hpp"

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger)
  : logger_(logger),
    lookahead_distance_(1.0),
    linear_speed_(0.8),
    max_angular_speed_(1.0),
    goal_tolerance_(0.2),
    turn_in_place_angle_(0.8),
    slowdown_distance_(1.6) {}

void ControlCore::configure(
  double lookahead_distance,
  double linear_speed,
  double max_angular_speed,
  double goal_tolerance,
  double turn_in_place_angle,
  double slowdown_distance)
{
  lookahead_distance_ = lookahead_distance;
  linear_speed_ = linear_speed;
  max_angular_speed_ = max_angular_speed;
  goal_tolerance_ = goal_tolerance;
  turn_in_place_angle_ = turn_in_place_angle;
  slowdown_distance_ = slowdown_distance;
}

void ControlCore::setPath(const nav_msgs::msg::Path& path)
{
  path_.clear();
  path_.reserve(path.poses.size());
  for (const auto& pose : path.poses) {
    path_.push_back(pose.pose.position);
  }
}

void ControlCore::clearPath()
{
  path_.clear();
}

bool ControlCore::goalReached(double robot_x, double robot_y) const
{
  if (path_.empty()) {
    return true;
  }
  const auto& goal = path_.back();
  return std::hypot(goal.x - robot_x, goal.y - robot_y) <= goal_tolerance_;
}

bool ControlCore::findLookahead(double robot_x, double robot_y, geometry_msgs::msg::Point& out) const
{
  if (path_.empty()) {
    return false;
  }

  // Start from the point nearest the robot so we never chase a point we
  // have already passed, then walk forward until we're a lookahead away.
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
    if (std::hypot(path_[i].x - robot_x, path_[i].y - robot_y) >= lookahead_distance_) {
      out = path_[i];
      return true;
    }
  }

  // Everything left is closer than the lookahead: aim for the end.
  out = path_.back();
  return true;
}

geometry_msgs::msg::Twist ControlCore::computeCommand(double robot_x, double robot_y, double robot_yaw) const
{
  geometry_msgs::msg::Twist cmd;

  geometry_msgs::msg::Point target;
  if (!findLookahead(robot_x, robot_y, target) || goalReached(robot_x, robot_y)) {
    return cmd;
  }

  // Express the target in the robot's frame: x forward, y to the left.
  const double dx = target.x - robot_x;
  const double dy = target.y - robot_y;
  const double forward = std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
  const double left = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;
  const double heading_error = std::atan2(left, forward);

  // Facing the wrong way: spin toward the target before driving.
  if (std::abs(heading_error) > turn_in_place_angle_) {
    cmd.angular.z = std::clamp(heading_error, -max_angular_speed_, max_angular_speed_);
    return cmd;
  }

  // Pure pursuit: the arc through the robot and the target has curvature
  // 2 * y / L^2, and the angular rate that follows it is v * curvature.
  const double dist_sq = forward * forward + left * left;
  const double curvature = (dist_sq > 1e-9) ? (2.0 * left / dist_sq) : 0.0;

  // Ease off as the end of the path comes up so we stop on it, not past it.
  const auto& goal = path_.back();
  const double to_goal = std::hypot(goal.x - robot_x, goal.y - robot_y);
  const double speed_scale = (slowdown_distance_ > 0.0)
    ? std::clamp(to_goal / slowdown_distance_, 0.25, 1.0)
    : 1.0;

  cmd.linear.x = linear_speed_ * speed_scale;
  // Reduce forward speed when a curve needs more than the allowed turning
  // rate. Clamping angular speed alone would widen the arc into obstacles.
  if (std::abs(curvature) > 1e-9) {
    cmd.linear.x = std::min(cmd.linear.x, max_angular_speed_ / std::abs(curvature));
  }
  cmd.angular.z = std::clamp(cmd.linear.x * curvature, -max_angular_speed_, max_angular_speed_);
  return cmd;
}

}  // namespace robot
