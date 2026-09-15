#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot
{

// Pure pursuit path follower for a differential-drive robot.
//
// Each cycle it picks a point on the path about `lookahead_distance` ahead of
// the robot and steers along the circular arc that passes through it. If the
// point is far off to the side the robot turns in place first.
class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit ControlCore(const rclcpp::Logger& logger);

    void configure(
      double lookahead_distance,
      double linear_speed,
      double max_angular_speed,
      double goal_tolerance,
      double turn_in_place_angle,
      double slowdown_distance);

    void setPath(const nav_msgs::msg::Path& path);
    void clearPath();
    bool hasPath() const { return !path_.empty(); }

    // True when the robot is within goal_tolerance of the path's last point.
    bool goalReached(double robot_x, double robot_y) const;

    // The path point the robot should currently steer toward.
    // Returns false when there is no path.
    bool findLookahead(double robot_x, double robot_y, geometry_msgs::msg::Point& out) const;

    // Velocity command for the robot at the given pose. Zero when there is
    // no path or the goal has been reached.
    geometry_msgs::msg::Twist computeCommand(double robot_x, double robot_y, double robot_yaw) const;

  private:
    std::vector<geometry_msgs::msg::Point> path_;
    rclcpp::Logger logger_;

    double lookahead_distance_;
    double linear_speed_;
    double max_angular_speed_;
    double goal_tolerance_;
    double turn_in_place_angle_;
    double slowdown_distance_;
};

}  // namespace robot

#endif  // CONTROL_CORE_HPP_
