#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot
{

struct ControlParams
{
  double max_speed = 0.8;          // m/s
  double min_speed = 0.15;         // m/s, floor while creeping up to the goal
  double max_angular_speed = 1.0;  // rad/s
  double accel = 0.5;              // m/s^2, how fast the speed may rise
  double decel = 1.0;              // m/s^2, how fast it may fall
  double lookahead_min = 1.2;      // m
  double lookahead_max = 2.5;      // m
  double lookahead_gain = 2.0;     // s, lookahead = gain * speed, clamped
  double curvature_gain = 2.0;     // slows the robot as the arc tightens
  double turn_enter_angle = 1.0;   // rad, start spinning in place above this heading error
  double turn_exit_angle = 0.3;    // rad, stop spinning once below this
  double turn_gain = 1.5;          // rad/s per rad while spinning
  double slowdown_distance = 1.5;  // m, ease off this far from the goal
  double goal_tolerance = 0.3;     // m
};

// Pure pursuit path follower for a differential-drive robot.
//
// Each cycle it picks a point on the path a lookahead ahead of the robot
// and steers along the circular arc that passes through it. The lookahead
// grows with speed, the speed drops for tight arcs and near the goal, and
// both are ramped so the robot never lurches. If the target is far off to
// the side the robot spins in place first, with hysteresis so it doesn't
// flip between spinning and driving at the boundary.
class ControlCore {
  public:
    explicit ControlCore(const rclcpp::Logger& logger);

    void configure(const ControlParams& params);

    void setPath(const nav_msgs::msg::Path& path);
    void clearPath();
    bool hasPath() const { return !path_.empty(); }

    bool goalReached(double robot_x, double robot_y) const;

    // The first path point at least `lookahead` from the robot, searching
    // forward from the point nearest the robot. Returns false with no path.
    bool findLookahead(double robot_x, double robot_y, double lookahead, geometry_msgs::msg::Point& out) const;

    // Velocity command for the robot at the given pose, `dt` seconds after
    // the previous call. Zero when there is no path or the goal is reached.
    geometry_msgs::msg::Twist computeCommand(double robot_x, double robot_y, double robot_yaw, double dt);

  private:
    void reset();

    std::vector<geometry_msgs::msg::Point> path_;
    rclcpp::Logger logger_;
    ControlParams params_;

    double speed_;
    bool turning_;
};

}  // namespace robot

#endif  // CONTROL_CORE_HPP_
