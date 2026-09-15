#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "control_core.hpp"

// Follows the latest path with pure pursuit, publishing velocity commands
// on a fixed timer.
class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    void loadParameters();
    void onPath(const nav_msgs::msg::Path::SharedPtr path);
    void onOdom(const nav_msgs::msg::Odometry::SharedPtr odom);
    void onTimer();

    static double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q);

    robot::ControlCore control_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string path_topic_;
    std::string odom_topic_;
    std::string cmd_vel_topic_;
    int control_period_ms_;
    double lookahead_distance_;
    double linear_speed_;
    double max_angular_speed_;
    double goal_tolerance_;
    double turn_in_place_angle_;
    double slowdown_distance_;

    bool have_odom_;
    double robot_x_;
    double robot_y_;
    double robot_yaw_;
    bool announced_arrival_;
};

#endif  // CONTROL_NODE_HPP_
