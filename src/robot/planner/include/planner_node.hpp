#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "planner_core.hpp"

// Plans a path to the latest goal and keeps checking for a better one as
// the map fills in and the robot moves, until it arrives or the goal times
// out. The robot's position is taken at the wheel axle, which is what the
// controller steers.
class PlannerNode : public rclcpp::Node {
  public:
    explicit PlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  private:
    enum class State {
      WAITING_FOR_GOAL,
      WAITING_FOR_ROBOT_TO_REACH_GOAL
    };

    void loadParameters();
    void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
    void onGoal(const geometry_msgs::msg::PointStamped::SharedPtr goal);
    void onOdom(const nav_msgs::msg::Odometry::SharedPtr odom);
    void onTimer();

    // Plans to the current goal. Publishes only if the robot has no path,
    // the old one is blocked, or the new one is strictly cheaper. On
    // failure the robot is stopped but the goal is kept for another try.
    void replan();
    // Drops the current goal and tells the controller to stop.
    void finishGoal(const char* reason);
    void publishEmptyPath();

    robot::PlannerCore planner_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string map_topic_;
    std::string goal_topic_;
    std::string odom_topic_;
    std::string path_topic_;
    double goal_tolerance_;
    double plan_timeout_;
    double odometry_forward_offset_;
    int lethal_cost_;
    int unknown_cost_;
    double cost_weight_;
    int snap_radius_;
    int escape_radius_;

    State state_;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;

    // Axle position, in the map frame.
    bool have_odom_;
    double robot_x_;
    double robot_y_;

    // The goal as requested, and where the planner actually sent the robot
    // (they differ when the requested point was inside an obstacle).
    double goal_x_;
    double goal_y_;
    double planned_goal_x_;
    double planned_goal_y_;
    rclcpp::Time goal_start_time_;

    // The path the controller is currently following, if any.
    bool have_path_;
    nav_msgs::msg::Path path_;
};

#endif  // PLANNER_NODE_HPP_
