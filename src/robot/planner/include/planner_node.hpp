#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "planner_core.hpp"

// Plans a path to the latest goal and keeps replanning as the map grows,
// until the robot arrives or the goal times out.
class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

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

    // Plans to the current goal and publishes the result.
    void replan();
    // Drops the current goal and tells the controller to stop.
    void finishGoal(const char* reason);

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
    int lethal_cost_;
    int unknown_cost_;
    double cost_weight_;
    int snap_radius_;

    State state_;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;

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
};

#endif  // PLANNER_NODE_HPP_
