#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include <string>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "map_memory_core.hpp"

// Fuses local costmaps into a global map whenever the robot has travelled
// far enough, and republishes that map on a timer.
class MapMemoryNode : public rclcpp::Node {
  public:
    explicit MapMemoryNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  private:
    void loadParameters();
    void onCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap);
    void onOdom(const nav_msgs::msg::Odometry::SharedPtr odom);
    void publishMap();
    void tryFusePendingCostmap();

    static double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q);

    robot::MapMemoryCore map_memory_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    std::string costmap_topic_;
    std::string odom_topic_;
    std::string map_topic_;
    std::string map_frame_;
    int publish_period_ms_;
    double update_distance_;
    double update_angle_;
    double max_update_interval_;
    double resolution_;
    double inflation_radius_;
    int width_;
    int height_;
    double origin_x_;
    double origin_y_;

    // Latest robot pose, in the map frame.
    bool have_odom_;
    double robot_x_;
    double robot_y_;
    double robot_yaw_;

    // Where the robot was the last time a costmap was fused.
    bool have_fused_;
    double last_fuse_x_;
    double last_fuse_y_;
    double last_fuse_yaw_ = 0.0;
    int64_t last_fuse_stamp_ = 0;
    std::deque<nav_msgs::msg::Odometry> odom_history_;
    nav_msgs::msg::OccupancyGrid::SharedPtr pending_costmap_;
};

#endif  // MAP_MEMORY_NODE_HPP_
