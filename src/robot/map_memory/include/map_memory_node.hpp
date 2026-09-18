#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include <cstdint>
#include <deque>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "map_memory_core.hpp"

// Fuses local observations into a global map whenever the robot has moved,
// turned or waited long enough, and republishes that map on a timer.
//
// Scans and odometry arrive on their own schedules, so a scan waits until
// the odometry sample after it has arrived, then gets placed at the pose
// interpolated to its own timestamp.
class MapMemoryNode : public rclcpp::Node {
  public:
    explicit MapMemoryNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  private:
    void loadParameters();
    void onCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap);
    void onOdom(const nav_msgs::msg::Odometry::SharedPtr odom);
    void fusePending();
    void publishMap();

    // Interpolates the odometry history to `stamp`. False if the history
    // doesn't bracket it yet.
    bool poseAt(int64_t stamp, double& x, double& y, double& yaw) const;

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
    double update_yaw_;
    double update_period_;
    double resolution_;
    int width_;
    int height_;
    double origin_x_;
    double origin_y_;
    double lethal_radius_;
    double inflation_radius_;
    double decay_;

    std::deque<nav_msgs::msg::Odometry> odom_history_;
    nav_msgs::msg::OccupancyGrid::SharedPtr pending_;

    // Where and when (sensor time) the last scan was fused.
    bool have_fused_;
    double last_fuse_x_;
    double last_fuse_y_;
    double last_fuse_yaw_;
    int64_t last_fuse_stamp_;
};

#endif  // MAP_MEMORY_NODE_HPP_
