#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "map_memory_core.hpp"

// Fuses local costmaps into a global map whenever the robot has moved,
// turned or waited long enough, and republishes that map on a timer.
class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    void loadParameters();
    void onCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap);
    void onOdom(const nav_msgs::msg::Odometry::SharedPtr odom);
    void publishMap();

    // Where the costmap's frame was when the scan was taken. Falls back to
    // the latest odometry if TF can't answer.
    void poseAtScan(const std_msgs::msg::Header& header, double& x, double& y, double& yaw) const;

    static double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q);

    robot::MapMemoryCore map_memory_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string costmap_topic_;
    std::string odom_topic_;
    std::string map_topic_;
    std::string map_frame_;
    int publish_period_ms_;
    double update_distance_;
    double update_yaw_;
    double update_period_;
    double max_fuse_yaw_rate_;
    double resolution_;
    int width_;
    int height_;
    double origin_x_;
    double origin_y_;

    // Latest robot pose and turn rate, in the map frame.
    bool have_odom_;
    double robot_x_;
    double robot_y_;
    double robot_yaw_;
    double yaw_rate_;

    // Where and when the last costmap was fused.
    bool have_fused_;
    double last_fuse_x_;
    double last_fuse_y_;
    double last_fuse_yaw_;
    rclcpp::Time last_fuse_time_;
};

#endif  // MAP_MEMORY_NODE_HPP_
