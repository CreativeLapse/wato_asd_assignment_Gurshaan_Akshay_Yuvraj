#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "costmap_core.hpp"

// Subscribes to the laser scan and publishes a local costmap for every scan.
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();

  private:
    void loadParameters();
    void onScan(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    robot::CostmapCore costmap_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr obstacles_pub_;

    std::string scan_topic_;
    std::string costmap_topic_;
    std::string obstacles_topic_;
    double resolution_;
    int width_;
    int height_;
    double inflation_radius_;
};

#endif  // COSTMAP_NODE_HPP_
