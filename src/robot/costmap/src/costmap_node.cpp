#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode()
  : Node("costmap"),
    costmap_(robot::CostmapCore(this->get_logger())),
    resolution_(0.0),
    width_(0),
    height_(0),
    inflation_radius_(0.0)
{
  loadParameters();
  costmap_.configure(resolution_, width_, height_, inflation_radius_);

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, 10, std::bind(&CostmapNode::onScan, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(costmap_topic_, 10);

  RCLCPP_INFO(this->get_logger(), "Costmap node listening on %s, publishing %s",
              scan_topic_.c_str(), costmap_topic_.c_str());
}

void CostmapNode::loadParameters()
{
  scan_topic_ = this->declare_parameter<std::string>("laserscan_topic", "/lidar");
  costmap_topic_ = this->declare_parameter<std::string>("costmap_topic", "/costmap");
  resolution_ = this->declare_parameter<double>("costmap.resolution", 0.4);
  width_ = this->declare_parameter<int>("costmap.width", 120);
  height_ = this->declare_parameter<int>("costmap.height", 120);
  inflation_radius_ = this->declare_parameter<double>("costmap.inflation_radius", 1.0);
}

void CostmapNode::onScan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  costmap_.processScan(*scan);

  // The grid is expressed in the laser's frame, so it borrows the scan header.
  nav_msgs::msg::OccupancyGrid msg = costmap_.grid();
  msg.header = scan->header;
  costmap_pub_->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
