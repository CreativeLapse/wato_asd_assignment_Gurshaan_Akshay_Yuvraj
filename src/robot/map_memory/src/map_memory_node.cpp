#include <chrono>
#include <cmath>
#include <memory>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
  : Node("map_memory"),
    map_memory_(robot::MapMemoryCore(this->get_logger())),
    publish_period_ms_(0),
    update_distance_(0.0),
    resolution_(0.0),
    width_(0),
    height_(0),
    origin_x_(0.0),
    origin_y_(0.0),
    have_odom_(false),
    robot_x_(0.0),
    robot_y_(0.0),
    robot_yaw_(0.0),
    have_fused_(false),
    last_fuse_x_(0.0),
    last_fuse_y_(0.0)
{
  loadParameters();
  map_memory_.configure(resolution_, width_, height_, origin_x_, origin_y_);

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    costmap_topic_, 10, std::bind(&MapMemoryNode::onCostmap, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10, std::bind(&MapMemoryNode::onOdom, this, std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_, 10);

  publish_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(publish_period_ms_), std::bind(&MapMemoryNode::publishMap, this));

  // Send the (empty) map right away so downstream nodes know the map's shape
  // before the robot has moved anywhere.
  publishMap();

  RCLCPP_INFO(this->get_logger(), "Map memory node ready; fusing every %.2f m of travel",
              update_distance_);
}

void MapMemoryNode::loadParameters()
{
  costmap_topic_ = this->declare_parameter<std::string>("local_costmap_topic", "/costmap");
  odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odom/filtered");
  map_topic_ = this->declare_parameter<std::string>("map_topic", "/map");
  map_frame_ = this->declare_parameter<std::string>("map_frame", "sim_world");
  publish_period_ms_ = this->declare_parameter<int>("publish_period_ms", 1000);
  update_distance_ = this->declare_parameter<double>("update_distance", 1.5);
  resolution_ = this->declare_parameter<double>("global_map.resolution", 0.5);
  width_ = this->declare_parameter<int>("global_map.width", 60);
  height_ = this->declare_parameter<int>("global_map.height", 60);
  origin_x_ = this->declare_parameter<double>("global_map.origin_x", -15.0);
  origin_y_ = this->declare_parameter<double>("global_map.origin_y", -15.0);
}

void MapMemoryNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  robot_x_ = odom->pose.pose.position.x;
  robot_y_ = odom->pose.pose.position.y;
  robot_yaw_ = yawFromQuaternion(odom->pose.pose.orientation);
  have_odom_ = true;
}

void MapMemoryNode::onCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap)
{
  if (!have_odom_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Costmap received but no odometry yet; cannot place it in the map.");
    return;
  }

  // Only fuse once the robot has moved far enough for the new costmap to
  // add something the old one didn't. The very first costmap always counts.
  if (have_fused_) {
    const double travelled = std::hypot(robot_x_ - last_fuse_x_, robot_y_ - last_fuse_y_);
    if (travelled < update_distance_) {
      return;
    }
  }

  map_memory_.fuse(*costmap, robot_x_, robot_y_, robot_yaw_);
  have_fused_ = true;
  last_fuse_x_ = robot_x_;
  last_fuse_y_ = robot_y_;

  RCLCPP_INFO(this->get_logger(), "Fused costmap into map at (%.2f, %.2f)", robot_x_, robot_y_);
  publishMap();
}

void MapMemoryNode::publishMap()
{
  nav_msgs::msg::OccupancyGrid msg = map_memory_.map();
  msg.header.stamp = this->now();
  msg.header.frame_id = map_frame_;
  map_pub_->publish(msg);
}

double MapMemoryNode::yawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
