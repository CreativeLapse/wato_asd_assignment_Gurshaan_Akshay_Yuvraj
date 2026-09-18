#include <chrono>
#include <cmath>
#include <memory>

#include "tf2/exceptions.h"

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
  : Node("map_memory"),
    map_memory_(robot::MapMemoryCore(this->get_logger())),
    publish_period_ms_(0),
    update_distance_(0.0),
    update_yaw_(0.0),
    update_period_(0.0),
    max_fuse_yaw_rate_(0.0),
    resolution_(0.0),
    width_(0),
    height_(0),
    origin_x_(0.0),
    origin_y_(0.0),
    have_odom_(false),
    robot_x_(0.0),
    robot_y_(0.0),
    robot_yaw_(0.0),
    yaw_rate_(0.0),
    have_fused_(false),
    last_fuse_x_(0.0),
    last_fuse_y_(0.0),
    last_fuse_yaw_(0.0),
    last_fuse_time_(0, 0, RCL_ROS_TIME)
{
  loadParameters();
  map_memory_.configure(resolution_, width_, height_, origin_x_, origin_y_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    costmap_topic_, 10, std::bind(&MapMemoryNode::onCostmap, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10, std::bind(&MapMemoryNode::onOdom, this, std::placeholders::_1));

  // Latched so a planner or viewer that starts late still gets the map.
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    map_topic_, rclcpp::QoS(1).transient_local());

  publish_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(publish_period_ms_), std::bind(&MapMemoryNode::publishMap, this));

  publishMap();

  RCLCPP_INFO(this->get_logger(), "Map memory node ready; fusing every %.2f m, %.2f rad or %.1f s",
              update_distance_, update_yaw_, update_period_);
}

void MapMemoryNode::loadParameters()
{
  costmap_topic_ = this->declare_parameter<std::string>("local_costmap_topic", "/costmap");
  odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odom/filtered");
  map_topic_ = this->declare_parameter<std::string>("map_topic", "/map");
  map_frame_ = this->declare_parameter<std::string>("map_frame", "sim_world");
  publish_period_ms_ = this->declare_parameter<int>("publish_period_ms", 1000);
  update_distance_ = this->declare_parameter<double>("update_distance", 1.0);
  update_yaw_ = this->declare_parameter<double>("update_yaw", 0.5);
  update_period_ = this->declare_parameter<double>("update_period", 2.0);
  max_fuse_yaw_rate_ = this->declare_parameter<double>("max_fuse_yaw_rate", 0.3);
  resolution_ = this->declare_parameter<double>("global_map.resolution", 0.25);
  width_ = this->declare_parameter<int>("global_map.width", 128);
  height_ = this->declare_parameter<int>("global_map.height", 128);
  origin_x_ = this->declare_parameter<double>("global_map.origin_x", -16.0);
  origin_y_ = this->declare_parameter<double>("global_map.origin_y", -16.0);
}

void MapMemoryNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  robot_x_ = odom->pose.pose.position.x;
  robot_y_ = odom->pose.pose.position.y;
  robot_yaw_ = yawFromQuaternion(odom->pose.pose.orientation);
  yaw_rate_ = odom->twist.twist.angular.z;
  have_odom_ = true;
}

void MapMemoryNode::onCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap)
{
  if (!have_odom_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Costmap received but no odometry yet; cannot place it in the map.");
    return;
  }

  // A scan taken mid-spin is smeared by the sweep itself, so wait it out.
  if (std::abs(yaw_rate_) > max_fuse_yaw_rate_) {
    return;
  }

  const rclcpp::Time now = this->now();
  if (have_fused_) {
    const double travelled = std::hypot(robot_x_ - last_fuse_x_, robot_y_ - last_fuse_y_);
    const double turned = std::abs(std::remainder(robot_yaw_ - last_fuse_yaw_, 2.0 * M_PI));
    const double waited = (now - last_fuse_time_).seconds();
    if (travelled < update_distance_ && turned < update_yaw_ && waited < update_period_) {
      return;
    }
  }

  double x = robot_x_;
  double y = robot_y_;
  double yaw = robot_yaw_;
  poseAtScan(costmap->header, x, y, yaw);

  map_memory_.fuse(*costmap, x, y, yaw);
  have_fused_ = true;
  last_fuse_x_ = robot_x_;
  last_fuse_y_ = robot_y_;
  last_fuse_yaw_ = robot_yaw_;
  last_fuse_time_ = now;

  RCLCPP_DEBUG(this->get_logger(), "Fused costmap into map at (%.2f, %.2f, %.2f rad)", x, y, yaw);
  publishMap();
}

void MapMemoryNode::poseAtScan(
  const std_msgs::msg::Header& header, double& x, double& y, double& yaw) const
{
  try {
    const auto tf = tf_buffer_->lookupTransform(
      map_frame_, header.frame_id, rclcpp::Time(header.stamp), rclcpp::Duration::from_seconds(0.1));
    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    yaw = yawFromQuaternion(tf.transform.rotation);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_DEBUG(this->get_logger(), "No transform at scan time, using latest odom: %s", ex.what());
  }
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
