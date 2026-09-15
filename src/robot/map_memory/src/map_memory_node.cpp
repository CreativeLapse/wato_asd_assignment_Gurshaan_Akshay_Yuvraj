#include <chrono>
#include <algorithm>
#include <cmath>
#include <memory>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode(const rclcpp::NodeOptions& options)
  : Node("map_memory", options),
    map_memory_(robot::MapMemoryCore(this->get_logger())),
    publish_period_ms_(0),
    update_distance_(0.0),
    update_angle_(0.0),
    max_update_interval_(0.0),
    resolution_(0.0),
    inflation_radius_(0.0),
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
  map_memory_.configure(resolution_, width_, height_, origin_x_, origin_y_, inflation_radius_);

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
  costmap_topic_ = this->declare_parameter<std::string>("local_costmap_topic", "/local_obstacles");
  odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odom/filtered");
  map_topic_ = this->declare_parameter<std::string>("map_topic", "/map");
  map_frame_ = this->declare_parameter<std::string>("map_frame", "sim_world");
  publish_period_ms_ = this->declare_parameter<int>("publish_period_ms", 1000);
  update_distance_ = this->declare_parameter<double>("update_distance", 1.5);
  update_angle_ = this->declare_parameter<double>("update_angle", 0.2);
  max_update_interval_ = this->declare_parameter<double>("max_update_interval", 0.5);
  resolution_ = this->declare_parameter<double>("global_map.resolution", 0.5);
  inflation_radius_ = this->declare_parameter<double>("global_map.inflation_radius", 3.4);
  width_ = this->declare_parameter<int>("global_map.width", 60);
  height_ = this->declare_parameter<int>("global_map.height", 60);
  origin_x_ = this->declare_parameter<double>("global_map.origin_x", -15.0);
  origin_y_ = this->declare_parameter<double>("global_map.origin_y", -15.0);
}

void MapMemoryNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  if (odom->header.frame_id != map_frame_) {
    return;
  }
  const auto stamp = rclcpp::Time(odom->header.stamp).nanoseconds();
  if (!odom_history_.empty()) {
    const auto latest = rclcpp::Time(odom_history_.back().header.stamp).nanoseconds();
    if (stamp < latest) {
      odom_history_.clear();
      pending_costmap_.reset();
      have_fused_ = false;
    } else if (stamp == latest) {
      return;
    }
  }
  odom_history_.push_back(*odom);
  while (odom_history_.size() > 100) {
    odom_history_.pop_front();
  }
  have_odom_ = true;
  tryFusePendingCostmap();
}

void MapMemoryNode::onCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap)
{
  tryFusePendingCostmap();
  // Keep the scan already waiting for its matching odometry. Replacing it
  // on every callback can starve fusion when odometry consistently lags.
  if (!pending_costmap_) {
    pending_costmap_ = costmap;
    tryFusePendingCostmap();
  }
}

void MapMemoryNode::tryFusePendingCostmap()
{
  if (!pending_costmap_) {
    return;
  }
  if (!have_odom_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Costmap received but no odometry yet; cannot place it in the map.");
    return;
  }

  const auto stamp = rclcpp::Time(pending_costmap_->header.stamp).nanoseconds();
  if (stamp < rclcpp::Time(odom_history_.front().header.stamp).nanoseconds()) {
    pending_costmap_.reset();
    return;
  }
  // Wait for the odometry sample after this scan, then interpolate the
  // sensor pose at acquisition time instead of using a later turn angle.
  const auto after = std::lower_bound(odom_history_.begin(), odom_history_.end(), stamp,
    [](const auto& odom, int64_t time) {
      return rclcpp::Time(odom.header.stamp).nanoseconds() < time;
    });
  if (after == odom_history_.end()) {
    return;
  }
  const auto before = after == odom_history_.begin() ? after : std::prev(after);
  if (after->child_frame_id != pending_costmap_->header.frame_id ||
      before->child_frame_id != pending_costmap_->header.frame_id) {
    pending_costmap_.reset();
    return;
  }
  const auto t0 = rclcpp::Time(before->header.stamp).nanoseconds();
  const auto t1 = rclcpp::Time(after->header.stamp).nanoseconds();
  const double fraction = t1 == t0 ? 0.0 : static_cast<double>(stamp - t0) / (t1 - t0);
  const auto& p0 = before->pose.pose.position;
  const auto& p1 = after->pose.pose.position;
  robot_x_ = p0.x + fraction * (p1.x - p0.x);
  robot_y_ = p0.y + fraction * (p1.y - p0.y);
  const double yaw0 = yawFromQuaternion(before->pose.pose.orientation);
  const double yaw1 = yawFromQuaternion(after->pose.pose.orientation);
  const double yaw_delta = std::atan2(std::sin(yaw1 - yaw0), std::cos(yaw1 - yaw0));
  robot_yaw_ = yaw0 + fraction * yaw_delta;

  // Refresh on motion, rotation, or elapsed sensor time. A stopped robot
  // must still be able to discover an opening and resume its saved goal.
  if (have_fused_) {
    const double travelled = std::hypot(robot_x_ - last_fuse_x_, robot_y_ - last_fuse_y_);
    const double turned = std::abs(std::atan2(std::sin(robot_yaw_ - last_fuse_yaw_),
                                              std::cos(robot_yaw_ - last_fuse_yaw_)));
    const double elapsed = (stamp - last_fuse_stamp_) * 1e-9;
    if (travelled < update_distance_ && turned < update_angle_ && elapsed < max_update_interval_) {
      pending_costmap_.reset();
      return;
    }
  }

  map_memory_.fuse(*pending_costmap_, robot_x_, robot_y_, robot_yaw_);
  pending_costmap_.reset();
  have_fused_ = true;
  last_fuse_x_ = robot_x_;
  last_fuse_y_ = robot_y_;
  last_fuse_yaw_ = robot_yaw_;
  last_fuse_stamp_ = stamp;

  RCLCPP_DEBUG(this->get_logger(), "Fused costmap into map at (%.2f, %.2f)", robot_x_, robot_y_);
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

#ifndef WATO_NODE_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
#endif
