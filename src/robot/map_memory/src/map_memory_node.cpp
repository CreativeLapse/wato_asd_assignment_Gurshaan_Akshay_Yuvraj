#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include "map_memory_node.hpp"

namespace
{
constexpr size_t kOdomHistory = 100;
}  // namespace

MapMemoryNode::MapMemoryNode(const rclcpp::NodeOptions& options)
  : Node("map_memory", options),
    map_memory_(robot::MapMemoryCore(this->get_logger())),
    publish_period_ms_(0),
    update_distance_(0.0),
    update_yaw_(0.0),
    update_period_(0.0),
    resolution_(0.0),
    width_(0),
    height_(0),
    origin_x_(0.0),
    origin_y_(0.0),
    lethal_radius_(0.0),
    inflation_radius_(0.0),
    decay_(0.0),
    have_fused_(false),
    last_fuse_x_(0.0),
    last_fuse_y_(0.0),
    last_fuse_yaw_(0.0),
    last_fuse_stamp_(0)
{
  loadParameters();
  map_memory_.configure(
    resolution_, width_, height_, origin_x_, origin_y_, lethal_radius_, inflation_radius_, decay_);

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
  costmap_topic_ = this->declare_parameter<std::string>("local_costmap_topic", "/local_obstacles");
  odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odom/filtered");
  map_topic_ = this->declare_parameter<std::string>("map_topic", "/map");
  map_frame_ = this->declare_parameter<std::string>("map_frame", "sim_world");
  publish_period_ms_ = this->declare_parameter<int>("publish_period_ms", 1000);
  update_distance_ = this->declare_parameter<double>("update_distance", 1.0);
  update_yaw_ = this->declare_parameter<double>("update_yaw", 0.3);
  update_period_ = this->declare_parameter<double>("update_period", 1.0);
  resolution_ = this->declare_parameter<double>("global_map.resolution", 0.25);
  width_ = this->declare_parameter<int>("global_map.width", 128);
  height_ = this->declare_parameter<int>("global_map.height", 128);
  origin_x_ = this->declare_parameter<double>("global_map.origin_x", -16.0);
  origin_y_ = this->declare_parameter<double>("global_map.origin_y", -16.0);
  lethal_radius_ = this->declare_parameter<double>("global_map.lethal_radius", 1.7);
  inflation_radius_ = this->declare_parameter<double>("global_map.inflation_radius", 3.2);
  decay_ = this->declare_parameter<double>("global_map.decay", 1.5);
}

void MapMemoryNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  if (odom->header.frame_id != map_frame_) {
    return;
  }

  const int64_t stamp = rclcpp::Time(odom->header.stamp).nanoseconds();
  if (!odom_history_.empty()) {
    const int64_t latest = rclcpp::Time(odom_history_.back().header.stamp).nanoseconds();
    if (stamp == latest) {
      return;
    }
    // Time went backwards: the simulation was reset, so nothing we hold
    // can be trusted any more.
    if (stamp < latest) {
      odom_history_.clear();
      pending_.reset();
      have_fused_ = false;
    }
  }

  odom_history_.push_back(*odom);
  while (odom_history_.size() > kOdomHistory) {
    odom_history_.pop_front();
  }
  fusePending();
}

void MapMemoryNode::onCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap)
{
  fusePending();
  // A scan already waiting for its odometry keeps its place; replacing it
  // every time would starve fusion whenever odometry lags.
  if (!pending_) {
    pending_ = costmap;
    fusePending();
  }
}

void MapMemoryNode::fusePending()
{
  if (!pending_) {
    return;
  }
  if (odom_history_.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Scan received but no odometry yet; cannot place it in the map.");
    return;
  }
  if (pending_->header.frame_id != odom_history_.back().child_frame_id) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Scan frame %s does not match odometry child frame %s; dropping it.",
                         pending_->header.frame_id.c_str(),
                         odom_history_.back().child_frame_id.c_str());
    pending_.reset();
    return;
  }

  const int64_t stamp = rclcpp::Time(pending_->header.stamp).nanoseconds();
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  if (!poseAt(stamp, x, y, yaw)) {
    // Older than anything we remember: give up on it. Otherwise the next
    // odometry sample will bracket it.
    if (stamp < rclcpp::Time(odom_history_.front().header.stamp).nanoseconds()) {
      pending_.reset();
    }
    return;
  }

  if (have_fused_) {
    const double travelled = std::hypot(x - last_fuse_x_, y - last_fuse_y_);
    const double turned = std::abs(std::remainder(yaw - last_fuse_yaw_, 2.0 * M_PI));
    const double waited = (stamp - last_fuse_stamp_) * 1e-9;
    if (travelled < update_distance_ && turned < update_yaw_ && waited < update_period_) {
      pending_.reset();
      return;
    }
  }

  map_memory_.fuse(*pending_, x, y, yaw);
  pending_.reset();
  have_fused_ = true;
  last_fuse_x_ = x;
  last_fuse_y_ = y;
  last_fuse_yaw_ = yaw;
  last_fuse_stamp_ = stamp;

  RCLCPP_DEBUG(this->get_logger(), "Fused scan into map at (%.2f, %.2f, %.2f rad)", x, y, yaw);
  publishMap();
}

bool MapMemoryNode::poseAt(int64_t stamp, double& x, double& y, double& yaw) const
{
  const auto after = std::lower_bound(
    odom_history_.begin(), odom_history_.end(), stamp,
    [](const nav_msgs::msg::Odometry& odom, int64_t t) {
      return rclcpp::Time(odom.header.stamp).nanoseconds() < t;
    });
  if (after == odom_history_.end()) {
    return false;
  }
  const auto before = (after == odom_history_.begin()) ? after : std::prev(after);

  const int64_t t0 = rclcpp::Time(before->header.stamp).nanoseconds();
  const int64_t t1 = rclcpp::Time(after->header.stamp).nanoseconds();
  const double f = (t1 == t0) ? 0.0 : static_cast<double>(stamp - t0) / (t1 - t0);

  const auto& p0 = before->pose.pose.position;
  const auto& p1 = after->pose.pose.position;
  x = p0.x + f * (p1.x - p0.x);
  y = p0.y + f * (p1.y - p0.y);

  const double yaw0 = yawFromQuaternion(before->pose.pose.orientation);
  const double yaw1 = yawFromQuaternion(after->pose.pose.orientation);
  yaw = yaw0 + f * std::remainder(yaw1 - yaw0, 2.0 * M_PI);
  return true;
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
