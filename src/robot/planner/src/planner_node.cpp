#include <chrono>
#include <cmath>
#include <memory>

#include "planner_node.hpp"

PlannerNode::PlannerNode(const rclcpp::NodeOptions& options)
  : Node("planner", options),
    planner_(robot::PlannerCore(this->get_logger())),
    goal_tolerance_(0.0),
    plan_timeout_(0.0),
    odometry_forward_offset_(0.0),
    lethal_cost_(0),
    unknown_cost_(0),
    cost_weight_(0.0),
    snap_radius_(0),
    escape_radius_(0),
    state_(State::WAITING_FOR_GOAL),
    have_odom_(false),
    robot_x_(0.0),
    robot_y_(0.0),
    goal_x_(0.0),
    goal_y_(0.0),
    planned_goal_x_(0.0),
    planned_goal_y_(0.0),
    goal_start_time_(0, 0, RCL_ROS_TIME),
    have_path_(false)
{
  loadParameters();
  planner_.configure(lethal_cost_, unknown_cost_, cost_weight_, snap_radius_, escape_radius_);

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, 10, std::bind(&PlannerNode::onMap, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    goal_topic_, 10, std::bind(&PlannerNode::onGoal, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10, std::bind(&PlannerNode::onOdom, this, std::placeholders::_1));
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_, 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&PlannerNode::onTimer, this));

  RCLCPP_INFO(this->get_logger(), "Planner node ready; waiting for a goal on %s", goal_topic_.c_str());
}

void PlannerNode::loadParameters()
{
  map_topic_ = this->declare_parameter<std::string>("map_topic", "/map");
  goal_topic_ = this->declare_parameter<std::string>("goal_topic", "/goal_point");
  odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odom/filtered");
  path_topic_ = this->declare_parameter<std::string>("path_topic", "/path");
  goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.6);
  plan_timeout_ = this->declare_parameter<double>("plan_timeout_seconds", 240.0);
  odometry_forward_offset_ = this->declare_parameter<double>("odometry_forward_offset", 1.3);
  lethal_cost_ = this->declare_parameter<int>("lethal_cost", 99);
  unknown_cost_ = this->declare_parameter<int>("unknown_cost", 10);
  cost_weight_ = this->declare_parameter<double>("cost_weight", 5.0);
  snap_radius_ = this->declare_parameter<int>("snap_radius", 20);
  escape_radius_ = this->declare_parameter<int>("escape_radius", 8);
}

void PlannerNode::onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
  const bool changed = !map_ || map_->info != map->info || map_->data != map->data;
  map_ = map;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL && (changed || !have_path_)) {
    replan();
  }
}

void PlannerNode::onGoal(const geometry_msgs::msg::PointStamped::SharedPtr goal)
{
  goal_x_ = goal->point.x;
  goal_y_ = goal->point.y;
  planned_goal_x_ = goal_x_;
  planned_goal_y_ = goal_y_;
  goal_start_time_ = this->now();
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  have_path_ = false;

  RCLCPP_INFO(this->get_logger(), "New goal: (%.2f, %.2f)", goal_x_, goal_y_);
  replan();
}

void PlannerNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  // Odometry reports the lidar, which sits ahead of the wheel axle.
  const auto& q = odom->pose.pose.orientation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  robot_x_ = odom->pose.pose.position.x - odometry_forward_offset_ * std::cos(yaw);
  robot_y_ = odom->pose.pose.position.y - odometry_forward_offset_ * std::sin(yaw);
  have_odom_ = true;
}

void PlannerNode::onTimer()
{
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    return;
  }

  if (have_odom_ && have_path_ &&
      std::hypot(robot_x_ - planned_goal_x_, robot_y_ - planned_goal_y_) < goal_tolerance_) {
    finishGoal("Goal reached");
    return;
  }

  const double elapsed = (this->now() - goal_start_time_).seconds();
  if (elapsed > plan_timeout_) {
    finishGoal("Goal timed out");
    return;
  }

  // The robot has moved since the last look, so a shorter route may have
  // opened up even if the map hasn't changed. A failed plan gets retried.
  replan();
}

void PlannerNode::replan()
{
  if (!map_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Have a goal but no map yet; waiting.");
    return;
  }
  if (!have_odom_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Have a goal but no odometry yet; waiting.");
    return;
  }

  nav_msgs::msg::Path path;
  double goal_x = 0.0;
  double goal_y = 0.0;
  const bool ok = planner_.plan(*map_, robot_x_, robot_y_, goal_x_, goal_y_, path, goal_x, goal_y);
  if (!ok) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Planning failed; stopping and keeping the goal for another try.");
    if (have_path_) {
      have_path_ = false;
      publishEmptyPath();
    }
    return;
  }

  // Keep the current path when it's still as good: swapping between equal
  // routes only makes the robot twitch.
  if (have_path_ && goal_x == planned_goal_x_ && goal_y == planned_goal_y_) {
    const double old_cost = planner_.remainingPathCost(*map_, path_, robot_x_, robot_y_);
    const double new_cost = planner_.remainingPathCost(*map_, path, robot_x_, robot_y_);
    if (old_cost <= new_cost + 1e-9) {
      return;
    }
  }

  path.header.stamp = this->now();
  path_ = path;
  planned_goal_x_ = goal_x;
  planned_goal_y_ = goal_y;
  have_path_ = true;
  path_pub_->publish(path);
  RCLCPP_INFO(this->get_logger(), "Published path with %zu waypoints", path.poses.size());
}

void PlannerNode::finishGoal(const char* reason)
{
  RCLCPP_INFO(this->get_logger(), "%s; waiting for the next goal.", reason);
  state_ = State::WAITING_FOR_GOAL;
  have_path_ = false;
  publishEmptyPath();
}

void PlannerNode::publishEmptyPath()
{
  // An empty path is the controller's cue to stop.
  nav_msgs::msg::Path empty;
  empty.header.stamp = this->now();
  if (map_) {
    empty.header.frame_id = map_->header.frame_id;
  }
  path_pub_->publish(empty);
}

#ifndef WATO_NODE_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
#endif
