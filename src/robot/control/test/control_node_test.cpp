#include <chrono>
#include <cmath>
#include <thread>
#include <vector>
#include "gtest/gtest.h"
#include "control_node.hpp"

TEST(ControlNodeTest, TracksAxleInsteadOfStoppingWhenLidarPassesGoal) {
  rclcpp::init(0, nullptr);
  auto control = std::make_shared<ControlNode>();
  auto client = std::make_shared<rclcpp::Node>("control_test_client");
  auto odometry = client->create_publisher<nav_msgs::msg::Odometry>("/odom/filtered", 10);
  auto paths = client->create_publisher<nav_msgs::msg::Path>("/path", 10);
  std::vector<geometry_msgs::msg::Twist> commands;
  auto subscription = client->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10,
    [&commands](const geometry_msgs::msg::Twist::SharedPtr cmd) { commands.push_back(*cmd); });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(control);
  executor.add_node(client);
  auto spin = [&executor](double seconds) {
    auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < end) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  };
  spin(0.4);
  nav_msgs::msg::Path path;
  for (int i = 0; i <= 2; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.y = i * 0.5;
    path.poses.push_back(pose);
  }
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.position.y = 1.3;
  odom.pose.pose.orientation.z = std::sqrt(0.5);
  odom.pose.pose.orientation.w = std::sqrt(0.5);
  // Robot axle at (0,0), facing north. Lidar already beyond the goal at y=1.
  paths->publish(path);
  odometry->publish(odom);
  spin(0.25);
  ASSERT_FALSE(commands.empty());
  EXPECT_GT(commands.back().linear.x, 0.0);
  EXPECT_NEAR(commands.back().angular.z, 0.0, 1e-6);
  // Arrival is measured at the axle, with the lidar another 1.3 m ahead.
  odom.pose.pose.position.y = 2.3;
  odometry->publish(odom);
  spin(0.25);
  EXPECT_DOUBLE_EQ(commands.back().linear.x, 0.0);
  EXPECT_DOUBLE_EQ(commands.back().angular.z, 0.0);
  executor.remove_node(control);
  executor.remove_node(client);
  rclcpp::shutdown();
}
