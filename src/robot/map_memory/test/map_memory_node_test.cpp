#include <chrono>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "map_memory_node.hpp"

// Drives the node over real topics: a 60x60 map at 0.5 m with no inflation
// so cell values can be read back directly.
TEST(MapMemoryNodeTest, PlacesScanAtInterpolatedPoseAndUpdatesWhileStationary)
{
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("global_map.resolution", 0.5),
    rclcpp::Parameter("global_map.width", 60),
    rclcpp::Parameter("global_map.height", 60),
    rclcpp::Parameter("global_map.origin_x", -15.0),
    rclcpp::Parameter("global_map.origin_y", -15.0),
    rclcpp::Parameter("global_map.lethal_radius", 0.0),
    rclcpp::Parameter("global_map.inflation_radius", 0.0)});
  auto mapper = std::make_shared<MapMemoryNode>(options);
  auto client = std::make_shared<rclcpp::Node>("map_memory_test_client");
  auto odometry = client->create_publisher<nav_msgs::msg::Odometry>("/odom/filtered", 10);
  auto scans = client->create_publisher<nav_msgs::msg::OccupancyGrid>("/local_obstacles", 10);
  std::vector<nav_msgs::msg::OccupancyGrid> maps;
  auto subscription = client->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10,
    [&maps](const nav_msgs::msg::OccupancyGrid::SharedPtr map) { maps.push_back(*map); });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(mapper);
  executor.add_node(client);
  auto spin = [&executor](double seconds) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < end) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  };
  spin(0.4);

  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "sim_world";
  odom.child_frame_id = "laser";
  odom.pose.pose.orientation.w = 1.0;
  odom.header.stamp.sec = 10;
  odometry->publish(odom);
  spin(0.1);

  nav_msgs::msg::OccupancyGrid scan;
  scan.header.frame_id = "laser";
  scan.header.stamp.sec = 11;
  scan.info.resolution = 0.5;
  scan.info.width = scan.info.height = 4;
  scan.info.origin.orientation.w = 1.0;
  scan.data.assign(16, 0);
  scan.data[0] = 100;
  scans->publish(scan);
  spin(0.1);
  scan.header.stamp.nanosec = 500000000;
  scans->publish(scan);  // must not push out the scan waiting at t=11
  spin(0.1);
  odom.header.stamp.sec = 12;
  odom.pose.pose.position.x = 2.0;
  odometry->publish(odom);
  spin(0.15);

  ASSERT_FALSE(maps.empty());
  // At t=11 the sensor was at x=1.0; neither odometry sample alone is right.
  EXPECT_EQ(maps.back().data[30 * 60 + 32], 100);
  EXPECT_EQ(maps.back().data[30 * 60 + 30], -1);

  // The robot then stands still. New observations must still get in.
  scan.header.stamp.sec = 13;
  scan.header.stamp.nanosec = 0;
  scans->publish(scan);
  spin(0.1);
  odom.header.stamp.sec = 13;
  odometry->publish(odom);
  spin(0.15);
  EXPECT_EQ(maps.back().data[30 * 60 + 34], 100);

  scan.header.stamp.sec = 14;
  scan.data.assign(16, 0);
  scans->publish(scan);
  spin(0.1);
  odom.header.stamp.sec = 14;
  odometry->publish(odom);
  spin(0.15);
  EXPECT_EQ(maps.back().data[30 * 60 + 34], 0);

  // A simulation clock reset must throw away the scan waiting in the queue.
  scan.header.stamp.sec = 15;
  scans->publish(scan);
  spin(0.1);
  odom.header.stamp.sec = 1;
  odometry->publish(odom);
  spin(0.1);
  scan.header.stamp.sec = 2;
  scan.data[0] = 100;
  scans->publish(scan);
  spin(0.1);
  odom.header.stamp.sec = 2;
  odometry->publish(odom);
  spin(0.15);
  EXPECT_EQ(maps.back().data[30 * 60 + 34], 100);

  executor.remove_node(mapper);
  executor.remove_node(client);
  rclcpp::shutdown();
}
