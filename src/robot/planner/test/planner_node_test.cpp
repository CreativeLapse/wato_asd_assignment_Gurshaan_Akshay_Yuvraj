#include <chrono>
#include <cmath>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "planner_node.hpp"

// Drives the node over real topics. The map is 20x20 at 0.5 m with origin
// (-5, -5); odometry puts the lidar at x=-0.7, so the axle is at x=-2.0.
class PlannerNodeTest : public ::testing::Test {
  protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override
    {
      planner = std::make_shared<PlannerNode>();
      client = std::make_shared<rclcpp::Node>("planner_test_client");
      maps = client->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
      odometry = client->create_publisher<nav_msgs::msg::Odometry>("/odom/filtered", 10);
      goals = client->create_publisher<geometry_msgs::msg::PointStamped>("/goal_point", 10);
      paths = client->create_subscription<nav_msgs::msg::Path>("/path", 10,
        [this](const nav_msgs::msg::Path::SharedPtr p) { received.push_back(*p); });
      executor.add_node(planner);
      executor.add_node(client);
      spin(0.4);

      map.header.frame_id = "sim_world";
      map.info.resolution = 0.5;
      map.info.width = map.info.height = 20;
      map.info.origin.position.x = map.info.origin.position.y = -5.0;
      map.info.origin.orientation.w = 1.0;
      map.data.assign(400, 0);

      nav_msgs::msg::Odometry odom;
      odom.pose.pose.position.x = -0.7;
      odom.pose.pose.orientation.w = 1.0;
      odometry->publish(odom);
      maps->publish(map);
      spin(0.2);
    }

    void TearDown() override
    {
      executor.remove_node(planner);
      executor.remove_node(client);
    }

    void spin(double seconds)
    {
      const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
      while (std::chrono::steady_clock::now() < end) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    }

    void sendGoal()
    {
      geometry_msgs::msg::PointStamped goal;
      goal.header.frame_id = "sim_world";
      goal.point.x = 2.0;
      goals->publish(goal);
      spin(0.2);
    }

    rclcpp::executors::SingleThreadedExecutor executor;
    std::shared_ptr<PlannerNode> planner;
    rclcpp::Node::SharedPtr client;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr maps;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goals;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr paths;
    nav_msgs::msg::OccupancyGrid map;
    std::vector<nav_msgs::msg::Path> received;
};

TEST_F(PlannerNodeTest, HeartbeatsAndUnrelatedChangesDoNotReplaceRoute)
{
  sendGoal();
  ASSERT_FALSE(received.empty());
  ASSERT_FALSE(received.back().poses.empty());
  EXPECT_NEAR(received.back().poses.front().pose.position.x, -1.75, 1e-6);

  const auto count = received.size();
  map.header.stamp.sec = 100;
  maps->publish(map);
  spin(0.2);
  map.data[2 * 20 + 10] = 100;
  maps->publish(map);
  spin(0.2);
  EXPECT_EQ(received.size(), count);
}

TEST_F(PlannerNodeTest, RetainsGoalAcrossTemporaryPlanningFailure)
{
  sendGoal();
  ASSERT_FALSE(received.empty());
  ASSERT_FALSE(received.back().poses.empty());

  for (int y = 0; y < 20; ++y) {
    map.data[y * 20 + 10] = 100;
  }
  maps->publish(map);
  spin(0.2);
  ASSERT_TRUE(received.back().poses.empty());

  // A new opening must resume the existing goal, without another click.
  map.data[15 * 20 + 10] = 0;
  maps->publish(map);
  spin(0.2);
  ASSERT_FALSE(received.back().poses.empty());
  EXPECT_NEAR(received.back().poses.back().pose.position.x, 2.25, 1e-6);

  robot::PlannerCore validator(rclcpp::get_logger("validator"));
  EXPECT_TRUE(validator.remainingPathIsValid(map, received.back(), -2.0, 0.0));
}

TEST_F(PlannerNodeTest, ReplacesStillValidDetourWhenAShortcutOpens)
{
  for (int y = 0; y < 15; ++y) {
    map.data[y * 20 + 10] = 100;
  }
  maps->publish(map);
  spin(0.2);
  sendGoal();
  ASSERT_FALSE(received.empty());
  ASSERT_FALSE(received.back().poses.empty());
  const auto detour = received.back();
  const auto count = received.size();

  // The old route above the wall remains usable; a new opening at y=0
  // creates a strictly shorter route and must replace it immediately.
  map.data[10 * 20 + 10] = 0;
  maps->publish(map);
  spin(0.2);
  ASSERT_GT(received.size(), count);
  ASSERT_FALSE(received.back().poses.empty());

  robot::PlannerCore validator(rclcpp::get_logger("validator"));
  ASSERT_TRUE(validator.remainingPathIsValid(map, detour, -2.0, 0.0));
  EXPECT_GT(validator.remainingPathCost(map, detour, -2.0, 0.0), 8.0);
  EXPECT_NEAR(validator.remainingPathCost(map, received.back(), -2.0, 0.0), 8.0, 1e-9);

  // A cost change beside the route is no reason to swap it, even across
  // several timer ticks.
  const auto optimum_count = received.size();
  map.data[10 * 20 + 9] = 49;
  maps->publish(map);
  spin(1.1);
  EXPECT_EQ(received.size(), optimum_count);
}

TEST_F(PlannerNodeTest, ReplansFromNewRobotCellEvenWithoutAMapUpdate)
{
  sendGoal();
  ASSERT_FALSE(received.back().poses.empty());
  const auto count = received.size();

  nav_msgs::msg::Odometry odom;
  odom.pose.pose.position.x = -0.7;
  odom.pose.pose.position.y = 1.0;
  odom.pose.pose.orientation.w = 1.0;
  odometry->publish(odom);
  spin(0.7);
  ASSERT_GT(received.size(), count);
  ASSERT_FALSE(received.back().poses.empty());
  EXPECT_NEAR(received.back().poses.front().pose.position.y, 1.25, 1e-9);
}
