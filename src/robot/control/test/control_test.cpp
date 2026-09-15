#include <cmath>

#include "gtest/gtest.h"

#include "control_core.hpp"

namespace
{

robot::ControlCore makeController()
{
  robot::ControlCore control(rclcpp::get_logger("control_test"));
  // lookahead 1.0, speed 0.5, max angular 1.0, tolerance 0.2, turn-in-place 0.8 rad, slowdown 1.0
  control.configure(1.0, 0.5, 1.0, 0.2, 0.8, 1.0);
  return control;
}

// Straight path from (0, 0) toward (dx, dy) * 10 in 0.5 m steps.
nav_msgs::msg::Path makePath(double dx, double dy)
{
  nav_msgs::msg::Path path;
  for (int i = 0; i <= 10; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = dx * 0.5 * i;
    pose.pose.position.y = dy * 0.5 * i;
    path.poses.push_back(pose);
  }
  return path;
}

}  // namespace

TEST(ControlTest, NoPathMeansStop)
{
  auto control = makeController();
  const auto cmd = control.computeCommand(0.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(cmd.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(cmd.angular.z, 0.0);
  EXPECT_FALSE(control.hasPath());
}

TEST(ControlTest, DrivesStraightAlongStraightPath)
{
  auto control = makeController();
  control.setPath(makePath(1.0, 0.0));

  const auto cmd = control.computeCommand(0.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(cmd.linear.x, 0.5);
  EXPECT_NEAR(cmd.angular.z, 0.0, 1e-9);
}

TEST(ControlTest, LookaheadIsFirstPointAtLeastLookaheadAway)
{
  auto control = makeController();
  control.setPath(makePath(1.0, 0.0));

  geometry_msgs::msg::Point target;
  ASSERT_TRUE(control.findLookahead(0.0, 0.0, target));
  EXPECT_DOUBLE_EQ(target.x, 1.0);
  EXPECT_DOUBLE_EQ(target.y, 0.0);

  // From partway along the path it looks ahead from the nearest point.
  ASSERT_TRUE(control.findLookahead(2.1, 0.0, target));
  EXPECT_DOUBLE_EQ(target.x, 3.5);
}

TEST(ControlTest, TurnsLeftTowardPathOnTheLeft)
{
  auto control = makeController();
  // Path runs along +x but offset 0.3 m to the robot's left.
  auto path = makePath(1.0, 0.0);
  for (auto& pose : path.poses) {
    pose.pose.position.y += 0.3;
  }
  control.setPath(path);

  const auto cmd = control.computeCommand(0.0, 0.0, 0.0);
  EXPECT_GT(cmd.linear.x, 0.0);
  EXPECT_GT(cmd.angular.z, 0.0);
}

TEST(ControlTest, SpinsInPlaceWhenFacingAway)
{
  auto control = makeController();
  control.setPath(makePath(0.0, 1.0));  // path heads +y, robot faces +x

  const auto cmd = control.computeCommand(0.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(cmd.linear.x, 0.0);
  EXPECT_GT(cmd.angular.z, 0.0);        // counter-clockwise toward +y
  EXPECT_LE(cmd.angular.z, 1.0);        // respects max_angular_speed
}

TEST(ControlTest, StopsAtGoal)
{
  auto control = makeController();
  control.setPath(makePath(1.0, 0.0));  // ends at (5, 0)

  EXPECT_TRUE(control.goalReached(4.9, 0.05));
  EXPECT_FALSE(control.goalReached(4.0, 0.0));

  const auto cmd = control.computeCommand(4.9, 0.05, 0.0);
  EXPECT_DOUBLE_EQ(cmd.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(cmd.angular.z, 0.0);
}

TEST(ControlTest, SlowsDownApproachingGoal)
{
  auto control = makeController();
  control.setPath(makePath(1.0, 0.0));

  const auto far = control.computeCommand(0.0, 0.0, 0.0);
  const auto near = control.computeCommand(4.5, 0.0, 0.0);
  EXPECT_LT(near.linear.x, far.linear.x);
  EXPECT_GT(near.linear.x, 0.0);
}

TEST(ControlTest, ClearingPathStopsRobot)
{
  auto control = makeController();
  control.setPath(makePath(1.0, 0.0));
  control.clearPath();

  EXPECT_FALSE(control.hasPath());
  const auto cmd = control.computeCommand(0.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(cmd.linear.x, 0.0);
}
