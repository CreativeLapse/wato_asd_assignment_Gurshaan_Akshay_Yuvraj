#include <cmath>

#include "gtest/gtest.h"

#include "control_core.hpp"

namespace
{

constexpr double kDt = 0.1;

robot::ControlCore makeController()
{
  robot::ControlCore control(rclcpp::get_logger("control_test"));
  robot::ControlParams params;
  params.max_speed = 0.8;
  params.min_speed = 0.15;
  params.max_angular_speed = 1.0;
  params.accel = 0.5;
  params.decel = 1.0;
  params.lookahead_min = 1.0;
  params.lookahead_max = 2.0;
  params.lookahead_gain = 2.0;
  params.curvature_gain = 2.0;
  params.turn_enter_angle = 1.0;
  params.turn_exit_angle = 0.3;
  params.turn_gain = 1.5;
  params.slowdown_distance = 1.5;
  params.goal_tolerance = 0.3;
  control.configure(params);
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

// Runs the controller at a fixed pose until the speed ramp has settled.
geometry_msgs::msg::Twist settle(robot::ControlCore& control, double x, double y, double yaw)
{
  geometry_msgs::msg::Twist cmd;
  for (int i = 0; i < 50; ++i) {
    cmd = control.computeCommand(x, y, yaw, kDt);
  }
  return cmd;
}

}  // namespace

TEST(ControlTest, NoPathMeansStop)
{
  auto control = makeController();
  const auto cmd = control.computeCommand(0.0, 0.0, 0.0, kDt);
  EXPECT_DOUBLE_EQ(cmd.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(cmd.angular.z, 0.0);
  EXPECT_FALSE(control.hasPath());
}

TEST(ControlTest, RampsUpToCruisingSpeedOnAStraight)
{
  auto control = makeController();
  control.setPath(makePath(1.0, 0.0));

  const auto first = control.computeCommand(0.0, 0.0, 0.0, kDt);
  EXPECT_NEAR(first.linear.x, 0.05, 1e-9);  // accel * dt
  EXPECT_NEAR(first.angular.z, 0.0, 1e-9);

  const auto cruising = settle(control, 0.0, 0.0, 0.0);
  EXPECT_NEAR(cruising.linear.x, 0.8, 1e-9);
  EXPECT_NEAR(cruising.angular.z, 0.0, 1e-9);
}

TEST(ControlTest, LookaheadIsFirstPointAtLeastLookaheadAway)
{
  auto control = makeController();
  control.setPath(makePath(1.0, 0.0));

  geometry_msgs::msg::Point target;
  ASSERT_TRUE(control.findLookahead(0.0, 0.0, 1.0, target));
  EXPECT_DOUBLE_EQ(target.x, 1.0);
  EXPECT_DOUBLE_EQ(target.y, 0.0);

  // From partway along the path it looks ahead from the nearest point.
  ASSERT_TRUE(control.findLookahead(2.1, 0.0, 1.0, target));
  EXPECT_DOUBLE_EQ(target.x, 3.5);

  // A longer lookahead reaches further along.
  ASSERT_TRUE(control.findLookahead(0.0, 0.0, 2.0, target));
  EXPECT_DOUBLE_EQ(target.x, 2.0);
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

  const auto cmd = settle(control, 0.0, 0.0, 0.0);
  EXPECT_GT(cmd.linear.x, 0.0);
  EXPECT_GT(cmd.angular.z, 0.0);
}

TEST(ControlTest, SlowsDownForTightArcs)
{
  auto straight = makeController();
  straight.setPath(makePath(1.0, 0.0));
  const auto on_straight = settle(straight, 0.0, 0.0, 0.0);

  auto curved = makeController();
  auto path = makePath(1.0, 0.0);
  for (auto& pose : path.poses) {
    pose.pose.position.y += 0.7;
  }
  curved.setPath(path);
  const auto on_curve = settle(curved, 0.0, 0.0, 0.0);

  EXPECT_GT(on_curve.linear.x, 0.0);
  EXPECT_LT(on_curve.linear.x, on_straight.linear.x);
}

TEST(ControlTest, SpinsInPlaceWhenFacingAway)
{
  auto control = makeController();
  control.setPath(makePath(0.0, 1.0));  // path heads +y, robot faces +x

  const auto cmd = control.computeCommand(0.0, 0.0, 0.0, kDt);
  EXPECT_DOUBLE_EQ(cmd.linear.x, 0.0);
  EXPECT_GT(cmd.angular.z, 0.0);        // counter-clockwise toward +y
  EXPECT_LE(cmd.angular.z, 1.0);        // respects max_angular_speed
}

TEST(ControlTest, KeepsSpinningUntilNearlyAligned)
{
  auto control = makeController();
  control.setPath(makePath(0.0, 1.0));  // path heads +y (yaw pi/2)

  // 0.6 rad of heading error is fine to drive through from a standing start.
  auto fresh = makeController();
  fresh.setPath(makePath(0.0, 1.0));
  EXPECT_GT(fresh.computeCommand(0.0, 0.0, M_PI / 2.0 - 0.6, kDt).linear.x, 0.0);

  // But once spinning, the same error keeps it spinning until it drops
  // below the exit angle.
  control.computeCommand(0.0, 0.0, 0.0, kDt);
  auto cmd = control.computeCommand(0.0, 0.0, M_PI / 2.0 - 0.6, kDt);
  EXPECT_DOUBLE_EQ(cmd.linear.x, 0.0);
  EXPECT_GT(cmd.angular.z, 0.0);

  cmd = control.computeCommand(0.0, 0.0, M_PI / 2.0 - 0.2, kDt);
  EXPECT_GT(cmd.linear.x, 0.0);
}

TEST(ControlTest, StopsAtGoal)
{
  auto control = makeController();
  control.setPath(makePath(1.0, 0.0));  // ends at (5, 0)

  EXPECT_TRUE(control.goalReached(4.8, 0.05));
  EXPECT_FALSE(control.goalReached(4.0, 0.0));

  const auto cmd = control.computeCommand(4.8, 0.05, 0.0, kDt);
  EXPECT_DOUBLE_EQ(cmd.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(cmd.angular.z, 0.0);
}

TEST(ControlTest, SlowsDownApproachingGoal)
{
  auto control = makeController();
  control.setPath(makePath(1.0, 0.0));

  const auto far = settle(control, 0.0, 0.0, 0.0);
  const auto near = settle(control, 4.5, 0.0, 0.0);
  EXPECT_LT(near.linear.x, far.linear.x);
  EXPECT_GE(near.linear.x, 0.15);  // never below min_speed
}

TEST(ControlTest, NewPathRestartsTheRamp)
{
  auto control = makeController();
  control.setPath(makePath(1.0, 0.0));
  settle(control, 0.0, 0.0, 0.0);

  control.setPath(makePath(1.0, 0.0));
  const auto cmd = control.computeCommand(0.0, 0.0, 0.0, kDt);
  EXPECT_NEAR(cmd.linear.x, 0.05, 1e-9);
}

TEST(ControlTest, ClearingPathStopsRobot)
{
  auto control = makeController();
  control.setPath(makePath(1.0, 0.0));
  control.clearPath();

  EXPECT_FALSE(control.hasPath());
  const auto cmd = control.computeCommand(0.0, 0.0, 0.0, kDt);
  EXPECT_DOUBLE_EQ(cmd.linear.x, 0.0);
}
