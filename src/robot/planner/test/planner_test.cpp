#include <cmath>

#include "gtest/gtest.h"

#include "planner_core.hpp"

namespace
{

constexpr int kSize = 20;
constexpr double kRes = 0.5;

// 20x20 map at 0.5 m, origin (-5, -5), all free.
nav_msgs::msg::OccupancyGrid makeMap()
{
  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "map";
  map.info.resolution = kRes;
  map.info.width = kSize;
  map.info.height = kSize;
  map.info.origin.position.x = -5.0;
  map.info.origin.position.y = -5.0;
  map.data.assign(kSize * kSize, 0);
  return map;
}

void setCell(nav_msgs::msg::OccupancyGrid& map, int cx, int cy, int8_t value)
{
  map.data[cy * kSize + cx] = value;
}

int8_t cellAtWorld(const nav_msgs::msg::OccupancyGrid& map, double wx, double wy)
{
  const int cx = static_cast<int>(std::floor((wx + 5.0) / kRes));
  const int cy = static_cast<int>(std::floor((wy + 5.0) / kRes));
  return map.data[cy * kSize + cx];
}

robot::PlannerCore makePlanner()
{
  robot::PlannerCore planner(rclcpp::get_logger("planner_test"));
  planner.configure(50, 30, 3.0, 10);
  return planner;
}

// A vertical wall at column 10 from the bottom up to (not including) row 15.
void addWallWithGapAtTop(nav_msgs::msg::OccupancyGrid& map)
{
  for (int cy = 0; cy < 15; ++cy) {
    setCell(map, 10, cy, 100);
  }
}

}  // namespace

TEST(PlannerTest, StraightLineOnEmptyMap)
{
  auto planner = makePlanner();
  auto map = makeMap();
  nav_msgs::msg::Path path;
  double gx = 0.0;
  double gy = 0.0;

  ASSERT_TRUE(planner.plan(map, -2.0, 0.0, 2.0, 0.0, path, gx, gy));

  ASSERT_FALSE(path.poses.empty());
  EXPECT_EQ(path.header.frame_id, "map");
  // Starts at the robot's cell and ends at the goal's cell.
  EXPECT_NEAR(path.poses.front().pose.position.x, -1.75, 1e-9);
  EXPECT_NEAR(path.poses.back().pose.position.x, 2.25, 1e-9);
  EXPECT_NEAR(gx, 2.25, 1e-9);
  EXPECT_NEAR(gy, 0.25, 1e-9);
  // Straight line: 8 steps for 8 cells.
  EXPECT_EQ(path.poses.size(), 9u);
}

TEST(PlannerTest, RoutesAroundWall)
{
  auto planner = makePlanner();
  auto map = makeMap();
  addWallWithGapAtTop(map);
  nav_msgs::msg::Path path;
  double gx = 0.0;
  double gy = 0.0;

  ASSERT_TRUE(planner.plan(map, -2.5, 0.0, 2.5, 0.0, path, gx, gy));

  bool crossed_through_gap = false;
  for (const auto& pose : path.poses) {
    const auto& p = pose.pose.position;
    EXPECT_LT(cellAtWorld(map, p.x, p.y), 50) << "path touches a wall cell";
    // The wall sits at world x in [0, 0.5); the gap is y >= 2.5.
    if (p.x > 0.0 && p.x < 0.5) {
      crossed_through_gap = p.y >= 2.5;
    }
  }
  EXPECT_TRUE(crossed_through_gap);

  // Consecutive waypoints are neighbours: no jumps.
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto& a = path.poses[i - 1].pose.position;
    const auto& b = path.poses[i].pose.position;
    EXPECT_LE(std::hypot(b.x - a.x, b.y - a.y), kRes * std::sqrt(2.0) + 1e-9);
  }
}

TEST(PlannerTest, PrefersLowCostCells)
{
  auto planner = makePlanner();
  auto map = makeMap();
  // A band of cost 40 along y = 0 (row 10) makes the direct route pricier
  // than a detour one row up, which is free.
  for (int cx = 0; cx < kSize; ++cx) {
    setCell(map, cx, 10, 40);
  }
  nav_msgs::msg::Path path;
  double gx = 0.0;
  double gy = 0.0;

  ASSERT_TRUE(planner.plan(map, -3.0, 0.0, 3.0, 0.0, path, gx, gy));

  // Only the first and last waypoints (forced onto row 10) may cost 40.
  for (size_t i = 1; i + 1 < path.poses.size(); ++i) {
    const auto& p = path.poses[i].pose.position;
    EXPECT_EQ(cellAtWorld(map, p.x, p.y), 0) << "waypoint " << i << " sits on the costly band";
  }
}

TEST(PlannerTest, BlockedGoalSnapsToNearestOpenCell)
{
  auto planner = makePlanner();
  auto map = makeMap();
  // A 3x3 obstacle centred on cell (15, 10) -> world (2.75, 0.25).
  for (int cy = 9; cy <= 11; ++cy) {
    for (int cx = 14; cx <= 16; ++cx) {
      setCell(map, cx, cy, 100);
    }
  }
  nav_msgs::msg::Path path;
  double gx = 0.0;
  double gy = 0.0;

  ASSERT_TRUE(planner.plan(map, -2.0, 0.0, 2.75, 0.25, path, gx, gy));

  EXPECT_LT(cellAtWorld(map, gx, gy), 50);
  EXPECT_NEAR(std::hypot(gx - 2.75, gy - 0.25), 1.0, 1e-9);  // two cells away
  EXPECT_NEAR(path.poses.back().pose.position.x, gx, 1e-9);
  EXPECT_NEAR(path.poses.back().pose.position.y, gy, 1e-9);
}

TEST(PlannerTest, UnknownCellsAreDrivable)
{
  auto planner = makePlanner();
  auto map = makeMap();
  std::fill(map.data.begin(), map.data.end(), -1);
  nav_msgs::msg::Path path;
  double gx = 0.0;
  double gy = 0.0;

  EXPECT_TRUE(planner.plan(map, -2.0, 0.0, 2.0, 0.0, path, gx, gy));
  EXPECT_FALSE(path.poses.empty());
}

TEST(PlannerTest, FailsWhenGoalIsWalledOff)
{
  auto planner = makePlanner();
  auto map = makeMap();
  for (int cy = 0; cy < kSize; ++cy) {
    setCell(map, 10, cy, 100);
  }
  nav_msgs::msg::Path path;
  double gx = 0.0;
  double gy = 0.0;

  EXPECT_FALSE(planner.plan(map, -2.0, 0.0, 2.0, 0.0, path, gx, gy));
  EXPECT_TRUE(path.poses.empty());
}

TEST(PlannerTest, RejectsPointsOffTheMap)
{
  auto planner = makePlanner();
  auto map = makeMap();
  nav_msgs::msg::Path path;
  double gx = 0.0;
  double gy = 0.0;

  EXPECT_FALSE(planner.plan(map, 0.0, 0.0, 50.0, 0.0, path, gx, gy));
  EXPECT_FALSE(planner.plan(map, -50.0, 0.0, 0.0, 0.0, path, gx, gy));
}
