#include <cmath>

#include "gtest/gtest.h"

#include "map_memory_core.hpp"

namespace
{

// Global map: 20x20 at 0.5 m, origin (-5, -5), no inflation.
robot::MapMemoryCore makeMap(double lethal = 0.0, double inflation = 0.0, double decay = 1.0)
{
  robot::MapMemoryCore map(rclcpp::get_logger("map_memory_test"));
  map.configure(0.5, 20, 20, -5.0, -5.0, lethal, inflation, decay);
  return map;
}

// Local grid: 10x10 at 0.5 m centred on the robot, all unknown.
nav_msgs::msg::OccupancyGrid makeLocal()
{
  nav_msgs::msg::OccupancyGrid local;
  local.info.resolution = 0.5;
  local.info.width = 10;
  local.info.height = 10;
  local.info.origin.position.x = -2.5;
  local.info.origin.position.y = -2.5;
  local.data.assign(100, -1);
  return local;
}

void setLocal(nav_msgs::msg::OccupancyGrid& local, int cx, int cy, int8_t value)
{
  local.data[cy * local.info.width + cx] = value;
}

}  // namespace

TEST(MapMemoryTest, StartsUnknown)
{
  auto map = makeMap();
  EXPECT_EQ(map.map().data.size(), 400u);
  for (const int8_t cell : map.map().data) {
    EXPECT_EQ(cell, -1);
  }
}

TEST(MapMemoryTest, ObstacleLandsAtRobotOffsetWithoutRotation)
{
  auto map = makeMap();
  auto local = makeLocal();
  // Local cell (7, 5) spans x in [1.0, 1.5), y in [0, 0.5) in the robot frame.
  setLocal(local, 7, 5, 100);

  map.fuse(local, 2.0, 1.0, 0.0);

  // World point (3.25, 1.25) -> global cell (16, 12).
  EXPECT_EQ(map.cellCost(16, 12), 100);
  // Cells the local window didn't cover stay unknown.
  EXPECT_EQ(map.cellCost(0, 0), -1);
}

TEST(MapMemoryTest, ObstacleIsRotatedByRobotYaw)
{
  auto map = makeMap();
  auto local = makeLocal();
  setLocal(local, 7, 5, 100);

  // Facing +y: the obstacle 1.25 m ahead ends up 1.25 m up from the robot.
  map.fuse(local, 2.0, 1.0, M_PI / 2.0);

  // World point (1.75, 2.25) -> global cell (13, 14).
  EXPECT_EQ(map.cellCost(13, 14), 100);
  EXPECT_EQ(map.cellCost(16, 12), -1);
}

TEST(MapMemoryTest, UnknownNeverErasesKnown)
{
  auto map = makeMap();
  auto local = makeLocal();
  setLocal(local, 7, 5, 100);
  map.fuse(local, 2.0, 1.0, 0.0);

  auto blank = makeLocal();
  map.fuse(blank, 2.0, 1.0, 0.0);

  EXPECT_EQ(map.cellCost(16, 12), 100);
}

TEST(MapMemoryTest, NewKnownValueOverwritesOld)
{
  auto map = makeMap();
  auto local = makeLocal();
  setLocal(local, 7, 5, 100);
  map.fuse(local, 2.0, 1.0, 0.0);

  // Same spot now seen as free: the map should believe the newer scan.
  setLocal(local, 7, 5, 0);
  map.fuse(local, 2.0, 1.0, 0.0);

  EXPECT_EQ(map.cellCost(16, 12), 0);
}

TEST(MapMemoryTest, InflatesRememberedObstacles)
{
  auto map = makeMap(0.5, 2.0, 1.0);
  auto local = makeLocal();
  setLocal(local, 5, 5, 100);  // right at the robot -> global cell (10, 10)
  map.fuse(local, 0.0, 0.0, 0.0);

  EXPECT_EQ(map.cellCost(10, 10), 100);
  EXPECT_EQ(map.cellCost(11, 10), 99);  // 0.5 m: lethal
  EXPECT_EQ(map.cellCost(12, 10), static_cast<int>(std::lround(98.0 * std::exp(-0.5))));
  EXPECT_EQ(map.cellCost(14, 10), static_cast<int>(std::lround(98.0 * std::exp(-1.5))));
  EXPECT_EQ(map.cellCost(15, 10), -1);  // beyond 2.0 m
}

TEST(MapMemoryTest, OccludedObstacleKeepsItsMarginUntilSeenFree)
{
  auto map = makeMap(0.5, 2.0, 1.0);
  auto local = makeLocal();
  setLocal(local, 5, 5, 100);
  map.fuse(local, 0.0, 0.0, 0.0);
  EXPECT_EQ(map.cellCost(10, 10), 100);
  EXPECT_EQ(map.cellCost(11, 10), 99);

  // The obstacle is out of view. A beam only confirms its neighbour is
  // empty; the body still can't fit there, so the margin has to stay.
  local = makeLocal();
  setLocal(local, 6, 5, 0);
  map.fuse(local, 0.0, 0.0, 0.0);
  EXPECT_EQ(map.cellCost(10, 10), 100);
  EXPECT_EQ(map.cellCost(11, 10), 99);

  // Once the obstacle itself is seen free, its margin goes with it.
  local = makeLocal();
  setLocal(local, 5, 5, 0);
  map.fuse(local, 0.0, 0.0, 0.0);
  EXPECT_EQ(map.cellCost(10, 10), 0);
  EXPECT_EQ(map.cellCost(11, 10), 0);
  EXPECT_EQ(map.cellCost(12, 10), -1);
}

TEST(MapMemoryTest, FreeSpaceFillsWithoutHoles)
{
  auto map = makeMap();
  auto local = makeLocal();
  std::fill(local.data.begin(), local.data.end(), 0);

  // Robot at the map centre, rotated 45 degrees: the diamond-shaped window
  // must leave no unknown gaps inside it.
  map.fuse(local, 0.0, 0.0, M_PI / 4.0);

  // Every global cell whose centre is within 1.5 m of the robot is well
  // inside the rotated 5 m window and must be free.
  for (int gy = 0; gy < 20; ++gy) {
    for (int gx = 0; gx < 20; ++gx) {
      const double wx = -5.0 + (gx + 0.5) * 0.5;
      const double wy = -5.0 + (gy + 0.5) * 0.5;
      if (std::hypot(wx, wy) <= 1.5) {
        EXPECT_EQ(map.cellCost(gx, gy), 0) << "hole at (" << gx << ", " << gy << ")";
      }
    }
  }
}
