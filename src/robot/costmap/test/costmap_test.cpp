#include <cmath>
#include <limits>

#include "gtest/gtest.h"

#include "costmap_core.hpp"

namespace
{

// 20x20 grid at 0.5 m: origin (-5, -5), robot in cell (10, 10).
robot::CostmapCore makeCostmap(double inflation = 1.0)
{
  robot::CostmapCore costmap(rclcpp::get_logger("costmap_test"));
  costmap.configure(0.5, 20, 20, inflation);
  return costmap;
}

sensor_msgs::msg::LaserScan singleBeam(double angle, double range)
{
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = angle;
  scan.angle_max = angle;
  scan.angle_increment = 0.0;
  scan.range_min = 0.1;
  scan.range_max = 10.0;
  scan.ranges = {static_cast<float>(range)};
  return scan;
}

}  // namespace

TEST(CostmapTest, GridIsCentredOnRobot)
{
  auto costmap = makeCostmap();
  const auto& info = costmap.grid().info;
  EXPECT_DOUBLE_EQ(info.origin.position.x, -5.0);
  EXPECT_DOUBLE_EQ(info.origin.position.y, -5.0);
  EXPECT_EQ(costmap.grid().data.size(), 400u);
}

TEST(CostmapTest, BeamMarksHitAndClearsPathToIt)
{
  auto costmap = makeCostmap();
  costmap.processScan(singleBeam(0.0, 3.0));

  // Hit at (3, 0) -> cell (16, 10).
  EXPECT_EQ(costmap.cellCost(16, 10), 100);
  // Every cell between the robot and the hit is free, except the one right
  // next to the obstacle, which the inflation band covers.
  for (int cx = 10; cx < 15; ++cx) {
    EXPECT_EQ(costmap.cellCost(cx, 10), 0) << "cell " << cx;
  }
  EXPECT_EQ(costmap.cellCost(15, 10), 50);
  // Cells beyond the hit and off the beam stay unknown.
  EXPECT_EQ(costmap.cellCost(19, 10), -1);
  EXPECT_EQ(costmap.cellCost(10, 15), -1);
}

TEST(CostmapTest, InflationFallsOffLinearly)
{
  auto costmap = makeCostmap(1.0);
  costmap.processScan(singleBeam(0.0, 3.0));

  // One cell (0.5 m) from the obstacle: 100 * (1 - 0.5 / 1.0) = 50.
  EXPECT_EQ(costmap.cellCost(17, 10), 50);
  EXPECT_EQ(costmap.cellCost(16, 11), 50);
  // Two cells (1.0 m) is exactly the radius: no inflation.
  EXPECT_EQ(costmap.cellCost(18, 10), -1);
  // Inflation never lowers a free cell below a real obstacle.
  EXPECT_EQ(costmap.cellCost(16, 10), 100);
}

TEST(CostmapTest, InfiniteBeamClearsWithoutObstacle)
{
  auto costmap = makeCostmap();
  costmap.processScan(singleBeam(M_PI / 2.0, std::numeric_limits<double>::infinity()));

  // Beam points +y; range_max (10 m) leaves the grid, so everything to the
  // top edge is free and nothing is marked occupied.
  for (int cy = 10; cy < 20; ++cy) {
    EXPECT_EQ(costmap.cellCost(10, cy), 0) << "cell " << cy;
  }
  for (const int8_t cell : costmap.grid().data) {
    EXPECT_NE(cell, 100);
  }
}

TEST(CostmapTest, EachScanStartsFresh)
{
  auto costmap = makeCostmap();
  costmap.processScan(singleBeam(0.0, 3.0));
  costmap.processScan(singleBeam(0.0, 2.0));

  EXPECT_EQ(costmap.cellCost(16, 10), -1);  // old hit at (3, 0) is gone
  EXPECT_EQ(costmap.cellCost(14, 10), 100); // new hit at (2, 0)
}
