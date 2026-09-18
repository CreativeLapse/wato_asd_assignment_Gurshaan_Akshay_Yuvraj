#include <cmath>
#include <limits>

#include "gtest/gtest.h"

#include "costmap_core.hpp"

namespace
{

// 20x20 grid at 0.5 m: origin (-5, -5), robot in cell (10, 10).
robot::CostmapCore makeCostmap(double lethal = 0.5, double inflation = 1.0, double decay = 2.0)
{
  robot::CostmapCore costmap(rclcpp::get_logger("costmap_test"));
  costmap.configure(0.5, 20, 20, lethal, inflation, decay);
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

int decayed(double dist, double lethal, double decay)
{
  return static_cast<int>(std::lround(98.0 * std::exp(-decay * (dist - lethal))));
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
  // Cells between the robot and the inflation band are free.
  for (int cx = 10; cx < 14; ++cx) {
    EXPECT_EQ(costmap.cellCost(cx, 10), 0) << "cell " << cx;
  }
  // Cells beyond the hit and off the beam stay unknown.
  EXPECT_EQ(costmap.cellCost(19, 10), -1);
  EXPECT_EQ(costmap.cellCost(10, 15), -1);
}

TEST(CostmapTest, LethalDiscThenExponentialDecay)
{
  auto costmap = makeCostmap(1.0, 2.5, 1.5);
  costmap.processScan(singleBeam(0.0, 3.0));

  EXPECT_EQ(costmap.cellCost(16, 10), 100);
  // Within 1.0 m: lethal.
  EXPECT_EQ(costmap.cellCost(15, 10), 99);
  EXPECT_EQ(costmap.cellCost(14, 10), 99);
  EXPECT_EQ(costmap.cellCost(16, 12), 99);
  // Past it the cost decays with distance.
  EXPECT_EQ(costmap.cellCost(13, 10), decayed(1.5, 1.0, 1.5));
  EXPECT_EQ(costmap.cellCost(12, 10), decayed(2.0, 1.0, 1.5));
  EXPECT_EQ(costmap.cellCost(11, 10), decayed(2.5, 1.0, 1.5));
  EXPECT_GT(costmap.cellCost(13, 10), costmap.cellCost(12, 10));
  // Beyond the inflation radius nothing is written.
  EXPECT_EQ(costmap.cellCost(16, 16), -1);
}

TEST(CostmapTest, InflationNeverLowersACell)
{
  auto costmap = makeCostmap(1.0, 2.5, 1.5);
  sensor_msgs::msg::LaserScan scan = singleBeam(0.0, 3.0);
  scan.angle_max = 0.0;
  scan.ranges = {3.0f, 3.5f};
  costmap.processScan(scan);

  // Two hits in a row: the nearer one stays a full obstacle even though it
  // sits inside the other's lethal disc.
  EXPECT_EQ(costmap.cellCost(16, 10), 100);
  EXPECT_EQ(costmap.cellCost(17, 10), 100);
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

  EXPECT_NE(costmap.cellCost(16, 10), 100); // old hit at (3, 0) is gone
  EXPECT_EQ(costmap.cellCost(17, 10), -1);  // and so is its inflation
  EXPECT_EQ(costmap.cellCost(14, 10), 100); // new hit at (2, 0)
}
