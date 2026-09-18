#include <cmath>
#include <limits>
#include <random>

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

// lethal 99, unknown 10, weight 5, snap 12 cells, escape 3 cells
robot::PlannerCore makePlanner()
{
  robot::PlannerCore planner(rclcpp::get_logger("planner_test"));
  planner.configure(99, 10, 5.0, 12, 3);
  return planner;
}

// Same, but costs don't matter: pure shortest distance.
robot::PlannerCore makeShortestPlanner()
{
  robot::PlannerCore planner(rclcpp::get_logger("planner_test"));
  planner.configure(99, 10, 0.0, 12, 3);
  return planner;
}

// A vertical wall at column 10 from the bottom up to (not including) row 15.
void addWallWithGapAtTop(nav_msgs::msg::OccupancyGrid& map)
{
  for (int cy = 0; cy < 15; ++cy) {
    setCell(map, 10, cy, 100);
  }
}

// Plain Dijkstra with none of the planner's machinery, so A* has something
// independent to be checked against. Returns metres, infinity if cut off.
double referenceShortestDistance(const nav_msgs::msg::OccupancyGrid& map, int start, int goal)
{
  const int width = map.info.width;
  const int height = map.info.height;
  const int count = width * height;
  const double infinity = std::numeric_limits<double>::infinity();
  std::vector<double> distance(count, infinity);
  std::vector<bool> visited(count, false);
  distance[start] = 0.0;

  auto blocked = [&map, width, height](int x, int y) {
    return x < 0 || y < 0 || x >= width || y >= height || map.data[y * width + x] >= 99;
  };

  for (int pass = 0; pass < count; ++pass) {
    int current = -1;
    for (int i = 0; i < count; ++i) {
      if (!visited[i] && (current == -1 || distance[i] < distance[current])) {
        current = i;
      }
    }
    if (current == -1 || !std::isfinite(distance[current])) {
      break;
    }
    if (current == goal) {
      return distance[current];
    }
    visited[current] = true;
    const int x = current % width;
    const int y = current / width;
    for (int ny = y - 1; ny <= y + 1; ++ny) {
      for (int nx = x - 1; nx <= x + 1; ++nx) {
        if ((nx == x && ny == y) || blocked(nx, ny)) {
          continue;
        }
        const bool diagonal = nx != x && ny != y;
        if (diagonal && (blocked(nx, y) || blocked(x, ny))) {
          continue;
        }
        const int next = ny * width + nx;
        distance[next] = std::min(
          distance[next], distance[current] + map.info.resolution * (diagonal ? std::sqrt(2.0) : 1.0));
      }
    }
  }
  return infinity;
}

void expectShortestRoute(const nav_msgs::msg::OccupancyGrid& map, int start, int goal)
{
  auto planner = makeShortestPlanner();
  const auto point = [&map](int cell) {
    return std::pair<double, double>{
      map.info.origin.position.x + (cell % map.info.width + 0.5) * map.info.resolution,
      map.info.origin.position.y + (cell / map.info.width + 0.5) * map.info.resolution};
  };
  const auto [sx, sy] = point(start);
  const auto [tx, ty] = point(goal);

  nav_msgs::msg::Path path;
  double gx = 0.0;
  double gy = 0.0;
  const bool found = planner.plan(map, sx, sy, tx, ty, path, gx, gy);
  const double optimum = referenceShortestDistance(map, start, goal);
  ASSERT_EQ(found, std::isfinite(optimum));
  if (!found) {
    EXPECT_TRUE(path.poses.empty());
    return;
  }

  double length = 0.0;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto& a = path.poses[i - 1].pose.position;
    const auto& b = path.poses[i].pose.position;
    length += std::hypot(b.x - a.x, b.y - a.y);
  }
  EXPECT_NEAR(length, optimum, 1e-8);
  EXPECT_NEAR(planner.remainingPathCost(map, path, sx, sy) * map.info.resolution, optimum, 1e-8);
  EXPECT_TRUE(planner.remainingPathIsValid(map, path, sx, sy));
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
  EXPECT_NEAR(path.poses.front().pose.position.x, -1.75, 1e-9);
  EXPECT_NEAR(path.poses.back().pose.position.x, 2.25, 1e-9);
  EXPECT_NEAR(gx, 2.25, 1e-9);
  EXPECT_NEAR(gy, 0.25, 1e-9);
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
    EXPECT_LT(cellAtWorld(map, p.x, p.y), 99) << "path touches a wall cell";
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

TEST(PlannerTest, ShortestDistanceIgnoresCostsWhenWeightIsZero)
{
  auto map = makeMap();
  for (int cx = 0; cx < kSize; ++cx) {
    setCell(map, cx, 10, 49);
  }
  expectShortestRoute(map, 10 * kSize + 4, 10 * kSize + 16);
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

  EXPECT_LT(cellAtWorld(map, gx, gy), 99);
  EXPECT_NEAR(std::hypot(gx - 2.75, gy - 0.25), 1.0, 1e-9);  // two cells away
  EXPECT_NEAR(path.poses.back().pose.position.x, gx, 1e-9);
  EXPECT_NEAR(path.poses.back().pose.position.y, gy, 1e-9);
}

TEST(PlannerTest, EscapesInflationBandAroundStart)
{
  auto planner = makePlanner();
  auto map = makeMap();
  // The robot at cell (4, 10) is boxed in by a 2-cell ring of inflation (99)
  // with a real wall (100) directly below it.
  for (int cy = 8; cy <= 12; ++cy) {
    for (int cx = 2; cx <= 6; ++cx) {
      setCell(map, cx, cy, 99);
    }
  }
  setCell(map, 4, 8, 100);
  setCell(map, 4, 9, 100);
  nav_msgs::msg::Path path;
  double gx = 0.0;
  double gy = 0.0;

  ASSERT_TRUE(planner.plan(map, -2.75, 0.25, 3.0, 0.0, path, gx, gy));

  // It leaves the band, never through the wall, and never re-enters it.
  bool left_band = false;
  for (const auto& pose : path.poses) {
    const auto& p = pose.pose.position;
    const int8_t cost = cellAtWorld(map, p.x, p.y);
    EXPECT_NE(cost, 100) << "path crosses the wall";
    if (cost < 99) {
      left_band = true;
    } else {
      EXPECT_FALSE(left_band) << "path re-entered the band";
    }
  }
  EXPECT_TRUE(left_band);
}

TEST(PlannerTest, LethalBandFarFromStartIsStillAWall)
{
  auto planner = makePlanner();
  auto map = makeMap();
  for (int cy = 0; cy < kSize; ++cy) {
    setCell(map, 10, cy, 99);
  }
  nav_msgs::msg::Path path;
  double gx = 0.0;
  double gy = 0.0;

  // Start is 6 cells from the band, twice the escape radius.
  EXPECT_FALSE(planner.plan(map, -3.0, 0.0, 3.0, 0.0, path, gx, gy));
}

TEST(PlannerTest, RemainingPathStaysValidUntilAnObstacleLandsOnIt)
{
  auto planner = makePlanner();
  auto map = makeMap();
  nav_msgs::msg::Path path;
  double gx = 0.0;
  double gy = 0.0;
  ASSERT_TRUE(planner.plan(map, -3.0, 0.0, 3.0, 0.0, path, gx, gy));
  EXPECT_TRUE(planner.remainingPathIsValid(map, path, -3.0, 0.0));

  // Cost changes off the path don't matter.
  setCell(map, 10, 2, 100);
  EXPECT_TRUE(planner.remainingPathIsValid(map, path, -3.0, 0.0));

  // Inflation right next to the robot is tolerated, it can drive out of it.
  setCell(map, 5, 10, 99);
  EXPECT_TRUE(planner.remainingPathIsValid(map, path, -3.0, 0.0));

  // A wall further along is not.
  setCell(map, 12, 10, 100);
  EXPECT_FALSE(planner.remainingPathIsValid(map, path, -3.0, 0.0));
}

TEST(PlannerTest, IgnoresObstaclesBehindTheRobot)
{
  auto planner = makePlanner();
  auto map = makeMap();
  nav_msgs::msg::Path path;
  double gx = 0.0;
  double gy = 0.0;
  ASSERT_TRUE(planner.plan(map, -2.0, 0.0, 2.0, 0.0, path, gx, gy));

  // The robot is past cell (6, 10) already.
  setCell(map, 6, 10, 100);
  EXPECT_TRUE(planner.remainingPathIsValid(map, path, 1.25, 0.25));
}

TEST(PlannerTest, InvalidatesDiagonalWhenAdjacentCellBlocksCorner)
{
  auto planner = makePlanner();
  auto map = makeMap();
  nav_msgs::msg::Path path;
  double gx = 0.0;
  double gy = 0.0;
  ASSERT_TRUE(planner.plan(map, 0.25, 0.25, 1.25, 1.25, path, gx, gy));

  setCell(map, 11, 10, 100);
  EXPECT_FALSE(planner.remainingPathIsValid(map, path, 0.25, 0.25));
}

TEST(PlannerTest, RemainingCostChargesTheHopBackOntoThePath)
{
  auto planner = makePlanner();
  auto map = makeMap();
  nav_msgs::msg::Path path;
  double gx = 0.0;
  double gy = 0.0;
  ASSERT_TRUE(planner.plan(map, -2.0, 0.0, 2.0, 0.0, path, gx, gy));

  // On the path, halfway along: four cells to go.
  EXPECT_NEAR(planner.remainingPathCost(map, path, 0.25, 0.25), 4.0, 1e-9);
  // One row off the path: one extra cell to get back on it.
  EXPECT_NEAR(planner.remainingPathCost(map, path, 0.25, 0.75), 5.0, 1e-9);
  // Blocked ahead: not worth anything.
  setCell(map, 12, 10, 100);
  EXPECT_TRUE(std::isinf(planner.remainingPathCost(map, path, 0.25, 0.25)));
}

TEST(PlannerTest, MatchesDijkstraForEveryThreeByThreeObstacleArrangement)
{
  auto map = makeMap();
  map.info.width = map.info.height = 3;
  for (int mask = 0; mask < 128; ++mask) {
    SCOPED_TRACE(mask);
    map.data.assign(9, 0);
    for (int cell = 1; cell < 8; ++cell) {
      map.data[cell] = (mask & (1 << (cell - 1))) ? 100 : 0;
    }
    expectShortestRoute(map, 0, 8);
  }
}

TEST(PlannerTest, MatchesDijkstraOnSeededMapsWithSoftCostsAndUnknowns)
{
  std::mt19937 generator(20260915);
  for (int trial = 0; trial < 200; ++trial) {
    SCOPED_TRACE(trial);
    auto map = makeMap();
    for (auto& cell : map.data) {
      const int draw = generator() % 100;
      cell = draw < 20 ? 100 : (draw < 40 ? -1 : static_cast<int8_t>(generator() % 50));
    }
    const int start = generator() % 400;
    const int goal = generator() % 400;
    map.data[start] = map.data[goal] = 0;
    expectShortestRoute(map, start, goal);
  }
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
