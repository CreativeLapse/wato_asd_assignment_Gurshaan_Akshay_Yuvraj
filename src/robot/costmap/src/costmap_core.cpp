#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "costmap_core.hpp"

namespace robot
{

namespace
{
constexpr int8_t kUnknown = -1;
constexpr int8_t kFree = 0;
constexpr int8_t kOccupied = 100;
}  // namespace

CostmapCore::CostmapCore(const rclcpp::Logger& logger)
  : inflation_radius_(0.0), logger_(logger) {}

void CostmapCore::configure(double resolution, int width, int height, double inflation_radius)
{
  grid_.info.resolution = resolution;
  grid_.info.width = width;
  grid_.info.height = height;
  // Centre the robot: the origin is the world position of cell (0, 0)'s corner.
  grid_.info.origin.position.x = -0.5 * width * resolution;
  grid_.info.origin.position.y = -0.5 * height * resolution;
  grid_.info.origin.position.z = 0.0;
  grid_.info.origin.orientation.w = 1.0;
  grid_.data.assign(static_cast<size_t>(width) * height, kUnknown);
  obstacle_grid_ = grid_;

  inflation_radius_ = inflation_radius;
  buildInflationKernel();

  RCLCPP_INFO(logger_, "Costmap configured: %dx%d cells at %.2f m, inflation %.2f m",
              width, height, resolution, inflation_radius);
}

void CostmapCore::buildInflationKernel()
{
  kernel_.clear();
  const double res = grid_.info.resolution;
  const int reach = static_cast<int>(std::ceil(inflation_radius_ / res));

  for (int dy = -reach; dy <= reach; ++dy) {
    for (int dx = -reach; dx <= reach; ++dx) {
      const double dist = std::hypot(dx, dy) * res;
      if (dist >= inflation_radius_) {
        continue;
      }
      // Linear falloff: 100 on the obstacle, 0 at the edge of the radius.
      const int cost = static_cast<int>(std::lround(kOccupied * (1.0 - dist / inflation_radius_)));
      if (cost > 0) {
        kernel_.push_back({dx, dy, static_cast<int8_t>(cost)});
      }
    }
  }
}

void CostmapCore::processScan(const sensor_msgs::msg::LaserScan& scan)
{
  std::fill(grid_.data.begin(), grid_.data.end(), kUnknown);

  int robot_cx = 0;
  int robot_cy = 0;
  worldToCell(0.0, 0.0, robot_cx, robot_cy);

  std::vector<std::pair<int, int>> obstacles;
  obstacles.reserve(scan.ranges.size());

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const double angle = scan.angle_min + i * scan.angle_increment;
    double range = scan.ranges[i];

    if (std::isnan(range) || range < scan.range_min) {
      continue;
    }

    // A beam that never hit anything still tells us the cells up to the
    // sensor's reach are free; it just contributes no obstacle.
    const bool hit = std::isfinite(range) && range <= scan.range_max;
    if (!hit) {
      range = scan.range_max;
    }

    int end_cx = 0;
    int end_cy = 0;
    worldToCell(range * std::cos(angle), range * std::sin(angle), end_cx, end_cy);
    traceFree(robot_cx, robot_cy, end_cx, end_cy);

    if (hit && inBounds(end_cx, end_cy)) {
      at(end_cx, end_cy) = kOccupied;
      obstacles.emplace_back(end_cx, end_cy);
    }
  }

  // Mapping must remember observations, not the temporary inflation band
  // around just the obstacles visible in this particular scan.
  obstacle_grid_ = grid_;
  inflate(obstacles);
}

void CostmapCore::traceFree(int x0, int y0, int x1, int y1)
{
  // Bresenham's line walk. The ray starts at the robot, which is inside the
  // grid, so the first out-of-bounds cell means it has left for good.
  const int dx = std::abs(x1 - x0);
  const int dy = -std::abs(y1 - y0);
  const int sx = (x0 < x1) ? 1 : -1;
  const int sy = (y0 < y1) ? 1 : -1;
  int err = dx + dy;

  int x = x0;
  int y = y0;
  while (x != x1 || y != y1) {
    if (!inBounds(x, y)) {
      return;
    }
    int8_t& cell = at(x, y);
    if (cell == kUnknown) {
      cell = kFree;
    }

    const int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y += sy;
    }
  }
}

void CostmapCore::inflate(const std::vector<std::pair<int, int>>& obstacles)
{
  for (const auto& [ox, oy] : obstacles) {
    for (const KernelCell& k : kernel_) {
      const int cx = ox + k.dx;
      const int cy = oy + k.dy;
      if (!inBounds(cx, cy)) {
        continue;
      }
      int8_t& cell = at(cx, cy);
      cell = std::max(cell, k.cost);
    }
  }
}

bool CostmapCore::inBounds(int cx, int cy) const
{
  return cx >= 0 && cy >= 0 &&
         cx < static_cast<int>(grid_.info.width) &&
         cy < static_cast<int>(grid_.info.height);
}

void CostmapCore::worldToCell(double x, double y, int& cx, int& cy) const
{
  cx = static_cast<int>(std::floor((x - grid_.info.origin.position.x) / grid_.info.resolution));
  cy = static_cast<int>(std::floor((y - grid_.info.origin.position.y) / grid_.info.resolution));
}

int8_t& CostmapCore::at(int cx, int cy)
{
  return grid_.data[static_cast<size_t>(cy) * grid_.info.width + cx];
}

int8_t CostmapCore::cellCost(int cx, int cy) const
{
  if (!inBounds(cx, cy)) {
    return kUnknown;
  }
  return grid_.data[static_cast<size_t>(cy) * grid_.info.width + cx];
}

}  // namespace robot
