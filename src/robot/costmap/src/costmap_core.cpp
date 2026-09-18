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
constexpr int8_t kLethal = 99;
constexpr int8_t kOccupied = 100;
}  // namespace

CostmapCore::CostmapCore(const rclcpp::Logger& logger)
  : lethal_radius_(0.0), inflation_radius_(0.0), decay_(0.0), logger_(logger) {}

void CostmapCore::configure(
  double resolution,
  int width,
  int height,
  double lethal_radius,
  double inflation_radius,
  double decay)
{
  grid_.info.resolution = resolution;
  grid_.info.width = width;
  grid_.info.height = height;
  grid_.info.origin.position.x = -0.5 * width * resolution;
  grid_.info.origin.position.y = -0.5 * height * resolution;
  grid_.info.origin.position.z = 0.0;
  grid_.info.origin.orientation.w = 1.0;
  grid_.data.assign(static_cast<size_t>(width) * height, kUnknown);

  lethal_radius_ = lethal_radius;
  inflation_radius_ = std::max(inflation_radius, lethal_radius);
  decay_ = decay;
  buildInflationKernel();

  RCLCPP_INFO(logger_, "Costmap configured: %dx%d cells at %.2f m, lethal %.2f m, inflation %.2f m",
              width, height, resolution, lethal_radius_, inflation_radius_);
}

void CostmapCore::buildInflationKernel()
{
  kernel_.clear();
  const double res = grid_.info.resolution;
  const int reach = static_cast<int>(std::ceil(inflation_radius_ / res));

  for (int dy = -reach; dy <= reach; ++dy) {
    for (int dx = -reach; dx <= reach; ++dx) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      const double dist = std::hypot(dx, dy) * res;
      if (dist > inflation_radius_) {
        continue;
      }

      int cost = kLethal;
      if (dist > lethal_radius_) {
        cost = static_cast<int>(std::lround((kLethal - 1) * std::exp(-decay_ * (dist - lethal_radius_))));
      }
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

  // A beam that never hit anything still clears the cells out to the
  // sensor's reach; it just contributes no obstacle. Bad readings get a
  // negative range and are skipped.
  std::vector<double> ranges(scan.ranges.size(), -1.0);
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const double range = scan.ranges[i];
    if (std::isnan(range) || range < scan.range_min) {
      continue;
    }
    ranges[i] = (std::isfinite(range) && range <= scan.range_max) ? range : scan.range_max;
  }

  std::vector<std::pair<int, int>> obstacles;
  obstacles.reserve(scan.ranges.size());

  for (size_t i = 0; i < ranges.size(); ++i) {
    if (ranges[i] < 0.0) {
      continue;
    }
    const double angle = scan.angle_min + i * scan.angle_increment;
    const bool hit = ranges[i] < scan.range_max;

    int end_cx = 0;
    int end_cy = 0;
    worldToCell(ranges[i] * std::cos(angle), ranges[i] * std::sin(angle), end_cx, end_cy);
    traceFree(robot_cx, robot_cy, end_cx, end_cy, !hit);

    if (hit && inBounds(end_cx, end_cy)) {
      int8_t& cell = at(end_cx, end_cy);
      if (cell != kOccupied) {
        cell = kOccupied;
        obstacles.emplace_back(end_cx, end_cy);
      }
    }
  }

  fillBetweenBeams(scan, ranges, robot_cx, robot_cy);
  inflate(obstacles);
}

void CostmapCore::fillBetweenBeams(
  const sensor_msgs::msg::LaserScan& scan,
  const std::vector<double>& ranges,
  int robot_cx,
  int robot_cy)
{
  // Far from the robot, neighbouring beams are more than a cell apart, so
  // tracing only along the beams leaves a speckle of unknown cells between
  // them. Trace extra rays through each gap, out to the nearer of the two
  // beams so nothing behind an obstacle gets cleared.
  const double res = grid_.info.resolution;
  for (size_t i = 0; i + 1 < ranges.size(); ++i) {
    if (ranges[i] < 0.0 || ranges[i + 1] < 0.0) {
      continue;
    }
    const double range = std::min(ranges[i], ranges[i + 1]);
    const bool open = range >= scan.range_max;
    const int extra = static_cast<int>(std::ceil(scan.angle_increment * range / res)) - 1;
    for (int k = 1; k <= extra; ++k) {
      const double angle = scan.angle_min +
        (i + static_cast<double>(k) / (extra + 1)) * scan.angle_increment;
      int end_cx = 0;
      int end_cy = 0;
      worldToCell(range * std::cos(angle), range * std::sin(angle), end_cx, end_cy);
      traceFree(robot_cx, robot_cy, end_cx, end_cy, open);
    }
  }
}

void CostmapCore::traceFree(int x0, int y0, int x1, int y1, bool include_end)
{
  // Bresenham's line walk. The ray starts at the robot, which is inside the
  // grid, so the first out-of-bounds cell means it has left for good.
  if (include_end && inBounds(x1, y1)) {
    int8_t& end = at(x1, y1);
    if (end == kUnknown) {
      end = kFree;
    }
  }
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
