#include <algorithm>
#include <cmath>
#include <limits>

#include "map_memory_core.hpp"

namespace robot
{

namespace
{
constexpr int8_t kUnknown = -1;
constexpr int8_t kLethal = 99;
constexpr int8_t kOccupied = 100;
}  // namespace

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

void MapMemoryCore::configure(
  double resolution,
  int width,
  int height,
  double origin_x,
  double origin_y,
  double lethal_radius,
  double inflation_radius,
  double decay)
{
  map_.info.resolution = resolution;
  map_.info.width = width;
  map_.info.height = height;
  map_.info.origin.position.x = origin_x;
  map_.info.origin.position.y = origin_y;
  map_.info.origin.position.z = 0.0;
  map_.info.origin.orientation.w = 1.0;
  map_.data.assign(static_cast<size_t>(width) * height, kUnknown);
  observations_ = map_.data;
  buildInflationKernel(lethal_radius, inflation_radius, decay);

  RCLCPP_INFO(logger_, "Global map configured: %dx%d cells at %.2f m, origin (%.1f, %.1f), lethal %.2f m",
              width, height, resolution, origin_x, origin_y, lethal_radius);
}

void MapMemoryCore::buildInflationKernel(double lethal_radius, double inflation_radius, double decay)
{
  kernel_.clear();
  const double res = map_.info.resolution;
  inflation_radius = std::max(inflation_radius, lethal_radius);
  const int reach = static_cast<int>(std::ceil(inflation_radius / res));

  for (int dy = -reach; dy <= reach; ++dy) {
    for (int dx = -reach; dx <= reach; ++dx) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      const double dist = std::hypot(dx, dy) * res;
      if (dist > inflation_radius) {
        continue;
      }

      int cost = kLethal;
      if (dist > lethal_radius) {
        cost = static_cast<int>(std::lround((kLethal - 1) * std::exp(-decay * (dist - lethal_radius))));
      }
      if (cost > 0) {
        kernel_.push_back({dx, dy, static_cast<int8_t>(cost)});
      }
    }
  }
}

void MapMemoryCore::fuse(
  const nav_msgs::msg::OccupancyGrid& local,
  double robot_x,
  double robot_y,
  double robot_yaw)
{
  const double res = map_.info.resolution;
  const double origin_x = map_.info.origin.position.x;
  const double origin_y = map_.info.origin.position.y;
  const int width = static_cast<int>(map_.info.width);
  const int height = static_cast<int>(map_.info.height);

  const double cos_yaw = std::cos(robot_yaw);
  const double sin_yaw = std::sin(robot_yaw);

  // Find which global cells the (rotated) local window can touch, so we only
  // visit those. Corners of the local window, expressed in the global frame:
  const double lx0 = local.info.origin.position.x;
  const double ly0 = local.info.origin.position.y;
  const double lx1 = lx0 + local.info.width * local.info.resolution;
  const double ly1 = ly0 + local.info.height * local.info.resolution;

  double min_x = std::numeric_limits<double>::infinity();
  double min_y = min_x;
  double max_x = -min_x;
  double max_y = -min_x;
  for (const double lx : {lx0, lx1}) {
    for (const double ly : {ly0, ly1}) {
      const double wx = robot_x + cos_yaw * lx - sin_yaw * ly;
      const double wy = robot_y + sin_yaw * lx + cos_yaw * ly;
      min_x = std::min(min_x, wx);
      max_x = std::max(max_x, wx);
      min_y = std::min(min_y, wy);
      max_y = std::max(max_y, wy);
    }
  }

  const int gx_begin = std::max(0, static_cast<int>(std::floor((min_x - origin_x) / res)));
  const int gx_end = std::min(width - 1, static_cast<int>(std::floor((max_x - origin_x) / res)));
  const int gy_begin = std::max(0, static_cast<int>(std::floor((min_y - origin_y) / res)));
  const int gy_end = std::min(height - 1, static_cast<int>(std::floor((max_y - origin_y) / res)));

  // Walking the *global* cells and looking up the local grid for each one
  // guarantees every global cell under the window is considered exactly once,
  // with no holes from rotation or from the two grids having different sizes.
  // Each global cell is probed at four interior points and takes the highest
  // known value, so a thin obstacle can't slip between samples.
  constexpr double kProbeOffsets[] = {0.25, 0.75};

  int updated = 0;
  for (int gy = gy_begin; gy <= gy_end; ++gy) {
    for (int gx = gx_begin; gx <= gx_end; ++gx) {
      int8_t best = kUnknown;
      for (const double oy : kProbeOffsets) {
        for (const double ox : kProbeOffsets) {
          const double wx = origin_x + (gx + ox) * res;
          const double wy = origin_y + (gy + oy) * res;

          // Global frame -> local frame (inverse of the robot pose).
          const double dx = wx - robot_x;
          const double dy = wy - robot_y;
          const double lx = cos_yaw * dx + sin_yaw * dy;
          const double ly = -sin_yaw * dx + cos_yaw * dy;

          best = std::max(best, sampleLocal(local, lx, ly));
        }
      }

      if (best != kUnknown) {
        observations_[static_cast<size_t>(gy) * width + gx] = best;
        ++updated;
      }
    }
  }

  inflate();

  RCLCPP_DEBUG(logger_, "Fused local grid at (%.2f, %.2f, %.2f rad): %d cells updated",
               robot_x, robot_y, robot_yaw, updated);
}

void MapMemoryCore::inflate()
{
  // Rebuild from scratch so a cleared obstacle also loses its margin.
  map_.data = observations_;
  const int width = static_cast<int>(map_.info.width);
  const int height = static_cast<int>(map_.info.height);

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if (observations_[static_cast<size_t>(y) * width + x] != kOccupied) {
        continue;
      }
      for (const KernelCell& k : kernel_) {
        const int nx = x + k.dx;
        const int ny = y + k.dy;
        if (!inBounds(nx, ny)) {
          continue;
        }
        int8_t& cell = map_.data[static_cast<size_t>(ny) * width + nx];
        cell = std::max(cell, k.cost);
      }
    }
  }
}

int8_t MapMemoryCore::sampleLocal(const nav_msgs::msg::OccupancyGrid& local, double lx, double ly)
{
  const double res = local.info.resolution;
  const int cx = static_cast<int>(std::floor((lx - local.info.origin.position.x) / res));
  const int cy = static_cast<int>(std::floor((ly - local.info.origin.position.y) / res));

  if (cx < 0 || cy < 0 ||
      cx >= static_cast<int>(local.info.width) ||
      cy >= static_cast<int>(local.info.height)) {
    return kUnknown;
  }
  return local.data[static_cast<size_t>(cy) * local.info.width + cx];
}

bool MapMemoryCore::inBounds(int cx, int cy) const
{
  return cx >= 0 && cy >= 0 &&
         cx < static_cast<int>(map_.info.width) &&
         cy < static_cast<int>(map_.info.height);
}

int8_t MapMemoryCore::cellCost(int cx, int cy) const
{
  if (!inBounds(cx, cy)) {
    return kUnknown;
  }
  return map_.data[static_cast<size_t>(cy) * map_.info.width + cx];
}

}  // namespace robot
