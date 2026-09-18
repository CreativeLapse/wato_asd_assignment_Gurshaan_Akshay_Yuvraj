#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

#include "planner_core.hpp"

namespace robot
{

namespace
{
constexpr int kObstacle = 100;
}  // namespace

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
  : lethal_cost_(99),
    unknown_cost_(10),
    cost_weight_(5.0),
    snap_radius_(20),
    escape_radius_(6),
    logger_(logger) {}

void PlannerCore::configure(int lethal_cost, int unknown_cost, double cost_weight, int snap_radius, int escape_radius)
{
  lethal_cost_ = lethal_cost;
  unknown_cost_ = unknown_cost;
  cost_weight_ = cost_weight;
  snap_radius_ = snap_radius;
  escape_radius_ = escape_radius;
}

bool PlannerCore::plan(
  const nav_msgs::msg::OccupancyGrid& map,
  double start_x,
  double start_y,
  double goal_x,
  double goal_y,
  nav_msgs::msg::Path& path,
  double& planned_goal_x,
  double& planned_goal_y)
{
  map_ = map;
  path.poses.clear();
  path.header = map_.header;

  if (!worldToCell(start_x, start_y, start_)) {
    RCLCPP_WARN(logger_, "Start (%.2f, %.2f) is off the map", start_x, start_y);
    return false;
  }
  CellIndex goal;
  if (!worldToCell(goal_x, goal_y, goal)) {
    RCLCPP_WARN(logger_, "Goal (%.2f, %.2f) is off the map", goal_x, goal_y);
    return false;
  }

  if (isBlocked(goal)) {
    const CellIndex requested = goal;
    if (!nearestOpenCell(goal, goal)) {
      RCLCPP_WARN(logger_, "No open cell near the goal; cannot plan");
      return false;
    }
    RCLCPP_INFO(logger_, "Goal cell (%d, %d) is blocked; planning to (%d, %d) instead",
                requested.x, requested.y, goal.x, goal.y);
  }
  cellToWorld(goal, planned_goal_x, planned_goal_y);

  std::vector<CellIndex> cells;
  if (!aStar(start_, goal, cells)) {
    RCLCPP_WARN(logger_, "A* found no path from (%d, %d) to (%d, %d)",
                start_.x, start_.y, goal.x, goal.y);
    return false;
  }

  path.poses.reserve(cells.size());
  for (const CellIndex& cell : cells) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = map_.header;
    cellToWorld(cell, pose.pose.position.x, pose.pose.position.y);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return true;
}

bool PlannerCore::pathIsClear(
  const nav_msgs::msg::OccupancyGrid& map,
  const nav_msgs::msg::Path& path,
  double robot_x,
  double robot_y)
{
  map_ = map;
  if (!worldToCell(robot_x, robot_y, start_)) {
    return false;
  }

  for (const auto& pose : path.poses) {
    CellIndex cell;
    if (!worldToCell(pose.pose.position.x, pose.pose.position.y, cell)) {
      return false;
    }
    if (!canEnter(cell)) {
      return false;
    }
  }
  return true;
}

bool PlannerCore::aStar(const CellIndex& start, const CellIndex& goal, std::vector<CellIndex>& cells) const
{
  const size_t cell_count = static_cast<size_t>(map_.info.width) * map_.info.height;
  std::vector<double> g_score(cell_count, std::numeric_limits<double>::infinity());
  std::vector<int> came_from(cell_count, -1);
  std::vector<char> closed(cell_count, 0);
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;

  g_score[toIndex(start)] = 0.0;
  open.emplace(start, heuristic(start, goal));

  while (!open.empty()) {
    const CellIndex current = open.top().index;
    open.pop();

    // A cell can sit in the queue more than once with stale scores; only the
    // first (cheapest) pop matters.
    const int current_index = toIndex(current);
    if (closed[current_index]) {
      continue;
    }
    closed[current_index] = 1;

    if (current == goal) {
      cells.clear();
      for (int i = current_index; i != -1; i = came_from[i]) {
        cells.push_back(fromIndex(i));
      }
      std::reverse(cells.begin(), cells.end());
      return true;
    }

    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        const CellIndex next(current.x + dx, current.y + dy);
        if (!inBounds(next) || !canEnter(next)) {
          continue;
        }
        const int next_index = toIndex(next);
        if (closed[next_index]) {
          continue;
        }

        // Don't let a diagonal step squeeze between two blocked cells.
        const bool diagonal = (dx != 0 && dy != 0);
        if (diagonal &&
            (!canEnter(CellIndex(current.x + dx, current.y)) ||
             !canEnter(CellIndex(current.x, current.y + dy)))) {
          continue;
        }

        const double distance = diagonal ? std::sqrt(2.0) : 1.0;
        const double step = distance * (1.0 + cost_weight_ * cellCost(next) / 100.0);
        const double tentative_g = g_score[current_index] + step;
        if (tentative_g >= g_score[next_index]) {
          continue;
        }

        g_score[next_index] = tentative_g;
        came_from[next_index] = current_index;
        open.emplace(next, tentative_g + heuristic(next, goal));
      }
    }
  }

  return false;
}

double PlannerCore::heuristic(const CellIndex& a, const CellIndex& b) const
{
  // Straight-line distance in cells. Every step costs at least its length,
  // so this never overestimates and A* stays optimal.
  return std::hypot(a.x - b.x, a.y - b.y);
}

bool PlannerCore::nearestOpenCell(const CellIndex& from, CellIndex& out) const
{
  // Callers may pass the same variable as `from` and `out`, so take a copy
  // before `out` gets written.
  const CellIndex centre = from;

  bool found = false;
  double best = std::numeric_limits<double>::infinity();
  for (int dy = -snap_radius_; dy <= snap_radius_; ++dy) {
    for (int dx = -snap_radius_; dx <= snap_radius_; ++dx) {
      const CellIndex candidate(centre.x + dx, centre.y + dy);
      if (!inBounds(candidate) || isBlocked(candidate)) {
        continue;
      }
      const double distance = std::hypot(dx, dy);
      if (distance < best) {
        best = distance;
        out = candidate;
        found = true;
      }
    }
  }
  return found;
}

int PlannerCore::cellCost(const CellIndex& cell) const
{
  if (!inBounds(cell)) {
    return kObstacle;
  }
  const int8_t raw = map_.data[static_cast<size_t>(cell.y) * map_.info.width + cell.x];
  return raw < 0 ? unknown_cost_ : raw;
}

bool PlannerCore::isBlocked(const CellIndex& cell) const
{
  return cellCost(cell) >= lethal_cost_;
}

bool PlannerCore::nearStart(const CellIndex& cell) const
{
  return std::hypot(cell.x - start_.x, cell.y - start_.y) <= escape_radius_;
}

bool PlannerCore::canEnter(const CellIndex& cell) const
{
  const int cost = cellCost(cell);
  if (cost < lethal_cost_) {
    return true;
  }
  return cost < kObstacle && nearStart(cell);
}

bool PlannerCore::inBounds(const CellIndex& cell) const
{
  return cell.x >= 0 && cell.y >= 0 &&
         cell.x < static_cast<int>(map_.info.width) &&
         cell.y < static_cast<int>(map_.info.height);
}

bool PlannerCore::worldToCell(double wx, double wy, CellIndex& cell) const
{
  const double res = map_.info.resolution;
  cell.x = static_cast<int>(std::floor((wx - map_.info.origin.position.x) / res));
  cell.y = static_cast<int>(std::floor((wy - map_.info.origin.position.y) / res));
  return inBounds(cell);
}

void PlannerCore::cellToWorld(const CellIndex& cell, double& wx, double& wy) const
{
  const double res = map_.info.resolution;
  wx = map_.info.origin.position.x + (cell.x + 0.5) * res;
  wy = map_.info.origin.position.y + (cell.y + 0.5) * res;
}

int PlannerCore::toIndex(const CellIndex& cell) const
{
  return cell.y * static_cast<int>(map_.info.width) + cell.x;
}

CellIndex PlannerCore::fromIndex(int index) const
{
  const int width = static_cast<int>(map_.info.width);
  return CellIndex(index % width, index / width);
}

}  // namespace robot
