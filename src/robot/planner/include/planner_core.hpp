#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot
{

// A cell in the occupancy grid.
struct CellIndex
{
  int x;
  int y;

  CellIndex() : x(0), y(0) {}
  CellIndex(int xx, int yy) : x(xx), y(yy) {}

  bool operator==(const CellIndex& other) const { return x == other.x && y == other.y; }
  bool operator!=(const CellIndex& other) const { return !(*this == other); }
};

// Lets CellIndex be a key in unordered containers.
struct CellIndexHash
{
  std::size_t operator()(const CellIndex& idx) const
  {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

// An entry in the A* open set: the cell and its f = g + h score.
struct AStarNode
{
  CellIndex index;
  double f_score;

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// Orders AStarNodes so the priority queue pops the lowest f_score first.
struct CompareF
{
  bool operator()(const AStarNode& a, const AStarNode& b) const
  {
    return a.f_score > b.f_score;
  }
};

// A* path planner over a nav_msgs OccupancyGrid.
//
// Cells at or above the lethal cost are walls. Everything else can be walked
// on. With the default cost_weight of zero, A* minimizes geometric distance
// on the 8-neighbour grid. An optional nonzero weight adds clearance costs.
// Unknown cells are traversable; optimality applies to the supplied map.
class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    // lethal_cost: cells >= this are impassable.
    // unknown_cost: cost assumed for cells that are -1 in the grid.
    // cost_weight: how strongly a cell's cost lengthens a step through it.
    //   A step costs distance * (1 + cost_weight * cost / 100).
    // snap_radius: how many cells to search for a free cell when the start
    //   or goal lands on an obstacle.
    void configure(int lethal_cost, int unknown_cost, double cost_weight, int snap_radius);

    // Plans from (start_x, start_y) to (goal_x, goal_y), all in the map frame.
    // On success fills `path` with cell-centre waypoints and reports the goal
    // actually planned to (it moves if the requested goal was blocked).
    bool plan(
      const nav_msgs::msg::OccupancyGrid& map,
      double start_x,
      double start_y,
      double goal_x,
      double goal_y,
      nav_msgs::msg::Path& path,
      double& planned_goal_x,
      double& planned_goal_y);

    // Validate only the untravelled portion of an existing route.
    bool remainingPathIsValid(
      const nav_msgs::msg::OccupancyGrid& map,
      const nav_msgs::msg::Path& path, double robot_x, double robot_y);

    // Cost from the same resolved start cell used by plan(). Infinity if
    // the old route does not contain that cell or its suffix is invalid.
    // This lets callers retain equal optima without ignoring shortcuts.
    double remainingPathCost(
      const nav_msgs::msg::OccupancyGrid& map,
      const nav_msgs::msg::Path& path, double robot_x, double robot_y);

  private:
    bool inBounds(const CellIndex& cell) const;
    bool worldToCell(double wx, double wy, CellIndex& cell) const;
    void cellToWorld(const CellIndex& cell, double& wx, double& wy) const;

    // Cost used for planning: unknown cells map to unknown_cost_.
    int cellCost(const CellIndex& cell) const;
    bool isBlocked(const CellIndex& cell) const;

    // Finds the closest cell to `from` that is not blocked. Returns false if
    // none exists within snap_radius_ cells.
    bool nearestOpenCell(const CellIndex& from, CellIndex& out) const;

    bool aStar(const CellIndex& start, const CellIndex& goal, std::vector<CellIndex>& cells) const;
    double heuristic(const CellIndex& a, const CellIndex& b) const;

    nav_msgs::msg::OccupancyGrid map_;
    int lethal_cost_;
    int unknown_cost_;
    double cost_weight_;
    int snap_radius_;
    rclcpp::Logger logger_;
};

}  // namespace robot

#endif  // PLANNER_CORE_HPP_
