#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <cstdint>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot
{

struct CellIndex
{
  int x;
  int y;

  CellIndex() : x(0), y(0) {}
  CellIndex(int xx, int yy) : x(xx), y(yy) {}

  bool operator==(const CellIndex& other) const { return x == other.x && y == other.y; }
  bool operator!=(const CellIndex& other) const { return !(*this == other); }
};

// An entry in the A* open set: the cell and its f = g + h score.
struct AStarNode
{
  CellIndex index;
  double f_score;

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

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
// on, but higher costs make a cell more expensive so paths keep their
// distance from obstacles. Unknown cells are treated as a fixed cost so the
// robot is willing to drive into space it hasn't seen yet.
//
// The robot itself may be sitting inside an inflation band (it parks close
// to walls), so lethal cells near the start are passable at a high price
// as long as they aren't a real obstacle (cost 100). That lets the search
// walk out of the band by the cheapest route without ever crossing a wall.
class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    // lethal_cost: cells >= this are impassable.
    // unknown_cost: cost assumed for cells that are -1 in the grid.
    // cost_weight: how strongly a cell's cost lengthens a step through it.
    //   A step costs distance * (1 + cost_weight * cost / 100).
    // snap_radius: how many cells to search for a free cell when the goal
    //   lands on an obstacle.
    // escape_radius: cells around the start where lethal (but not
    //   obstacle) cells may still be crossed.
    void configure(int lethal_cost, int unknown_cost, double cost_weight, int snap_radius, int escape_radius);

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

    // True if no waypoint beyond the escape radius of the robot has become
    // lethal in this map. Cheap enough to run on every map update.
    bool pathIsClear(
      const nav_msgs::msg::OccupancyGrid& map,
      const nav_msgs::msg::Path& path,
      double robot_x,
      double robot_y);

  private:
    bool inBounds(const CellIndex& cell) const;
    bool worldToCell(double wx, double wy, CellIndex& cell) const;
    void cellToWorld(const CellIndex& cell, double& wx, double& wy) const;
    int toIndex(const CellIndex& cell) const;
    CellIndex fromIndex(int index) const;

    int cellCost(const CellIndex& cell) const;
    bool isBlocked(const CellIndex& cell) const;
    bool nearStart(const CellIndex& cell) const;
    bool canEnter(const CellIndex& cell) const;

    bool nearestOpenCell(const CellIndex& from, CellIndex& out) const;
    bool aStar(const CellIndex& start, const CellIndex& goal, std::vector<CellIndex>& cells) const;
    double heuristic(const CellIndex& a, const CellIndex& b) const;

    nav_msgs::msg::OccupancyGrid map_;
    CellIndex start_;
    int lethal_cost_;
    int unknown_cost_;
    double cost_weight_;
    int snap_radius_;
    int escape_radius_;
    rclcpp::Logger logger_;
};

}  // namespace robot

#endif  // PLANNER_CORE_HPP_
