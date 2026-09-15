#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <cstdint>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

// Turns a single laser scan into a robot-centred occupancy grid.
//
// The grid lives in the laser's own frame: the robot sits at the centre and
// +x points along the laser's zero angle. Every scan rebuilds the grid from
// scratch, so it only ever describes what the laser can see right now. Cells
// are -1 (unknown), 0 (free), 100 (obstacle) or an inflation cost in between.
class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    // Sizes the grid. Must be called once before processScan.
    // The origin is chosen so the robot is in the middle of the grid.
    void configure(double resolution, int width, int height, double inflation_radius);

    // Rebuilds the grid from one scan: free space along every beam, an
    // obstacle at every hit, then an inflation band around each obstacle.
    void processScan(const sensor_msgs::msg::LaserScan& scan);

    const nav_msgs::msg::OccupancyGrid& grid() const { return grid_; }
    const nav_msgs::msg::OccupancyGrid& obstacleGrid() const { return obstacle_grid_; }

    int8_t cellCost(int cx, int cy) const;

  private:
    struct KernelCell {
      int dx;
      int dy;
      int8_t cost;
    };

    bool inBounds(int cx, int cy) const;
    void worldToCell(double x, double y, int& cx, int& cy) const;
    int8_t& at(int cx, int cy);

    // Marks unknown cells along the segment (x0,y0)->(x1,y1) as free. The end
    // cell itself is left untouched so an obstacle can still be placed there.
    void traceFree(int x0, int y0, int x1, int y1);

    // Precomputes the cost falloff disc that gets stamped on every obstacle.
    void buildInflationKernel();
    void inflate(const std::vector<std::pair<int, int>>& obstacles);

    nav_msgs::msg::OccupancyGrid grid_;
    nav_msgs::msg::OccupancyGrid obstacle_grid_;
    std::vector<KernelCell> kernel_;
    double inflation_radius_;
    rclcpp::Logger logger_;
};

}  // namespace robot

#endif  // COSTMAP_CORE_HPP_
