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
// scratch, so it only ever describes what the laser can see right now.
//
// Cell values: -1 unknown, 0 free, 100 a laser hit, 99 anywhere the robot's
// body would overlap an obstacle (the lethal disc), and a decaying cost
// beyond that so planners keep their distance without being forbidden.
class CostmapCore {
  public:
    explicit CostmapCore(const rclcpp::Logger& logger);

    // Sizes the grid and builds the inflation kernel. Must be called once
    // before processScan. Radii are in metres; decay is the exponential
    // falloff rate applied past the lethal radius.
    void configure(
      double resolution,
      int width,
      int height,
      double lethal_radius,
      double inflation_radius,
      double decay);

    void processScan(const sensor_msgs::msg::LaserScan& scan);

    const nav_msgs::msg::OccupancyGrid& grid() const { return grid_; }

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

    void buildInflationKernel();
    void inflate(const std::vector<std::pair<int, int>>& obstacles);

    nav_msgs::msg::OccupancyGrid grid_;
    std::vector<KernelCell> kernel_;
    double lethal_radius_;
    double inflation_radius_;
    double decay_;
    rclcpp::Logger logger_;
};

}  // namespace robot

#endif  // COSTMAP_CORE_HPP_
