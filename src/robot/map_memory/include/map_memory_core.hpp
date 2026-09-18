#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <cstdint>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

// Accumulates robot-centred local observations into one fixed global map.
//
// The map remembers raw observations: a cell keeps its last known value
// (free or hit) until a newer scan says otherwise, and unknown never erases
// what was already learned. The published map is rebuilt from those
// observations after every fusion, with a lethal disc (99) around every
// remembered hit and a cost that decays with distance beyond it. Inflating
// here rather than in the local costmap means an obstacle that has dropped
// out of view keeps its margin.
class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    // Allocates the global grid. (origin_x, origin_y) is the world position
    // of the grid's bottom-left corner. Radii are in metres; decay is the
    // exponential falloff rate applied past the lethal radius.
    void configure(
      double resolution,
      int width,
      int height,
      double origin_x,
      double origin_y,
      double lethal_radius,
      double inflation_radius,
      double decay);

    // Stamps one local grid onto the observations and rebuilds the map. The
    // pose is where the local frame sits in the global frame: the sensor's
    // position and its heading in radians.
    void fuse(
      const nav_msgs::msg::OccupancyGrid& local,
      double robot_x,
      double robot_y,
      double robot_yaw);

    const nav_msgs::msg::OccupancyGrid& map() const { return map_; }

    int8_t cellCost(int cx, int cy) const;

  private:
    struct KernelCell {
      int dx;
      int dy;
      int8_t cost;
    };

    bool inBounds(int cx, int cy) const;

    // Reads the local grid at a point given in the local frame.
    // Returns -1 when the point falls outside the grid or is unknown.
    static int8_t sampleLocal(const nav_msgs::msg::OccupancyGrid& local, double lx, double ly);

    void buildInflationKernel(double lethal_radius, double inflation_radius, double decay);
    void inflate();

    nav_msgs::msg::OccupancyGrid map_;
    std::vector<int8_t> observations_;
    std::vector<KernelCell> kernel_;
    rclcpp::Logger logger_;
};

}  // namespace robot

#endif  // MAP_MEMORY_CORE_HPP_
