#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <cstdint>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

// Accumulates robot-centred local costmaps into one fixed global map.
//
// The global map never forgets: a cell keeps its last known value until a
// newer costmap says otherwise. Unknown cells in a costmap never erase what
// was already learned.
class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    // Allocates the global grid. (origin_x, origin_y) is the world position
    // of the grid's bottom-left corner.
    void configure(double resolution, int width, int height, double origin_x, double origin_y,
                   double inflation_radius = 0.0);

    // Stamps one local costmap onto the global map. The pose is where the
    // costmap's frame sits in the global frame: the robot's position and its
    // heading in radians.
    void fuse(
      const nav_msgs::msg::OccupancyGrid& local,
      double robot_x,
      double robot_y,
      double robot_yaw);

    const nav_msgs::msg::OccupancyGrid& map() const { return map_; }

    int8_t cellCost(int cx, int cy) const;

  private:
    bool inBounds(int cx, int cy) const;

    // Reads the local costmap at a point given in the local frame.
    // Returns -1 when the point falls outside the costmap or is unknown.
    static int8_t sampleLocal(const nav_msgs::msg::OccupancyGrid& local, double lx, double ly);
    void inflateGlobalMap();

    nav_msgs::msg::OccupancyGrid map_;
    std::vector<int8_t> observations_;
    struct KernelCell { int dx; int dy; int8_t cost; };
    std::vector<KernelCell> inflation_kernel_;
    rclcpp::Logger logger_;
};

}  // namespace robot

#endif  // MAP_MEMORY_CORE_HPP_
