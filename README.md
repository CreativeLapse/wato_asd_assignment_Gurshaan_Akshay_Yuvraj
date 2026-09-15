# WATonomous ASD Admissions Assignment

Team: Gurshaan, Akshay, Yuvraj

A differential-drive robot in Gazebo drives itself to any point clicked in
Foxglove while avoiding the walls, boxes and cylinders in the arena. The
software is four ROS 2 Humble nodes that pass data down a pipeline:

```
/lidar ──▶ costmap ──▶ /local_obstacles ──▶ map_memory ──▶ /map ──▶ planner ──▶ /path ──▶ control ──▶ /cmd_vel
```

Map memory, planning, and control also receive `/odom/filtered` from
`odometry_spoof`. The local `/costmap` remains available for visualization;
the planner uses the globally inflated `/map`.

| Node | Package | What it does |
|---|---|---|
| **costmap** | `src/robot/costmap` | Turns each laser scan into a 48 m × 48 m grid centred on the sensor (0.4 m cells). Ray tracing produces raw free/occupied observations on `/local_obstacles`; `/costmap` adds a local inflation band for visualization. |
| **map_memory** | `src/robot/map_memory` | Stitches raw observations into a fixed 30 m × 30 m world map (0.5 m cells), interpolating odometry at each scan's timestamp. Updates after 1.5 m of travel, a 0.2 rad turn, or 0.5 s, including while stopped. Remembers obstacles and rebuilds their inflation globally, so occluded obstacles keep their clearance. Newer known observations overwrite older ones; unknown never erases. |
| **planner** | `src/robot/planner` | Shortest-distance A* on the 8-neighbour world grid (`cost_weight: 0.0`), with a Euclidean heuristic and no diagonal corner cutting. The 3.4 m inflation radius blocks about 1.7 m around obstacle cells at cost 50, measured from the wheel axle. Searches again on map changes and every 0.5 s while moving; replaces invalid or longer remaining routes and retains equal optima to avoid unnecessary switching. Unknown cells are drivable; blocked goals snap to open space. A temporary planning failure stops the robot and retains the goal for retry, with a 180 s overall deadline. |
| **control** | `src/robot/control` | Pure pursuit at the wheel axle: converts the lidar odometry using its 1.3 m forward offset and follows a point 1 m ahead. Cruises at 0.8 m/s with a 20 Hz control loop, reduces speed to preserve tight turns within the 1 rad/s turning limit, spins when off-heading, and slows within 1.6 m of arrival. Planner and controller use the same axle position for goal completion. |
| tf_throttle | `src/robot/tf_throttle` | Gazebo streams poses at 1 kHz; this republishes the newest transform per frame at 20 Hz on `/tf` so Foxglove stays responsive. |
| odometry_spoof | `src/robot/odometry_spoof` | Provided by WATonomous: derives `/odom/filtered` from the simulator's TF. |

The shortest-path guarantee is for distance between the resolved start and goal
cells on the current 0.5 m grid, subject to its blocked cells and corner rules.
Tests compare A* against an independent Dijkstra search on all 128 obstacle
arrangements of a 3 × 3 grid with fixed open endpoints and 200 seeded larger maps.
Several routes can tie for shortest distance. Unseen obstacles, continuous paths
between grid directions, and driving time are outside this grid-distance guarantee.

Each of the four main packages splits into a `*_core` library (pure algorithm,
no ROS calls beyond logging) and a `*_node` executable (subscriptions,
publishers, timers, parameters). Every tunable lives in the package's
`config/params.yaml`.

## Running it

Requires Docker Engine (Linux) or Docker Desktop (macOS / WSL). On Apple
Silicon, create `watod-config.local.sh` next to `watod-config.sh` containing
`PLATFORM="arm64"`.

```bash
./watod build          # build the robot, gazebo and foxglove images
./watod up             # start everything
./watod down           # stop and remove the containers
```

Then open [Foxglove](https://app.foxglove.dev), connect to
`ws://localhost:<FOXGLOVE_BRIDGE_PORT>` (the port is printed by `./watod up`
and stored in `modules/.env`), and import the layout at
`config/wato_asd_training_foxglove_config .json`. Use the **Publish point**
tool in the 3D panel to click a goal; the robot plans a path and drives to it.

## Running the unit tests

The core libraries and ROS node interactions have gtest suites, including
regressions for route stability, retrying a saved goal, axle-based steering,
and matching scans to odometry. The robot image build runs these tests.
Inside the robot container:

```bash
./watod -t robot            # open a shell in the robot container
colcon build && colcon test && colcon test-result --verbose
```

## Repository layout

Everything outside `src/robot/{costmap,map_memory,planner,control,tf_throttle}`
and `src/robot/bringup_robot/launch` is the unmodified WATonomous training
monorepo: `watod` and `watod-config.sh` drive Docker Compose, `docker/` holds
the Dockerfiles, `modules/` the compose files, `src/gazebo` the simulator
world, and `src/samples` the reference pub/sub examples.

## Original setup notes

The assignment is supported on Linux Ubuntu >= 22.04, Windows (WSL), and
macOS. You can set up an
[Ubuntu Virtual Machine](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview),
[WSL](https://learn.microsoft.com/en-us/windows/wsl/install), or
[dual boot](https://opensource.com/article/18/5/dual-boot-linux), then
[install Docker Engine](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository).

Assignment page: https://wiki.watonomous.ca/
