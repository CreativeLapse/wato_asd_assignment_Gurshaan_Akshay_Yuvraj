# WATonomous ASD Admissions Assignment

Team: Gurshaan, Akshay, Yuvraj

A differential-drive robot in Gazebo drives itself to any point clicked in
Foxglove while avoiding the walls, boxes and cylinders in the arena. The
software is four ROS 2 Humble nodes that pass data down a pipeline:

```
/lidar ──▶ costmap ──▶ /costmap ──▶ map_memory ──▶ /map ──▶ planner ──▶ /path ──▶ control ──▶ /cmd_vel
                                        ▲                      ▲                     ▲
                                        └──────── /odom/filtered (odometry_spoof) ───┘
```

| Node | Package | What it does |
|---|---|---|
| **costmap** | `src/robot/costmap` | Turns each laser scan into a 48 m × 48 m grid centred on the robot (0.4 m cells). Beams are ray-traced to mark free space, hits become obstacles, and every obstacle gets a linear inflation band. |
| **map_memory** | `src/robot/map_memory` | Stitches costmaps into a fixed 30 m × 30 m world map (0.5 m cells). It fuses a new costmap every 1.5 m of travel, walking the global cells under the rotated local window so the coarser map never has holes. Newer known values overwrite older ones; unknown never erases. |
| **planner** | `src/robot/planner` | A* over the world map: 8-connected, Euclidean heuristic, no corner cutting, and a step cost that rises with cell cost so paths hug open space. Unknown cells are drivable. A goal that lands on an obstacle snaps to the nearest open cell. Replans whenever the map changes, gives up after 60 s. |
| **control** | `src/robot/control` | Pure pursuit: chases the path point 1 m ahead along a circular arc, spins in place if the target is far off-heading, slows down near the end, and stops on arrival or when the path is cleared. |
| tf_throttle | `src/robot/tf_throttle` | Gazebo streams poses at 1 kHz; this republishes the newest transform per frame at 20 Hz on `/tf` so Foxglove stays responsive. |
| odometry_spoof | `src/robot/odometry_spoof` | Provided by WATonomous: derives `/odom/filtered` from the simulator's TF. |

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

The core libraries have gtest suites (26 tests across the four packages).
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

## Acknowledgement

The nodes in this repository were written with the help of Claude (Anthropic),
used as a pair programmer for the implementation and unit tests. All design
decisions, tuning and verification in simulation are ours.

## Original setup notes

The assignment is supported on Linux Ubuntu >= 22.04, Windows (WSL), and
macOS. You can set up an
[Ubuntu Virtual Machine](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview),
[WSL](https://learn.microsoft.com/en-us/windows/wsl/install), or
[dual boot](https://opensource.com/article/18/5/dual-boot-linux), then
[install Docker Engine](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository).

Assignment page: https://wiki.watonomous.ca/
