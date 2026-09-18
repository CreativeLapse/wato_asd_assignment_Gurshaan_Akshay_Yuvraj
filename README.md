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
| **costmap** | `src/robot/costmap` | Turns each laser scan into a 40 m × 40 m grid centred on the robot (0.2 m cells). Beams are ray-traced to mark free space and hits become obstacles. Every hit is wrapped in a 1 m lethal disc sized to the robot's body, and cost decays exponentially out to 2.5 m so paths prefer open space without being forbidden near walls. |
| **map_memory** | `src/robot/map_memory` | Stitches costmaps into a fixed 32 m × 32 m world map (0.25 m cells). It fuses a costmap after 1 m of travel, 0.5 rad of turning or 2 s, whichever comes first, skipping scans taken mid-spin. Each costmap is placed with the robot's TF pose at the scan's timestamp, so obstacles land where they were seen. Newer known values overwrite older ones; unknown never erases. |
| **planner** | `src/robot/planner` | A* over the world map: 8-connected, Euclidean heuristic, no corner cutting, and a step cost that rises with cell cost so paths hug open space. Unknown cells are drivable. A goal on an obstacle snaps to the nearest open cell. If the robot is parked inside an inflation band it may cross it (never a real wall) to get out. A path is kept until the map blocks it or 5 s pass; a failed plan is retried on the next map, and a goal is dropped only after 4 min. |
| **control** | `src/robot/control` | Pure pursuit: chases the path point 1.2–2.5 m ahead (growing with speed) along a circular arc. Speed ramps up to 0.8 m/s, drops for tight arcs and near the goal, and the robot spins in place when the target is far off-heading, with hysteresis so it doesn't chatter. Stops on arrival, when the path is cleared, or if odometry goes quiet. |
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

If the hosted Foxglove app will not open the 3D panel, run a local viewer
instead ([Lichtblick](https://github.com/lichtblick-suite/lichtblick), the
open-source fork of Foxglove Studio, needs no account):

```bash
docker run -d --name wato_viewer --restart unless-stopped -p 8080:8080 \
  ghcr.io/lichtblick-suite/lichtblick:latest
open "http://localhost:8080/?ds=foxglove-websocket&ds.url=ws%3A%2F%2Flocalhost%3A10020"
```

Then import `config/wato_path_finding_layout.json` (Layouts ▸ Import from
file). It shows the world map, the planned path, the clicked goal, the robot
pose and the planner/control log side by side.

## Running the unit tests

The core libraries have gtest suites (37 tests across the four packages).
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
