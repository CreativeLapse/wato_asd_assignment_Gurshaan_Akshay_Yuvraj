# WATonomous ASD Admissions Assignment

Team: Gurshaan, Akshay, Yuvraj

A differential-drive robot in Gazebo drives itself to any point clicked in
Foxglove while avoiding the walls, boxes and cylinders in the arena. The
software is four ROS 2 Humble nodes that pass data down a pipeline:

```
/lidar ──▶ costmap ──▶ /local_obstacles ──▶ map_memory ──▶ /map ──▶ planner ──▶ /path ──▶ control ──▶ /cmd_vel
                 └──▶ /costmap (inflated, for viewing)
```

Map memory, the planner and the controller also take `/odom/filtered` from
`odometry_spoof`. Odometry reports the lidar, which sits 1.3 m ahead of the
wheel axle; the planner and controller convert it to the axle, since that is
the point that actually follows the path.

| Node | Package | What it does |
|---|---|---|
| **costmap** | `src/robot/costmap` | Turns each laser scan into a 40 m × 40 m grid centred on the sensor (0.2 m cells). Beams are ray-traced to mark free space and hits become obstacles. The raw grid goes to map memory on `/local_obstacles`; `/costmap` is the same grid with a 1.7 m lethal disc around every hit and a cost that decays out to 3.2 m, for viewing. |
| **map_memory** | `src/robot/map_memory` | Stitches raw observations into a fixed 32 m × 32 m world map (0.25 m cells). A scan waits for the odometry sample after it, then is placed at the pose interpolated to its own timestamp. It fuses after 1 m of travel, 0.3 rad of turning or 1 s, including while stopped. Newer known values overwrite older ones; unknown never erases. The published map is rebuilt from the remembered hits with the same lethal disc and decay, so an obstacle that has dropped out of view keeps its margin. |
| **planner** | `src/robot/planner` | A* over the world map: 8-connected, Euclidean heuristic, no corner cutting, and a step cost that rises with cell cost so paths hug open space. Unknown cells are drivable. A goal on an obstacle snaps to the nearest open cell. If the robot is parked inside an inflation band it may cross it (never a real wall) to get out. It searches again on every map change and every 0.5 s as the robot moves, but only publishes when the current path is blocked or the new one is strictly cheaper, so equal routes never flip. A failed plan stops the robot and is retried on the next map; a goal is dropped only after 4 min. |
| **control** | `src/robot/control` | Pure pursuit at the axle: chases the path point 1.2–2.5 m ahead (growing with speed) along a circular arc. Speed ramps up to 0.8 m/s, drops for tight arcs, never asks for more than the 1 rad/s turn limit so the arc is kept rather than widened, and eases off near the goal. Spins in place when the target is far off-heading, with hysteresis so it doesn't chatter. Stops on arrival, when the path is cleared, or if odometry goes quiet. |
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

The core libraries have gtest suites, and each node has a test that drives
it over real topics: route stability, retrying a kept goal, axle-based
steering, and matching scans to odometry. A* is cross-checked against an
independent Dijkstra on every 3 × 3 obstacle arrangement and 200 seeded maps.
The robot image build runs all of them. Inside the robot container:

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
