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

## How it works

The costmap node (`src/robot/costmap`) turns each laser scan into a 40 m
square grid of 0.2 m cells centred on the robot. Every beam is ray-traced to
mark free space, with extra rays between beams that have spread more than a
cell apart, and each hit becomes an obstacle. Around each hit it draws a
1 m lethal disc, sized to the robot's body, and lets the cost decay out to
2.5 m. That way the planner prefers open space but can still get close to a
wall when it has to.

Map memory (`src/robot/map_memory`) stitches those costmaps into a fixed
32 m square world map at 0.25 m. It fuses a new costmap once the robot has
travelled 1 m, turned 0.5 rad or waited 2 s, and skips scans taken while
spinning fast. Each costmap is placed using the robot's TF pose at the scan's
own timestamp, so obstacles land where they were seen even mid-turn. Newer
readings overwrite older ones, and an unknown cell never erases a known one.

The planner (`src/robot/planner`) runs A* over the world map: 8-connected,
Euclidean heuristic, no cutting corners, and a step cost that rises with cell
cost. Unknown cells count as drivable. A goal clicked on an obstacle snaps to
the nearest open cell. If the robot is parked inside an inflation band it may
cross the band to get out, though never a real wall. A path is kept until the
map blocks it or 5 s pass. A failed plan is retried when the next map
arrives, and a goal is only dropped after 4 min.

Control (`src/robot/control`) is pure pursuit. It chases the path point 1.2
to 2.5 m ahead, further at higher speed, along a circular arc. Speed ramps up
to 0.8 m/s, drops for tight arcs and near the goal, and the robot spins in
place when the target is far off its heading, with some hysteresis so it
doesn't flip between spinning and driving. It stops on arrival, when the path
is cleared, or if odometry goes quiet.

Two smaller nodes support these. `tf_throttle` republishes Gazebo's 1 kHz
pose stream at 20 Hz on `/tf` so Foxglove stays responsive, and
`odometry_spoof`, provided by WATonomous, derives `/odom/filtered` from the
simulator's TF.

Each of the four main packages is split into a `*_core` library holding the
algorithm, with no ROS calls beyond logging, and a `*_node` executable that
owns the subscriptions, publishers, timers and parameters. Every tunable is in
the package's `config/params.yaml`.

## Running it

You need Docker Engine on Linux or Docker Desktop on macOS or WSL. On Apple
Silicon, create `watod-config.local.sh` next to `watod-config.sh` containing
`PLATFORM="arm64"` and `ACTIVE_MODULES="robot gazebo vis_tools"`.

```bash
./watod build          # build the robot, gazebo and foxglove images
./watod up             # start everything
./watod down           # stop and remove the containers
```

Then open [Foxglove](https://app.foxglove.dev) and connect to
`ws://localhost:<FOXGLOVE_BRIDGE_PORT>`. The port is printed by `./watod up`
and stored in `modules/.env`. Import the layout at
`config/wato_asd_training_foxglove_config .json`, pick the Publish point tool
in the 3D panel and click a goal. The robot plans a path and drives to it.

If the hosted Foxglove app won't open the 3D panel, run a local viewer
instead. [Lichtblick](https://github.com/lichtblick-suite/lichtblick) is the
open-source fork of Foxglove Studio and needs no account:

```bash
docker run -d --name wato_viewer --restart unless-stopped -p 8080:8080 \
  ghcr.io/lichtblick-suite/lichtblick:latest
open "http://localhost:8080/?ds=foxglove-websocket&ds.url=ws%3A%2F%2Flocalhost%3A10020"
```

Import `config/wato_path_finding_layout.json` from Layouts, then Import from
file. It shows the world map, the planned path, the clicked goal, the robot
pose and the planner and control logs side by side.

## Running the unit tests

The core libraries have gtest suites, 38 tests across the four packages, and
the robot image build runs them. To run them yourself inside the robot
container:

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
