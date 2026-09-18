ARG BASE_IMAGE=ghcr.io/watonomous/robot_base/base:humble-ubuntu22.04

# Refresh the ROS signing key bundled in the older base image.
FROM ${BASE_IMAGE} AS ros_base
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros2-latest-archive-keyring.gpg

################################ Source ################################
FROM ros_base AS source

# The ROS apt key baked into the base image expired in 2025; refresh it so apt sees the current index
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor --yes -o /usr/share/keyrings/ros2-latest-archive-keyring.gpg

WORKDIR ${AMENT_WS}/src

# Copy in source code 
COPY src/robot/odometry_spoof odometry_spoof
COPY src/robot/costmap costmap
COPY src/robot/map_memory map_memory
COPY src/robot/planner planner
COPY src/robot/control control
COPY src/robot/tf_throttle tf_throttle
COPY src/robot/bringup_robot bringup_robot

# Scan for rosdeps
RUN apt-get -qq update && rosdep update && \
    rosdep install --from-paths . --ignore-src -r -s \
        | (grep 'apt-get install' || true) \
        | awk '{print $3}' \
        | sort  > /tmp/colcon_install_list

################################# Dependencies ################################
FROM ros_base AS dependencies

# The ROS apt key baked into the base image expired in 2025; refresh it so apt sees the current index
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor --yes -o /usr/share/keyrings/ros2-latest-archive-keyring.gpg

# ADD MORE DEPENDENCIES HERE

# Install Rosdep requirements
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt-fast install -qq -y --no-install-recommends $(cat /tmp/colcon_install_list)

# Copy in source code from source stage
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src

# Dependency Cleanup
WORKDIR /
RUN apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

################################ Build ################################
FROM dependencies AS build

# Build ROS2 packages. One package at a time with two compilers: the node
# test targets are heavy and a full parallel build runs a 4 GB Docker VM
# out of memory.
WORKDIR ${AMENT_WS}
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    MAKEFLAGS=-j2 colcon build --executor sequential \
        --cmake-args -DCMAKE_BUILD_TYPE=Release --install-base ${WATONOMOUS_INSTALL}

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon test --install-base ${WATONOMOUS_INSTALL} --event-handlers console_direct+ && \
    colcon test-result --verbose

# Source and Build Artifact Cleanup 
RUN rm -rf src/* build/* devel/* install/* log/*

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]
