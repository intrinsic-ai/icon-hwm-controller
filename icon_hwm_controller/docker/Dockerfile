ARG ROS_BASE_IMAGE=ros:kilted-ros-base

FROM ${ROS_BASE_IMAGE} AS base

ARG AMENT_WORKSPACE_DIR=/ament_ws

SHELL ["/bin/bash", "-c"]

# Make the workspace directory available inside derived images.
ENV AMENT_WORKSPACE_DIR=${AMENT_WORKSPACE_DIR}

WORKDIR ${AMENT_WORKSPACE_DIR}

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
 && apt-get install -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-filelock \
    python3-rosdep \
    ros-${ROS_DISTRO}-backward-ros \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-tl-expected \
    ros-${ROS_DISTRO}-xacro \
 && rm -rf /var/lib/apt/lists/*

# Suppress error if rosdep init was already run in the base image.
RUN rosdep init || true

# Copy the relevant source code inside the container and build it.
COPY icon_hwm_controller \
     ${AMENT_WORKSPACE_DIR}/src/icon_hwm_controller
COPY icon_hwm_controller_msgs \
     ${AMENT_WORKSPACE_DIR}/src/icon_hwm_controller_msgs
COPY icon_shared_memory_vendor \
     ${AMENT_WORKSPACE_DIR}/src/icon_shared_memory_vendor

RUN git clone --branch main --single-branch --depth 1 \
    https://github.com/intrinsic-ai/sdk-ros.git \
    ${AMENT_WORKSPACE_DIR}/src/sdk-ros

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update \
 && rosdep update \
 && rosdep install -iy \
    --from-path $(colcon list --packages-up-to icon_hwm_controller --paths-only) \
    --rosdistro ${ROS_DISTRO} \
 && colcon build --base-paths ${AMENT_WORKSPACE_DIR} \
    --packages-up-to icon_hwm_controller \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=dialog
