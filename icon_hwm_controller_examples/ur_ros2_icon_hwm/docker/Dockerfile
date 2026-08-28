FROM icon_hwm_base:latest

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
 && apt-get install -y \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-${ROS_DISTRO}-ur-controllers \
    ros-${ROS_DISTRO}-ur-description \
    ros-${ROS_DISTRO}-ur-robot-driver \
 && rm -rf /var/lib/apt/lists/*

COPY . ${AMENT_WORKSPACE_DIR}/src/ur_ros2_icon_hwm

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update \
 && rosdep update \
 && rosdep install -iy \
    --from-path $(colcon list --packages-up-to ur_ros2_icon_hwm --paths-only) \
    --rosdistro ${ROS_DISTRO} \
 && colcon build --base-paths ${AMENT_WORKSPACE_DIR} \
    --packages-up-to ur_ros2_icon_hwm \
 && rm -rf /var/lib/apt/lists/*   

ENV DEBIAN_FRONTEND=dialog
