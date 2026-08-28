FROM icon_hwm_base:latest

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
 && apt-get install -y \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
 && rm -rf /var/lib/apt/lists/*

COPY . ${AMENT_WORKSPACE_DIR}/src/fanuc_ros2_icon_hwm

# The tag v2.3.0 is targetting ROS 2 Jazzy.
RUN git clone --branch v2.3.0 --depth 1 \
    https://github.com/FANUC-CORPORATION/fanuc_description.git \
    ${AMENT_WORKSPACE_DIR}/src/fanuc_description \
 && git clone --branch v2.3.0 --depth 1 \
    --recurse-submodules --shallow-submodules \
    https://github.com/FANUC-CORPORATION/fanuc_driver.git \
    ${AMENT_WORKSPACE_DIR}/src/fanuc_driver \
 # The driver is currently released for ROS 2 Jazzy only while the ICON HWM is
 # targetting Kilted. We apply patches to remove the version check in
 # fanuc_hardware_interface and fix version-related changes in
 # fanuc_controllers.
 # For the migration notes see https://control.ros.org/jazzy/doc/ros2_controllers/doc/migration.html.
 && git -C ${AMENT_WORKSPACE_DIR}/src/fanuc_driver apply \
    ${AMENT_WORKSPACE_DIR}/src/fanuc_ros2_icon_hwm/docker/0001-remove-fanuc-hardware-interface-version-check.patch \
 && git -C ${AMENT_WORKSPACE_DIR}/src/fanuc_driver apply \
    ${AMENT_WORKSPACE_DIR}/src/fanuc_ros2_icon_hwm/docker/0002-fix-fanuc-controllers-kilted-compatibility.patch \
 && source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update \
 && rosdep update \
 && rosdep install -iy \
    --from-path $(colcon list --packages-up-to fanuc_ros2_icon_hwm --paths-only) \
    --rosdistro ${ROS_DISTRO} \
 && colcon build --base-paths ${AMENT_WORKSPACE_DIR} \
    --packages-up-to fanuc_ros2_icon_hwm \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && rm -rf /var/lib/apt/lists/*   

ENV DEBIAN_FRONTEND=dialog

