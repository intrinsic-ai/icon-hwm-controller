FROM icon_hwm_base:latest

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
 && apt-get install -y \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
 && rm -rf /var/lib/apt/lists/*

COPY . ${AMENT_WORKSPACE_DIR}/src/fanuc_ros2_icon_hwm

# TODO: Pin the commit to 250d3fa4713bf680d57e887330cf498ca06696f7.
RUN git clone https://github.com/FANUC-CORPORATION/fanuc_description.git ${AMENT_WORKSPACE_DIR}/src/fanuc_description \
 # TODO: Pin the commit to 5898135bb9eeae8fedbb2e923dbb707c949411f6.
 && git clone --branch main --single-branch --recurse-submodules https://github.com/FANUC-CORPORATION/fanuc_driver.git ${AMENT_WORKSPACE_DIR}/src/fanuc_driver \
 # The driver is currently released for ROS 2 Jazzy only while the ICON HWM is targetting Kilted.
 # We apply a patch to remove the version check in the fanuc_hardware_interface.
 && git -C ${AMENT_WORKSPACE_DIR}/src/fanuc_driver apply ${AMENT_WORKSPACE_DIR}/src/fanuc_ros2_icon_hwm/docker/0001-remove-fanuc-hardware-interface-version-check.patch \
 && source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update \
 && rosdep update \
 && rosdep install -iy --from-path $(colcon list --packages-up-to fanuc_ros2_icon_hwm --paths-only) --rosdistro ${ROS_DISTRO} \
 && colcon build --base-paths ${AMENT_WORKSPACE_DIR} --packages-up-to fanuc_ros2_icon_hwm \
 && rm -rf /var/lib/apt/lists/*   

ENV DEBIAN_FRONTEND=dialog

