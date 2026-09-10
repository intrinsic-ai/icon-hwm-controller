# Copyright 2026 Intrinsic Innovation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

FROM icon_hwm_base:latest

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
 && apt-get install -y \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
 && rm -rf /var/lib/apt/lists/*

COPY . ${AMENT_WORKSPACE_DIR}/src/khi_ros2_icon_hwm

RUN git clone -b main https://github.com/Kawasaki-Robotics/khi_ros2.git \
    ${AMENT_WORKSPACE_DIR}/src/khi_ros2 \
    # The following commit is targetting ROS 2 Jazzy but also compiles on
    # Kilted.
 && git -C ${AMENT_WORKSPACE_DIR}/src/khi_ros2 checkout \
    32a34658cd80343bfe7b7fb82cabce3bfd2b4381 \
 && git -C ${AMENT_WORKSPACE_DIR}/src/khi_ros2 apply \
    ${AMENT_WORKSPACE_DIR}/src/khi_ros2_icon_hwm/patches/0001-remove-robot-controller-emergency-and-protective-stop-checks.patch \
 && source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update \
 && rosdep update \
 && rosdep install -iy \
    --from-path $(colcon list --packages-up-to khi_ros2_icon_hwm --paths-only) \
    --rosdistro ${ROS_DISTRO} \
 && colcon build --base-paths ${AMENT_WORKSPACE_DIR} \
    --packages-up-to khi_ros2_icon_hwm \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && rm -rf /var/lib/apt/lists/*   

ENV DEBIAN_FRONTEND=dialog

