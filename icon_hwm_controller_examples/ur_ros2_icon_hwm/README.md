# Universal Robots (UR) ROS 2 ICON Hardware Module

This package contains the **Universal Robots (UR) ICON Hardware Module**, packaging the official [Universal Robots ROS 2 Driver (`Universal_Robots_ROS2_Driver`)](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) into a real-time Intrinsic Service and deployable Hardware Device.

It supports all modern Universal Robots arms:
* **e-Series**: UR3e, UR5e, UR10e, UR16e
* **Next-Gen Heavy Payload**: UR20, UR30
* **CB3 Series**: UR3, UR5, UR10

---

## Table of Contents

1. [Prerequisites and Robot Controller Setup](#prerequisites-and-robot-controller-setup)
2. [Building the Docker Image](#building-the-docker-image)
3. [Building the Intrinsic Service Asset](#building-the-intrinsic-service-asset)
4. [Kinematics Model and Limit Acquisition](#kinematics-model-and-limit-acquisition)
5. [Error Handling and UR Safety Recovery Sequence](#error-handling-and-ur-safety-recovery-sequence)

---

## Prerequisites and Robot Controller Setup

Before running the driver against a physical UR controller or URSim:

1. **Host Prerequisites**: See main read-me.
   * [Docker Engine & Docker Compose](https://docs.docker.com/engine/install/)
   * [Bazelisk / Bazel](https://bazel.build/install/bazelisk)
   * [`inctl` CLI](https://flowstate.intrinsic.ai/docs/guides/build_with_code/set_up_your_development_environment/inctl_inbuild_installation/)

2. **Install the External Control URCap**:
   * Download and install the `externalcontrol` URCap on the robot's PolyScope controller (per the [Universal Robots ROS 2 Driver Setup Guide](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/setup/robot_setup.html)).
   * In the PolyScope installation settings, set the **Host IP** to the IP address of your Intrinsic IPC / Docker host and the **Host Port** to `50002`.

3. **Enable Remote Control Mode**:
   * In PolyScope, navigate to **Settings -> System -> Remote Control** and set it to **Enable**.
   * Switch the control mode selector (top right corner of the PolyScope UI) from **Local** to **Remote**.

4. **Verify Network Connectivity**:
   Ensure your host machine or Intrinsic IPC can communicate with the robot's primary IP (e.g. `192.170.10.100`) on dashboard port `29999` and RTDE port `30004`.

---

## Building the Docker Image

### 1. Build the Base Image
From the repository root, build the shared base image (`icon_hwm_base:latest`):
```bash
docker compose -f icon_hwm_controller/docker/docker-compose.yml build
```

### 2. Build the UR Image
Build the image containing `Universal_Robots_ROS2_Driver` and the UR operational state node:
```bash
docker compose -f icon_hwm_controller_examples/ur_ros2_icon_hwm/docker/docker-compose.yml build
```

### 3. Export the Image Tarball
Save the container image to `icon_hwm.tar` in the package directory so Bazel can package it as an OCI layer:
```bash
docker image save ur_ros2_icon_hwm:latest -o icon_hwm_controller_examples/ur_ros2_icon_hwm/icon_hwm.tar
```

---

## Building the Intrinsic Service Asset

Compile the Intrinsic Service bundle using Bazel:

```bash
bazel build //icon_hwm_controller_examples/ur_ros2_icon_hwm:ur_ros2_icon_hwm_service
```

The output asset bundle is created at:
`bazel-bin/icon_hwm_controller_examples/ur_ros2_icon_hwm/ur_ros2_icon_hwm_service.bundle.tar`

---

## Kinematics Model and Limit Acquisition

To create an accurate and safe SDF model for your Universal Robots manipulator:

### Limit Sources for UR Robots
* **SDF Model**: The SDF model can be based on the publicly available URDF and meshes from the [`Universal_Robots_ROS2_Description`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description).
* **Position & Velocity Limits**: Obtained from the Universal Robots user manual or PolyScope safety configuration screens (see [here](https://www.universal-robots.com/manuals/EN/HTML/SW10_11/Content/prod-usr-man/software/PolyScopeX/polyx-safety/polyx-Joint-LimitsApp.htm)).
  > [!IMPORTANT]
  > PolyScope manuals specify joint positions in degrees ($\pm 360^\circ$) and joint velocities in degrees/second (e.g. $180^\circ/\text{s}$ or $360^\circ/\text{s}$). Always convert them to **radians** ($\text{rad} = \text{deg} \times \frac{\pi}{180}$) and **radians/second** ($\text{rad}/\text{s}$) for the SDF model.
* **Acceleration & Jerk Limits**: Configured based on your process requirements and safety plane configurations (typically $15.0\,\text{rad}/\text{s}^2$ acceleration and $1000.0\,\text{rad}/\text{s}^3$ jerk).
* **Inverse Kinematics Solver**: All Universal Robots 6-DOF arms use the dedicated `ur` kinematics solver:
  ```xml
  <intrinsic:ik_solver>ur</intrinsic:ik_solver>
  ```

---

### Default Configuration Protobuf ([`proto/ur_ros2_icon_hwm_default_config.textproto`](proto/ur_ros2_icon_hwm_default_config.textproto))

The following section only contains the UR-specific launch file configuration. Please refer to the [FANUC example](../fanuc_ros2_icon_hwm/README.md) for more details on how to generate an Intrinsic hardware device and sideload it into your solution.

```textproto
# proto-file: google/protobuf/any.proto
# proto-message: google.protobuf.Any

[type.googleapis.com/intrinsic_proto.icon.HardwareModuleConfig] {
  drives_realtime_clock: true
  control_frequency_hz: 500.0
  module_config {
    [type.googleapis.com/intrinsic_proto.services.Ros2HwmConfig] {
      launch_package: "ur_ros2_icon_hwm"
      launch_file: "ur_control.launch.py"
      launch_parameters {
        key: "hwm_name"
        value: "robot"
      }
      launch_parameters {
        key: "ur_type"
        value: "ur5e"
      }
      launch_parameters {
        key: "robot_ip"
        value: "192.170.10.100"
      }
      icon_hwm_controller_config {
        lock_memory: true
        shm_namespace: ""
        realtime_priority_low: 40
        realtime_priority_high: 45
      }
    }
  }
}
```

---

## Error Handling and UR Safety Recovery Sequence

Error monitoring and recovery is managed by [`ur_operational_state_node.cpp`](src/ur_operational_state_node.cpp).

### 1. State Monitoring
The node subscribes to dashboard and controller state topics:
* `io_and_status_controller/safety_mode` (`ur_dashboard_msgs/msg/SafetyMode`):
  * Monitors `PROTECTIVE_STOP`, `SAFEGUARD_STOP`, `SYSTEM_EMERGENCY_STOP`, `ROBOT_EMERGENCY_STOP`, `VIOLATION`, `FAULT`, etc.
  * Any mode other than `NORMAL` or `REDUCED` marks the state as `FAULTED`.
* `io_and_status_controller/robot_mode` (`ur_dashboard_msgs/msg/RobotMode`):
  * `DISCONNECTED`, `NO_CONTROLLER`, `CONFIRM_SAFETY` -> `FAULTED`.
  * `BOOTING`, `POWER_OFF`, `POWER_ON`, `IDLE`, `BACKDRIVE` -> `DISABLED`.
  * `RUNNING` (with program running) -> `ENABLED`.
* `io_and_status_controller/robot_program_running` (`std_msgs/msg/Bool`).

### 2. Fault Clearing Sequence (`ClearFaults`)
When **Clear Faults** is executed:
1. **Connect Dashboard**: Connects to the UR Dashboard Server if currently disconnected.
2. **Unlock Protective Stop / Restart Safety**:
   * If in `PROTECTIVE_STOP`, calls `dashboard_client/unlock_protective_stop`.
   * If in safety violation, fault, or safeguard stop, calls `dashboard_client/restart_safety`.
3. **Dismiss Popups**: Calls `close_safety_popup` and `close_popup`.
4. **Restore Running State**:
   * Calls the `ur_robot_state_helper/set_mode` action targeting `ROBOT_MODE_RUNNING` with `play_program=true`.
5. **Headless Fallback**:
   * If `SetMode` action fails, performs brake release (`dashboard_client/brake_release`) and restarts the external control script via `io_and_status_controller/resend_robot_program`.
6. `icon_hwm_controller` re-activates the hardware component and resumes real-time control.
