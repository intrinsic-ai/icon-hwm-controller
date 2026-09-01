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
5. [Combining Scene Object and Service into a Hardware Device](#combining-scene-object-and-service-into-a-hardware-device)
6. [Deployment and Flowstate Configuration](#deployment-and-flowstate-configuration)
   - [Option A: Deploying as a Hardware Device (Recommended)](#option-a-deploying-as-a-hardware-device-recommended)
   - [Option B: Deploying as a Standalone Service](#option-b-deploying-as-a-standalone-service)
   - [Configuring the Realtime Control Service](#configuring-the-realtime-control-service)
7. [Error Handling and UR Safety Recovery Sequence](#error-handling-and-ur-safety-recovery-sequence)

---

## Prerequisites and Robot Controller Setup

Before running the driver against a physical UR controller or URSim:

1. **Host Prerequisites**:
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
* **Position & Velocity Limits**: Obtained from the Universal Robots user manual or PolyScope safety configuration screens.
  > [!IMPORTANT]
  > PolyScope manuals specify joint positions in degrees ($\pm 360^\circ$) and joint velocities in degrees/second (e.g. $180^\circ/\text{s}$ or $360^\circ/\text{s}$). Always convert them to **radians** ($\text{rad} = \text{deg} \times \frac{\pi}{180}$) and **radians/second** ($\text{rad}/\text{s}$) for the SDF model.
* **Acceleration & Jerk Limits**: Configured based on your process requirements and safety plane configurations (typically $15.0\,\text{rad}/\text{s}^2$ acceleration and $1000.0\,\text{rad}/\text{s}^3$ jerk).
* **Inverse Kinematics Solver**: All Universal Robots 6-DOF arms use the dedicated `ur` kinematics solver:
  ```xml
  <intrinsic:ik_solver>ur</intrinsic:ik_solver>
  ```

---

## Combining Scene Object and Service into a Hardware Device

In the Intrinsic platform, an **`intrinsic_hardware_device`** packages:
1. An **`intrinsic_scene_object`**: The robot kinematics model (SDF) with UR-specific inverse kinematics solver (`<intrinsic:ik_solver>ur</intrinsic:ik_solver>`), collision/visual geometries, and joint limits.
2. An **`intrinsic_service`**: The real-time ROS 2 driver service (`ur_ros2_icon_hwm_service`).

### 1. Starlark `BUILD` Definition

```python
load("@ai_intrinsic_sdks//intrinsic/scene/build_defs:sdf_scene_object.bzl", "sdf_scene_object")
load("@ai_intrinsic_sdks//intrinsic/assets/scene_objects/build_defs:scene_object.bzl", "intrinsic_scene_object")
load("@ai_intrinsic_sdks//intrinsic/assets/services/build_defs:services.bzl", "intrinsic_service")
load("@ai_intrinsic_sdks//intrinsic/assets/hardware_devices/build_defs:hardware_device.bzl", "intrinsic_hardware_device")

package(default_visibility = ["//visibility:public"])

# Step A: Kinematics and Mesh Definition (SDF Model)
sdf_scene_object(
    name = "ur5e_sdf",
    src = "models/ur5e.sdf",
    sdf_assets = glob(["models/meshes/**"]),
)

# Step B: Scene Object Asset
# (Pre-configured scene object models can also be imported from the Intrinsic Open Core (IOC) repository)
intrinsic_scene_object(
    name = "ur5e_scene_object",
    manifest = "proto/ur5e_scene_object_manifest.textproto",
    scene_object = ":ur5e_sdf",
)

# Step C: Combined Hardware Device Asset
intrinsic_hardware_device(
    name = "ur5e_hardware_device",
    assets = [
        ":ur5e_scene_object",
        ":ur_ros2_icon_hwm_service",
    ],
    manifest = "proto/ur5e_hardware_device_manifest.textproto",
)
```

---

### 2. Hardware Device Manifest (`proto/ur5e_hardware_device_manifest.textproto`)

```textproto
# proto-file: intrinsic/assets/hardware_devices/proto/v1/hardware_device_manifest.proto
# proto-message: intrinsic_proto.hardware_devices.v1.HardwareDeviceManifest

metadata {
  id {
    package: "ai.intrinsic"
    name: "ur5e_hardware_device"
  }
  vendor {
    display_name: "Intrinsic"
  }
  documentation {
    description: "Universal Robots UR5e Hardware Device combining kinematics model and ROS 2 HWM service."
  }
  display_name: "Universal Robots UR5e Hardware Device"
}

graph {
  nodes {
    key: "scene_object"
    value {
      asset: "ai.intrinsic.ur5e_scene_object"
    }
  }
  nodes {
    key: "service"
    value {
      asset: "ai.intrinsic.ur_ros2_icon_hwm"
    }
  }
}
```

---

### 3. Default Configuration Protobuf ([`proto/ur_ros2_icon_hwm_default_config.textproto`](proto/ur_ros2_icon_hwm_default_config.textproto))

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
        value: "ur_hwm"
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

## Deployment and Flowstate Configuration

### Option A: Deploying as a Hardware Device (Recommended)

Packaging and installing the robot as an `intrinsic_hardware_device` simultaneously adds the robot kinematics into the 3D scene and connects the ROS 2 driver service.

1. **Build the Hardware Device Asset**:
   ```bash
   # Building from the local workspace or the Intrinsic Open Core (IOC) repository:
   bazel build //icon_hwm_controller_examples/ur_ros2_icon_hwm:ur5e_hardware_device
   ```

2. **Install the Asset into your Flowstate Solution**:
   ```bash
   inctl asset install bazel-bin/icon_hwm_controller_examples/ur_ros2_icon_hwm/ur5e_hardware_device.bundle.tar \
     --org=<org>@<project> --cluster=<cluster>
   ```

3. **Add the Hardware Device Instance**:
   Instantiate the hardware device into the running solution under the name `robot`:
   ```bash
   inctl service add ai.intrinsic.ur5e_hardware_device --name=robot \
     --org=<org>@<project> --cluster=<cluster>
   ```

---

### Option B: Deploying as a Standalone Service

If you already have a pre-existing Scene Object in your solution and only need the ROS 2 communication service:

1. **Build and Install the Service**:
   ```bash
   bazel build //icon_hwm_controller_examples/ur_ros2_icon_hwm:ur_ros2_icon_hwm_service

   inctl asset install bazel-bin/icon_hwm_controller_examples/ur_ros2_icon_hwm/ur_ros2_icon_hwm_service.bundle.tar \
     --org=<org>@<project> --cluster=<cluster>
   ```

2. **Add the Service Instance**:
   ```bash
   inctl service add ai.intrinsic.ur_ros2_icon_hwm --name="ur_hwm" \
     --org=<org>@<project> --cluster=<cluster>
   ```

---

### Configuring the Realtime Control Service

In Flowstate, navigate to **Services -> Realtime Control Service -> Manage Configuration** and configure the `IconMainConfig`:

```protobuf
control_frequency_hz: 500.0
hardware_module_names: ["robot"]
hardware_module_that_drives_clock: "robot"

realtime_control_config {
  parts_by_name {
    key: "arm"
    value {
      part_type_name: "HalArmPart"
      safety_action_type_name: "intrinsic.stop"
      hardware_resource_name: "robot"
      config {
        [type.googleapis.com/intrinsic_proto.icon.HalArmPartConfig] {
          joint_position_command { module_name: "robot" interface_name: "joint_position_command" }
          joint_position_state   { module_name: "robot" interface_name: "joint_position_state" }
          joint_velocity_state   { module_name: "robot" interface_name: "joint_velocity_state" }
        }
      }
    }
  }
}
```

*(Note: When deploying as a standalone service with `--name="ur_hwm"`, replace `"robot"` with `"ur_hwm"` in the `module_name` fields above.)*

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
