# FANUC ROS 2 ICON Hardware Module

This package contains the **FANUC ICON Hardware Module**, packaging the official [FANUC ROS 2 Driver (`fanuc_driver`)](https://github.com/FANUC-CORPORATION/fanuc_driver) into a real-time Intrinsic Service and deployable Hardware Device.

It supports both collaborative robots (CRX Series: CRX-5iA, CRX-10iA, CRX-10iA/L, CRX-20iA/L, CRX-25iA, CRX-30iA) and traditional industrial robots (LR Mate 200iD/7L, LR Mate 200iD/4S, LR-10iA/10, M-20iD/25, M-20iD/35, etc.).

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
7. [Error Handling and Alarm Recovery Sequence](#error-handling-and-alarm-recovery-sequence)

---

## Prerequisites and Robot Controller Setup

Before connecting Flowstate or the ROS 2 driver to your physical FANUC controller:

1. **Host Prerequisites**:
   * [Docker Engine & Docker Compose](https://docs.docker.com/engine/install/)
   * [Bazelisk / Bazel](https://bazel.build/install/bazelisk)
   * [`inctl` CLI](https://flowstate.intrinsic.ai/docs/guides/build_with_code/set_up_your_development_environment/inctl_inbuild_installation/)

2. **Install the FANUC ROS 2 Driver on the Robot Controller**:
   Follow the [Official FANUC Driver Quickstart Guide](https://fanuc-corporation.github.io/fanuc_driver_doc/main/docs/quick_start/quick_start.html#robot-controller-setup) to set up the controller correctly and configure the TCP communication port.

3. **Disable UOP Override for Flowstate / ROS 2 Compatibility**:
   If the robot controller was previously commissioned with Flowstate and its proprietary hardware module (see [FANUC Stream Motion Setup](https://flowstate.intrinsic.ai/docs/guides/commission_solution/set_up_robot/fanuc_stream_motion_setup/)), ensure the following system variable setting:
   ```text
   $OPWORK.$uop_disable = 1
   ```
   This ensures that software-level start/motion commands are permitted without external UOP hardware interlock signals.

4. **Verify Network Connectivity**:
   Ensure your host machine or Intrinsic IPC can ping the robot controller IP (e.g. `192.170.10.1`).

---

## Building the Docker Image

### 1. Build the Base Image
From the repository root, build the shared base image (`icon_hwm_base:latest`):
```bash
docker compose -f icon_hwm_controller/docker/docker-compose.yml build
```

### 2. Build the FANUC Image
Build the image containing `fanuc_driver` and the FANUC operational state node:
```bash
docker compose -f icon_hwm_controller_examples/fanuc_ros2_icon_hwm/docker/docker-compose.yml build
```

### 3. Export the Image Tarball
Save the container image to `icon_hwm.tar` in the package directory so Bazel can package it as an OCI layer:
```bash
docker image save fanuc_ros2_icon_hwm:latest -o icon_hwm_controller_examples/fanuc_ros2_icon_hwm/icon_hwm.tar
```

---

## Building the Intrinsic Service Asset

Compile the Intrinsic Service bundle using Bazel:

```bash
bazel build //icon_hwm_controller_examples/fanuc_ros2_icon_hwm:fanuc_ros2_icon_hwm_service
```

The output asset bundle is created at:
`bazel-bin/icon_hwm_controller_examples/fanuc_ros2_icon_hwm/fanuc_ros2_icon_hwm_service.bundle.tar`

---

## Kinematics Model and Limit Acquisition

To create an accurate and safe SDF model for your FANUC robot:

### Limit Sources for FANUC Robots
* **Position & Velocity Limits**: Found in the FANUC mechanical specifications datasheet or operator manual.
  > [!IMPORTANT]
  > Datasheets give values in degrees ($^\circ$) and degrees/sec ($^\circ/\text{s}$). Always convert them to **radians** ($\text{rad} = \text{deg} \times \frac{\pi}{180}$) and **radians/sec** ($\text{rad}/\text{s}$) for the SDF model.
* **Acceleration & Jerk Limits**: FANUC controllers enforce internal limits depending on payload and model series. You can query these exact values directly from the controller via Stream Motion using [`stream_motion_example.cpp`](https://github.com/FANUC-CORPORATION/fanuc_driver/tree/main/fanuc_libs) in `fanuc_libs`.
* **Inverse Kinematics Solver**: All 6-DOF FANUC industrial arms use the `spherical_wrist` solver while the CRX collaborative robots have to use the general `kinematic_chain` solver:
  ```xml
  <intrinsic:ik_solver>spherical_wrist</intrinsic:ik_solver>
  ```

---

## Combining Scene Object and Service into a Hardware Device

In the Intrinsic platform, an **`intrinsic_hardware_device`** packages:
1. An **`intrinsic_scene_object`**: The robot's kinematic chain, collision and visual meshes, joint limits, and inverse kinematics solver (`spherical_wrist`).
2. An **`intrinsic_service`**: The real-time ROS 2 driver service (`fanuc_ros2_icon_hwm_service`).

By packaging them into an `intrinsic_hardware_device`, adding the robot to a Flowstate Solution provisions both the 3D visual/kinematic representation and the communication backend in one step.

The control frequency depends on the robot model and controller generation and is either 125 or 250 Hz for the R-30iB+ and 500 or 1000 Hz for the R-50iA controller. The following configuration files assume the more recent R-50iA controller running at the default 500 Hz control frequency.

### 1. Starlark `BUILD` Definition

```python
load("@ai_intrinsic_sdks//intrinsic/scene/build_defs:sdf_scene_object.bzl", "sdf_scene_object")
load("@ai_intrinsic_sdks//intrinsic/assets/scene_objects/build_defs:scene_object.bzl", "intrinsic_scene_object")
load("@ai_intrinsic_sdks//intrinsic/assets/services/build_defs:services.bzl", "intrinsic_service")
load("@ai_intrinsic_sdks//intrinsic/assets/hardware_devices/build_defs:hardware_device.bzl", "intrinsic_hardware_device")

package(default_visibility = ["//visibility:public"])

# Step A: Kinematics and Mesh Definition (SDF Model)
sdf_scene_object(
    name = "crx20ia_l_sdf",
    src = "models/crx20ia_l.sdf",
    sdf_assets = glob(["models/meshes/**"]),
)

# Step B: Scene Object Asset
# (Pre-configured scene object models can also be imported from the Intrinsic Open Core (IOC) repository)
intrinsic_scene_object(
    name = "crx20ia_l_scene_object",
    manifest = "proto/crx20ia_l_scene_object_manifest.textproto",
    scene_object = ":crx20ia_l_sdf",
)

# Step C: Combined Hardware Device Asset
intrinsic_hardware_device(
    name = "fanuc_crx20ia_l_hardware_device",
    assets = [
        ":crx20ia_l_scene_object",
        ":fanuc_ros2_icon_hwm_service",
    ],
    manifest = "proto/fanuc_crx20ia_l_hardware_device_manifest.textproto",
)
```

---

### 2. Hardware Device Manifest (`proto/fanuc_crx20ia_l_hardware_device_manifest.textproto`)

```textproto
# proto-file: intrinsic/assets/hardware_devices/proto/v1/hardware_device_manifest.proto
# proto-message: intrinsic_proto.hardware_devices.v1.HardwareDeviceManifest

metadata {
  id {
    package: "ai.intrinsic"
    name: "fanuc_crx20ia_l_hardware_device"
  }
  vendor {
    display_name: "Intrinsic"
  }
  documentation {
    description: "FANUC CRX-20iA/L Hardware Device combining kinematics geometry and ROS 2 HWM service."
  }
  display_name: "FANUC CRX-20iA/L Hardware Device"
}

graph {
  nodes {
    key: "scene_object"
    value {
      asset: "ai.intrinsic.fanuc_crx20ia_l_scene_object"
    }
  }
  nodes {
    key: "service"
    value {
      asset: "ai.intrinsic.fanuc_ros2_icon_hwm"
    }
  }
}
```

---

### 3. Default Configuration Protobuf ([`proto/fanuc_ros2_icon_hwm_default_config.textproto`](proto/fanuc_ros2_icon_hwm_default_config.textproto))

```textproto
# proto-file: google/protobuf/any.proto
# proto-message: google.protobuf.Any

[type.googleapis.com/intrinsic_proto.icon.HardwareModuleConfig] {
  drives_realtime_clock: true
  control_frequency_hz: 500.0
  module_config {
    [type.googleapis.com/intrinsic_proto.services.Ros2HwmConfig] {
      launch_package: "fanuc_ros2_icon_hwm"
      launch_file: "fanuc_control.launch.py"
      launch_parameters {
        key: "hwm_name"
        value: "fanuc_hwm"
      }
      launch_parameters {
        key: "robot_ip"
        value: "192.170.10.1"
      }
      launch_parameters {
        key: "robot_model"
        value: "crx20ia_l"
      }
      launch_parameters {
        key: "robot_series"
        value: "crx"
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
   bazel build //icon_hwm_controller_examples/fanuc_ros2_icon_hwm:fanuc_crx20ia_l_hardware_device
   ```

2. **Install the Asset into your Flowstate Solution**:
   ```bash
   inctl asset install bazel-bin/icon_hwm_controller_examples/fanuc_ros2_icon_hwm/fanuc_crx20ia_l_hardware_device.bundle.tar \
     --org=<org>@<project> --cluster=<cluster>
   ```

3. **Add the Hardware Device Instance**:
   Instantiate the hardware device into the running solution under the name `robot`:
   ```bash
   inctl service add ai.intrinsic.fanuc_crx20ia_l_hardware_device --name=robot \
     --org=<org>@<project> --cluster=<cluster>
   ```

---

### Option B: Deploying as a Standalone Service

If you already have a pre-existing Scene Object in your solution and only need the ROS 2 communication service:

1. **Build and Install the Service**:
   ```bash
   bazel build //icon_hwm_controller_examples/fanuc_ros2_icon_hwm:fanuc_ros2_icon_hwm_service

   inctl asset install bazel-bin/icon_hwm_controller_examples/fanuc_ros2_icon_hwm/fanuc_ros2_icon_hwm_service.bundle.tar \
     --org=<org>@<project> --cluster=<cluster>
   ```

2. **Add the Service Instance**:
   ```bash
   inctl service add ai.intrinsic.fanuc_ros2_icon_hwm --name="fanuc_hwm" \
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

*(Note: When deploying as a standalone service with `--name="fanuc_hwm"`, replace `"robot"` with `"fanuc_hwm"` in the `module_name` fields above.)*

---

## Error Handling and FANUC Alarm Recovery Sequence

Error handling is implemented in [`fanuc_operational_state_node.cpp`](src/fanuc_operational_state_node.cpp).

### 1. State Monitoring
The node subscribes to `/fanuc_gpio_controller/robot_status` (`fanuc_msgs/msg/RobotStatus`):
* **Emergency Stop (`e_stopped == true`)**: Reports `FAULTED`. Requires physical release of the E-stop button before clearing faults.
* **Collaborative Contact Stop (`contact_stop_mode != NONE`)**: Reports `FAULTED`. Can be cleared via `clear_faults`.
* **Teach Pendant Active (`tp_enabled == true`)**: Reports `DISABLED` (manual mode). Motion via Flowstate is safely prevented.
* **Alarm / Error (`in_error == true` or `motion_possible == false`)**: Reports `FAULTED`.

### 2. Alarm Recovery Sequence (`ClearFaults`)
When **Clear Faults** is triggered from Flowstate:
1. Calls `/fanuc_gpio_controller/reset` (`fanuc_msgs/srv/Reset`) to reset active alarms on the FANUC controller.
2. Waits 200 ms for controller acknowledgment.
3. Calls `/fanuc_gpio_controller/switch_control_state` (`fanuc_msgs/srv/SwitchControlState`) with `status=1` to switch motion control authority back to the ROS 2 driver.
4. `icon_hwm_controller` re-activates the hardware component and resumes real-time trajectory execution.
