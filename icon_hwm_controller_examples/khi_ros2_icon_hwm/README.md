# Kawasaki (KHI) ROS 2 ICON Hardware Module

This package contains the **Kawasaki Heavy Industries (KHI) ICON Hardware Module**, packaging the official [Kawasaki Robotics ROS 2 Driver (`khi_ros2`)](https://github.com/Kawasaki-Robotics/khi_ros2) into a real-time Intrinsic Service and deployable Hardware Device.

It supports a range of Kawasaki industrial and collaborative robots across the RS, BX, BXP, and duAro series with `f` (F60 / F0x) and `f_duaro` robot controllers.

---

## Table of Contents

1. [Prerequisites and Robot Controller Setup](#prerequisites-and-robot-controller-setup)
2. [Building the ROS 2 Service Docker Image](#building-the-ros-2-service-docker-image)
3. [Building the Intrinsic Service Asset](#building-the-intrinsic-service-asset)
4. [Combining Scene Object and Service into a Hardware Device](#combining-scene-object-and-service-into-a-hardware-device)
5. [Deployment and Flowstate Configuration](#deployment-and-flowstate-configuration)
   - [Deploying as a Hardware Device](#option-a-deploying-as-a-hardware-device-recommended)
   - [Configuring the Realtime Control Service](#configuring-the-realtime-control-service)
6. [Error Handling and Alarm Recovery Sequence](#error-handling-and-alarm-recovery-sequence)
   - [Known Issue: Resetting Twice on `ClearFaults`](#known-issue-resetting-twice-on-clearfaults)

---

## Prerequisites and Robot Controller Setup

Before connecting Flowstate or the ROS 2 driver to your physical Kawasaki controller:

1. **Host Prerequisites**: See main read-me.
   * [Docker Engine & Docker Compose](https://docs.docker.com/engine/install/)
   * [Bazelisk / Bazel](https://bazel.build/install/bazelisk)
   * [`inctl` CLI](https://flowstate.intrinsic.ai/docs/guides/build_with_code/set_up_your_development_environment/inctl_inbuild_installation/)

2. **Verify Network Connectivity**:
   Ensure your host machine or Intrinsic IPC can ping the robot controller IP (e.g. `192.170.10.1`).

3. **Configure the Kawasaki Robot Controller**:
   * On the controller, then in the Classic Mode select `Menu -> Aux Function -> System -> Network Setting -> Ethernet Port Setting`. Select `Port 1` and set the IP e.g. to `192.170.10.1` with the subnet mask `255.255.255.0`.
   * Follow the [official `khi_ros2` setup guide](https://github.com/Kawasaki-Robotics/khi_ros2#preparation) to prepare your F-Series controller for KRNX Real-Time Control (RTC):
     * Ensure the Teach/Repeat switch on the controller is set to **Repeat**.
     * Set **Teach Lock** to **Off** on the Teach Pendant.
     * Ensure all Emergency Stop and Protective Stop signals are released, and switch the controller state from **Hold** to **Run**.

---

## Building the ROS 2 Service Docker Image

The ROS 2 hardware module and the ICON controller run inside a container on the Intrinsic platform. As a first step, build the Docker image.

### 1. Build the Base Image
From the repository root, build the shared base image (`icon_hwm_base:latest`):
```bash
docker compose -f icon_hwm_controller/docker/docker-compose.yml build
```

### 2. Build the KHI Image
Build the image containing the patched `khi_ros2` driver and the KHI operational state node:
```bash
docker compose -f icon_hwm_controller_examples/khi_ros2_icon_hwm/docker/docker-compose.yml build
```

> [!NOTE]
> During the Docker build, two patches from [`patches/`](patches/) are applied on top of the upstream `khi_ros2` driver:
> * [`0001-remove-robot-controller-emergency-and-protective-stop-checks.patch`](patches/0001-remove-robot-controller-emergency-and-protective-stop-checks.patch): Removes false-positive emergency and protective stop checks in `KhiKrnxDriver::has_met_ros_requirements()` on F60 controllers.
> * [`0002-minimize-smoothing-and-delays.patch`](patches/0002-minimize-smoothing-and-delays.patch): Configures the KRNX Real-Time Control (RTC) driver for minimal latency and raw setpoint tracking.

### 3. Export the Image Tarball
Save the container image to `icon_hwm.tar` in the package directory so Bazel can package it as an OCI layer:
```bash
docker image save khi_ros2_icon_hwm:latest -o icon_hwm_controller_examples/khi_ros2_icon_hwm/icon_hwm.tar
```

---

## Building the Intrinsic Service Asset

Now compile the Intrinsic Service bundle using Bazel. This adds the Intrinsic layer for parameter parsing on top of the base image.

```bash
bazel build //icon_hwm_controller_examples/khi_ros2_icon_hwm:khi_ros2_icon_hwm_service
```

The output asset bundle is created at:
`bazel-bin/icon_hwm_controller_examples/khi_ros2_icon_hwm/khi_ros2_icon_hwm_service.bundle.tar`

---

## Combining Scene Object and Service into a Hardware Device

In the Intrinsic platform, an **`intrinsic_hardware_device`** packages:
1. An **`intrinsic_scene_object`**: The robot's kinematic chain, collision and visual meshes, joint limits, and inverse kinematics solver (`kinematic_chain` or `spherical_wrist`).
2. An **`intrinsic_service`**: The real-time ROS 2 driver service (`khi_ros2_icon_hwm_service`).

By packaging them into an `intrinsic_hardware_device`, adding the robot to a Flowstate Solution provisions both the 3D visual/kinematic representation and the communication backend in one step.

For the next steps, create a new folder `hardware_devices/khi_rs007l_b001` (or whichever Kawasaki robot you are onboarding) inside this package with the following structure:
```text
hardware_devices/khi_rs007l_b001/
│   ├── khi_rs007l_b001.sdf
│   └── meshes/
│       ├── collision/
│       └── visual/
├── BUILD
├── khi_rs007l_b001_limits.pbtxt
├── khi_rs007l_b001_scene_object_manifest.textproto
└── khi_rs007l_b001_hardware_device_manifest.textproto
```

---

### 1. Kinematic Model

The SDF model can be generated from the **Xacro/URDF and meshes** found in the official [`khi_description`](https://github.com/Kawasaki-Robotics/khi_ros2/tree/main/khi_description) package.

Be sure to follow the [main guide](../../README.md#custom-intrinsic-sdf-tags) in this repository to add all required additional Intrinsic XML tags and modifications (such as `<intrinsic:ik_solver>`, `<intrinsic:cartesian_limits>`, the `flange` attachment frame, and `TimeslicerPlugin`):
```xml
<intrinsic:ik_solver>kinematic_chain</intrinsic:ik_solver>
```

---

### 2. Robot Limits and Control Frequency

The limits and control rate can be obtained as follows:

* **Position, Velocity, and Acceleration Limits** can be obtained from:
  * The Xacro macro files in `khi_description/config/<series>/<robot>_macro.xacro` and the MoveIt limit configs in `khi_moveit_config/config/<robot>/joint_limits.yaml` from the [`khi_ros2`](https://github.com/Kawasaki-Robotics/khi_ros2) repository.
  * The Kawasaki mechanical specifications datasheet for the specific manipulator model.
  > [!IMPORTANT]
  > Datasheets give values in degrees ($^\circ$) and degrees/sec ($^\circ/\text{s}$). Always convert them to **radians** ($\text{rad} = \text{deg} \times \frac{\pi}{180}$) and **radians/sec** ($\text{rad}/\text{s}$) for the SDF model and limits file.
* **Control Frequency**: The KHI KRNX Real-Time Control (RTC) interface depends on the controller generation and can be found in [`khi_description/config/update_rate`](https://github.com/Kawasaki-Robotics/khi_ros2/tree/main/khi_description/config/update_rate).

Below you can find an example of these limits for the Kawasaki RS007L (`khi_rs007l_b001_limits.pbtxt`).

<details>
<summary>Example robot limits for RS007L (<code>khi_rs007l_b001_limits.pbtxt</code>)</summary>

```textproto
# proto-file: intrinsic/scene/proto/v1/scene_object_updates.proto
# proto-message: intrinsic_proto.scene_object.v1.SceneObjectUpdates

updates: {
  update_joints {
    joint_system_limits: {
      key: "joint1"
      value: {
        min_position: -3.14159265359  # -180 deg
        max_position: 3.14159265359   # +180 deg
        max_velocity: 6.0057
        max_acceleration: 6.1348
        max_jerk: 200
        max_effort: 1000
      }
    }
    joint_system_limits: {
      key: "joint2"
      value: {
        min_position: -2.35619449019  # -135 deg
        max_position: 2.35619449019   # +135 deg
        max_velocity: 5.4105
        max_acceleration: 1.8035
        max_jerk: 200
        max_effort: 1000
      }
    }
    joint_system_limits: {
      key: "joint3"
      value: {
        min_position: -2.74016692563  # -157 deg
        max_position: 2.74016692563   # +157 deg
        max_velocity: 7.1558
        max_acceleration: 4.2935
        max_jerk: 200
        max_effort: 1000
      }
    }
    joint_system_limits: {
      key: "joint4"
      value: {
        min_position: -3.49065850399  # -200 deg
        max_position: 3.49065850399   # +200 deg
        max_velocity: 8.9274
        max_acceleration: 18.7187
        max_jerk: 200
        max_effort: 1000
      }
    }
    joint_system_limits: {
      key: "joint5"
      value: {
        min_position: -2.18166156499  # -125 deg
        max_position: 2.18166156499   # +125 deg
        max_velocity: 8.9274
        max_acceleration: 21.5984
        max_jerk: 200
        max_effort: 1000
      }
    }
    joint_system_limits: {
      key: "joint6"
      value: {
        min_position: -6.28318530718  # -360 deg
        max_position: 6.28318530718   # +360 deg
        max_velocity: 17.4532
        max_acceleration: 9.5993
        max_jerk: 200
        max_effort: 1000
      }
    }

    # Joint position limits are set 0.1 rad below the system limits.
    # Velocity, acceleration, and jerk limits are scaled to 95% of system limits.
    # Effort limits are scaled to 90% of system limits.
    joint_application_limits: {
      key: "joint1"
      value: {
        min_position: -3.04159265359
        max_position: 3.04159265359
        max_velocity: 5.682
        max_acceleration: 5.82806
        max_jerk: 180
        max_effort: 900
      }
    }
    joint_application_limits: {
      key: "joint2"
      value: {
        min_position: -2.25619449019
        max_position: 2.25619449019
        max_velocity: 5.139975
        max_acceleration: 1.713325
        max_jerk: 180
        max_effort: 900
      }
    }
    joint_application_limits: {
      key: "joint3"
      value: {
        min_position: -2.64016692563
        max_position: 2.64016692563
        max_velocity: 6.79801
        max_acceleration: 4.072
        max_jerk: 180
        max_effort: 900
      }
    }
    joint_application_limits: {
      key: "joint4"
      value: {
        min_position: -3.39065850399
        max_position: 3.39065850399
        max_velocity: 8.48103
        max_acceleration: 17.659
        max_jerk: 180
        max_effort: 900
      }
    }
    joint_application_limits: {
      key: "joint5"
      value: {
        min_position: -2.08166156499
        max_position: 2.08166156499
        max_velocity: 8.48103
        max_acceleration: 20.51848
        max_jerk: 180
        max_effort: 900
      }
    }
    joint_application_limits: {
      key: "joint6"
      value: {
        min_position: -6.18318530718
        max_position: 6.18318530718
        max_velocity: 16.58054
        max_acceleration: 9.119335
        max_jerk: 180
        max_effort: 900
      }
    }
  }
}
```
</details>

---

### 3. Default Configuration Protobuf ([`proto/khi_ros2_icon_hwm_default_config.textproto`](proto/khi_ros2_icon_hwm_default_config.textproto))

The default configuration Protobuf contains the arguments that are set by default when the service asset is added to the world. Here you configure which parameters are passed to [`launch/khi_control.launch.py`](launch/khi_control.launch.py) (such as `hwm_name`, `robot`, `robot_ip`, and optionally `robot_controller` or `controller_no`).

<details>
<summary>Example configuration Proto for RS007L</summary>

```textproto
# proto-file: google/protobuf/any.proto
# proto-message: google.protobuf.Any

[type.googleapis.com/intrinsic_proto.icon.HardwareModuleConfig] {
  drives_realtime_clock: true
  control_frequency_hz: 500.0
  module_config {
    [type.googleapis.com/intrinsic_proto.services.Ros2HwmConfig] {
      launch_package: "khi_ros2_icon_hwm"
      launch_file: "khi_control.launch.py"
      launch_parameters {
        key: "hwm_name"
        value: "robot"
      }
      launch_parameters {
        key: "robot"
        value: "rs007l-b001"
      }
      launch_parameters {
        key: "robot_ip"
        value: "192.170.10.1"
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
</details>

---

### 4. Hardware Device Manifest (`khi_rs007l_b001_hardware_device_manifest.textproto`)

The hardware device manifest contains metadata about the asset (such as package ID, display name, and description) and links the `scene_object` and `service` assets together.

<details>
<summary>Example hardware device manifest for RS007L</summary>

```textproto
# proto-file: intrinsic/assets/hardware_devices/proto/v1/hardware_device_manifest.proto
# proto-message: intrinsic_proto.hardware_devices.v1.HardwareDeviceManifest

metadata {
  id {
    package: "ai.intrinsic"
    name: "khi_rs007l_b001_hardware_device"
  }
  vendor {
    display_name: "Kawasaki"
  }
  documentation {
    description: "Kawasaki RS007L (rs007l_b001) Hardware Device combining kinematics geometry and ROS 2 HWM service."
  }
  display_name: "Kawasaki RS007L Hardware Device"
}

graph {
  nodes {
    key: "scene_object"
    value {
      asset: "ai.intrinsic.khi_rs007l_b001_scene_object"
    }
  }
  nodes {
    key: "service"
    value {
      asset: "ai.intrinsic.khi_ros2_icon_hwm"
    }
  }
}
```
</details>

---

### 5. Starlark `BUILD` Definition

An example `BUILD` file that builds the Intrinsic hardware device can be found below.

<details>
<summary>Example <code>BUILD</code> file for RS007L</summary>

```python
load("@ai_intrinsic_sdks//intrinsic/assets/hardware_devices/build_defs:hardware_device.bzl", "intrinsic_hardware_device")
load("@ai_intrinsic_sdks//intrinsic/assets/scene_objects/build_defs:scene_object.bzl", "intrinsic_scene_object")
load("@ai_intrinsic_sdks//intrinsic/scene/build_defs:sdf_scene_object.bzl", "sdf_scene_object")

package(default_visibility = ["//visibility:public"])

# Step A: Kinematics and Mesh Definition (SDF Model)
sdf_scene_object(
    name = "khi_rs007l_b001_sdf",
    src = "model/khi_rs007l_b001.sdf",
    sdf_assets = glob([
        "model/meshes/**",
    ]),
    updates_pbtxts = [
        "khi_rs007l_b001_limits.pbtxt",
    ],
)

# Step B: Scene Object Asset
intrinsic_scene_object(
    name = "khi_rs007l_b001_scene_object",
    manifest = "khi_rs007l_b001_scene_object_manifest.textproto",
    scene_object = ":khi_rs007l_b001_sdf",
)

# Step C: Combined Hardware Device Asset
intrinsic_hardware_device(
    name = "khi_rs007l_b001_hardware_device",
    assets = [
        ":khi_rs007l_b001_scene_object",
        ":khi_ros2_icon_hwm_service",
    ],
    manifest = "khi_rs007l_b001_hardware_device_manifest.textproto",
)
```
</details>

---

## Deployment and Flowstate Configuration

### Deploying as a Hardware Device

Packaging and installing the robot as an `intrinsic_hardware_device` simultaneously adds the robot kinematics into the 3D scene and connects the ROS 2 driver service.

1. **Build the Hardware Device Asset**:
   ```bash
   bazel build //icon_hwm_controller_examples/khi_ros2_icon_hwm/hardware_devices/khi_rs007l_b001:khi_rs007l_b001_hardware_device
   ```

2. **Install the Asset into your Flowstate Solution**:
   ```bash
   inctl asset install bazel-bin/icon_hwm_controller_examples/khi_ros2_icon_hwm/hardware_devices/khi_rs007l_b001/khi_rs007l_b001_hardware_device.bundle.tar \
     --org=<org>@<project> --cluster=<cluster>
   ```

3. **Add the Hardware Device Instance**:
   Instantiate the hardware device into the running solution under the name `robot`:
   ```bash
   inctl service add ai.intrinsic.khi_rs007l_b001_hardware_device --name=robot \
     --org=<org>@<project> --cluster=<cluster>
   ```

---

### Configuring the Realtime Control Service

In Flowstate, navigate to **Services -> Realtime Control Service -> Config** and configure the `IconMainConfig`:

```protobuf
[type.googleapis.com/intrinsic_proto.icon.IconMainConfig]:  {
  intrinsic_runtime:  {}
  control_frequency_hz:  500
  hard_deadline:  true
  services:  {
    world_service_from_grpc:  {
      world_id:  "world"
    }
    kinematics_from_world_service:  true
    assembly_from_world_service:  true
  }
  realtime_control_config:  {
    parts_by_name:  {
      key:  "arm"
      value:  {
        part_type_name:  "HalArmPart"
        config:  {
          [type.googleapis.com/intrinsic_proto.icon.HalArmPartConfig]:  {
            joint_position_command:  {
              module_name:  "robot"
              interface_name:  "joint_position_command"
            }
            joint_position_state:  {
              module_name:  "robot"
              interface_name:  "joint_position_state"
            }
            joint_velocity_state:  {
              module_name:  "robot"
              interface_name:  "joint_velocity_state"
            }
            kinematics_model_name:  "arm"
          }
        }
        safety_action_type_name:  "intrinsic.stop"
        hardware_resource_name:  "robot"
      }
    }
  }
  hardware_module_names:  "robot"
  hardware_module_that_drives_clock:  "robot"
  hardware_module_read_write_timeout_seconds:  10
  deactivated_hardware_configuration:  {}
}
```

---

## Error Handling and Alarm Recovery Sequence

Error handling is implemented in [`khi_operational_state_node.cpp`](src/khi_operational_state_node.cpp).

### 1. State Monitoring
The node subscribes to `/khi_controller<N>/khi_publisher/error_info` (`khi_msgs/msg/ErrorInfo`, where `<N>` defaults to `0`):
* When `error_codes` is empty (no active error reported), the node publishes `OperationalStatus::ENABLED` on `/operational_status`.
* When `error_codes` is non-empty (e.g., E-stop pressed, protective stop, or motor power off), the node transitions to `OperationalStatus::FAULTED` with the controller's error message (e.g., `KHI robot error active: Motor power OFF. Call clear_faults to recover.`).

### 2. Alarm Recovery Sequence (`ClearFaults`)
When **Clear Faults** is triggered from Flowstate:
1. Calls `/khi_controller<N>/khi_service/reset_error` (`khi_msgs/srv/ResetError`) to clear active alarms on the Kawasaki controller.
2. Resets the cached `OperationalStatus` in `khi_operational_state_node` back to `ENABLED`.
3. `icon_hwm_controller` re-activates `KhiHardwareInterface` and `icon_controller`, which restores motor power, restarts the KRNX RTC program on the controller, and resumes real-time trajectory execution.

### Known Issue: Resetting Twice on `ClearFaults`

> [!WARNING]
> **On `ClearFaults`, you must perform a reset twice to recover from a robot fault.**
>
> When an error occurs on the Kawasaki controller (for example, `Motor power OFF`), `KhiHardwareInterface::write()` immediately returns `return_type::DEACTIVATE` and sleeps for 1 second during deactivation. Because `ros2_control` deactivates `icon_controller` and stalls the real-time loop before ICON can process the `FAULTED` operational status, ICON first faults with a real-time clock timeout:
> ```text
> DEADLINE_EXCEEDED: BeginCycle: robot: Timeout after 200.006 ms
> ```
> Meanwhile, `khi_publisher` publishes the error on `/khi_controller0/khi_publisher/error_info` only a single time when the fault occurs, and `khi_operational_state_node` caches the `FAULTED` state.
>
> As a result, when you trigger **Clear Faults / Reset** the **first** time, the hardware interface and real-time clock recover from the `DEADLINE_EXCEEDED` timeout, but enabling motion fails because the cached KHI fault state is now surfaced:
> ```text
> 'robot': Cannot enable motion while faulted (KHI robot error active: Motor power OFF. Call clear_faults to recover.)
> ```
> Triggering **Clear Faults / Reset a second time** clears the KHI fault state and re-enables motion as expected.
