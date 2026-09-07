# FANUC ROS 2 ICON Hardware Module

This package contains the **FANUC ICON Hardware Module**, packaging the official [FANUC ROS 2 Driver (`fanuc_driver`)](https://github.com/FANUC-CORPORATION/fanuc_driver) into a real-time Intrinsic Service and deployable Hardware Device.

It supports both collaborative robots (CRX Series: CRX-5iA, CRX-10iA, CRX-10iA/L, CRX-20iA/L, CRX-25iA, CRX-30iA) and traditional industrial robots (LR Mate 200iD/7L, LR Mate 200iD/4S, LR-10iA/10, M-20iD/25, M-20iD/35, etc.).

> [!NOTE]
> If you use Intrinsic Enterprise, consider using the [native FANUC hardware module](https://flowstate.intrinsic.ai/docs/guides/commission_solution/set_up_robot/fanuc_stream_motion_setup/), which provides additional I/O support as well as automated set-up of the robot controller.
>
> Important: For best performance, FANUC robots require jerk-limited trajectories. Exceeding the robot's jerk limits causes the robot controller to clamp the motions, leading to tracking errors and path deviations.
> Either make sure to use a jerk-limited trajectory generator (such as provided by Intrinsic Enterprise) or set lower, more conservative joint velocity and acceleration limits.

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

1. **Host Prerequisites**: See main read-me.
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

## Building the ROS 2 Service Docker Image

The ROS 2 hardware module and the ICON controller are running inside a container on the Intrinsic platform. As a first step we will have to build the Docker image.

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

Now compile the Intrinsic Service bundle using Bazel. This adds the Intrinsic layer for parameter parsing on top of the base image.

```bash
bazel build //icon_hwm_controller_examples/fanuc_ros2_icon_hwm:fanuc_ros2_icon_hwm_service
```

The output asset bundle is created at:
`bazel-bin/icon_hwm_controller_examples/fanuc_ros2_icon_hwm/fanuc_ros2_icon_hwm_service.bundle.tar`

---

## Combining Scene Object and Service into a Hardware Device

In the Intrinsic platform, an **`intrinsic_hardware_device`** packages:
1. An **`intrinsic_scene_object`**: The robot's kinematic chain, collision and visual meshes, joint limits, and inverse kinematics solver (`spherical_wrist` for industrial and `kinematic_chain` for collaborative robots).
2. An **`intrinsic_service`**: The real-time ROS 2 driver service (`fanuc_ros2_icon_hwm_service`).

By packaging them into an `intrinsic_hardware_device`, adding the robot to a Flowstate Solution provisions both the 3D visual/kinematic representation and the communication backend in one step.

For the next steps, create a new folder `hardware_devices/fanuc_crx20ia_l` (or whichever robot you are onboarding) inside this folder with the following empty files. We will populate them in the following sections.
```
hardware_devices/fanuc_crx20ia_l/
├── model/
│   ├── fanuc_crx20ia_l.sdf
│   └── meshes/
│       ├── collision/
│       └── visual/
├── BUILD
├── fanuc_crx20ia_l_limits.pbtxt
├── fanuc_crx20ia_l_scene_object_manifest.textproto
└── fanuc_crx20ia_l_hardware_device_manifest.textproto
```

---

### 1. Kinematic Model

The SDF model can be based on the **URDF and meshes** found in the official [`fanuc_description`](https://github.com/FANUC-CORPORATION/fanuc_description) repository. In case your robot is not available there, check out the lower-resolution meshes in [ROS Industrial's `fanuc` repository](https://github.com/ros-industrial/fanuc).

Be sure to follow the main guide in this repository to add all required additional Intrinsic tags and modifications. All 6-DOF FANUC industrial arms use the `spherical_wrist` inverse kinematics solver while the [CRX collaborative robots](https://www.fanuc.eu/eu-en/crx-series) have to use the general `kinematic_chain` solver:
```xml
<intrinsic:ik_solver>spherical_wrist</intrinsic:ik_solver>
```

---

### 2. Robot limits and Control Frequency

The limits can be obtained as follows:

* **Position & Velocity Limits** can be either obtained from:
  * The FANUC mechanical specifications datasheet which can be downloaded from the product webpage (see e.g. [LR Mate 200iD/7L](https://www.fanuc.eu/eu-en/product/robot/lr-mate-200id7l)).
  * Found under "Zero point position and motion limit" in the mechanical unit operator's manual which can be downloaded upon registration from the [FANUC support portal](https://myportal.fanucamerica.com/).
  > [!IMPORTANT]
  > Datasheets give values in degrees ($^\circ$) and degrees/sec ($^\circ/\text{s}$). Always convert them to **radians** ($\text{rad} = \text{deg} \times \frac{\pi}{180}$) and **radians/sec** ($\text{rad}/\text{s}$) for the SDF model.
* **Acceleration & Jerk Limits**: FANUC controllers enforce internal limits depending on payload and model series. You can query these exact values directly from the controller via Stream Motion using [`stream_motion_example.cpp`](https://github.com/FANUC-CORPORATION/fanuc_driver/tree/main/fanuc_libs) in `fanuc_libs`.
* The **control frequency** depends on the robot model and controller generation and is either 125 or 250 Hz for the R-30iB+ and 500 or 1000 Hz for the R-50iA controller. The following configuration files assume the more recent R-50iA controller running at the default 500 Hz control frequency.

> [!NOTE]
> For the FANUC robots it is important to determine these limits correctly. They differ largely in between robots of different sizes and are vastly different between industrial and collaborative robots. Do not guess these values and do not copy them from another model.

Below you can find an example of these limits for the CRX-20iA/L (`fanuc_crx20ia_l_limits.pbtxt`).

<details>
<summary>Example robot limits for CRX-20iA/L</summary>

```textproto
# proto-file: intrinsic/scene/proto/v1/scene_object_updates.proto
# proto-message: intrinsic_proto.scene_object.v1.SceneObjectUpdates

updates: {
  update_joints {
    joint_system_limits: {
      key: "joint_a1"
      value: {
        min_position: -3.141592654  # -180 deg
        max_position: 3.141592654  # +180 deg
        max_velocity: 1.3683381335635543
        max_acceleration: 2.5656340004316647
        max_jerk: 9.6211275016187425
        max_effort: 200
      }
    }
    joint_system_limits: {
      key: "joint_a2"
      value: {
        min_position: -3.141592654  # -180 deg
        max_position: 3.141592654  # +180 deg
        max_velocity: 0.68416906678177714
        max_acceleration: 1.2828170002158323
        max_jerk: 4.8105637508093713
        max_effort: 200
      }
    }
    joint_system_limits: {
      key: "joint_a3"
      value: {
        min_position: -4.71238898  # -270 deg
        max_position: 4.71238898  # +270 deg
        max_velocity: 0.68416906678177714
        max_acceleration: 1.2828170002158323
        max_jerk: 4.8105637508093713
        max_effort: 200
      }
    }
    joint_system_limits: {
      key: "joint_a4"
      value: {
        min_position: -3.316125579  # -190 deg
        max_position: 3.316125579  # +190 deg
        max_velocity: 1.9242255003237483
        max_acceleration: 3.6079230740968145
        max_jerk: 13.529711593110504
        max_effort: 200
      }
    }
    joint_system_limits: {
      key: "joint_a5"
      value: {
        min_position: -3.141592654  # -180 deg
        max_position: 3.141592654  # +180 deg
        max_velocity: 1.5393804002589986
        max_acceleration: 2.8863382504856228
        max_jerk: 10.823768439321084
        max_effort: 200
      }
    }
    joint_system_limits: {
      key: "joint_a6"
      value: {
        min_position: -3.926990817  # -225 deg
        max_position: 3.926990817  # +225 deg
        max_velocity: 1.9242255003237483
        max_acceleration: 3.6079230740968145
        max_jerk: 13.529711593110504
        max_effort: 200
      }
    }

    # Joint position limits are set 0.1 rad below the system limits.
    joint_application_limits: {
      key: "joint_a1"
      value: {
        min_position: -3.041592654
        max_position: 3.041592654
        max_velocity: 1.29978439307202
        max_acceleration: 2.4370957370100381
        max_jerk: 9.1391090137876425
        max_effort: 180
      }
    }
    joint_application_limits: {
      key: "joint_a2"
      value: {
        min_position: -3.041592654
        max_position: 3.041592654
        max_velocity: 0.64989219653601
        max_acceleration: 1.2185478685050191
        max_jerk: 4.5695545068938213
        max_effort: 180
      }
    }
    joint_application_limits: {
      key: "joint_a3"
      value: {
        min_position: -4.61238898
        max_position: 4.61238898
        max_velocity: 0.64989219653601
        max_acceleration: 1.2185478685050191
        max_jerk: 4.5695545068938213
        max_effort: 180
      }
    }
    joint_application_limits: {
      key: "joint_a4"
      value: {
        min_position: -3.216125579
        max_position: 3.216125579
        max_velocity: 1.8278218027575284
        max_acceleration: 3.427166128084564
        max_jerk: 12.851873042295667
        max_effort: 180
      }
    }
    joint_application_limits: {
      key: "joint_a5"
      value: {
        min_position: -3.041592654
        max_position: 3.041592654
        max_velocity: 1.4622574422060226
        max_acceleration: 2.7417327041362931
        max_jerk: 10.281497640511096
        max_effort: 180
      }
    }
    joint_application_limits: {
      key: "joint_a6"
      value: {
        min_position: -3.826990817
        max_position: 3.826990817
        max_velocity: 1.8278218027575284
        max_acceleration: 3.427166128084564
        max_jerk: 12.851873042295667
        max_effort: 180
      }
    }
  }
}
```
</details>

---

### 3. Default Configuration Protobuf ([`proto/fanuc_ros2_icon_hwm_default_config.textproto`](proto/fanuc_ros2_icon_hwm_default_config.textproto))

The default configuration Protobuf contains the arguments that are set by default when an asset is added to the world. There we can set which parameters are passed to the ROS 2 launch file. More information on the individual `launch_parameters` can be found in the [official FANUC ROS 2 driver](https://github.com/FANUC-CORPORATION/fanuc_driver).

<details>
<summary>Example configuration Proto for CRX-20iA/L</summary>

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
        value: "robot"
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
</details>

---

### 4. Hardware Device Manifest (`fanuc_crx20ia_l_hardware_device_manifest.textproto`)
The hardware device manifest contains metadata about the asset such as asset name and a description.

<details>
<summary>Example hardware device manifest for CRX-20iA/L</summary>

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
</details>

---

### 5. Starlark `BUILD` Definition
An example `BUILD` file that builds the Intrinsic hardware device can be found below.

<details>
<summary>Example `BUILD` file for CRX-20iA/L</summary>

```python
load("@ai_intrinsic_sdks//intrinsic/scene/build_defs:sdf_scene_object.bzl", "sdf_scene_object")
load("@ai_intrinsic_sdks//intrinsic/assets/scene_objects/build_defs:scene_object.bzl", "intrinsic_scene_object")
load("@ai_intrinsic_sdks//intrinsic/assets/services/build_defs:services.bzl", "intrinsic_service")
load("@ai_intrinsic_sdks//intrinsic/assets/hardware_devices/build_defs:hardware_device.bzl", "intrinsic_hardware_device")

package(default_visibility = ["//visibility:public"])

# Step A: Kinematics and Mesh Definition (SDF Model)
sdf_scene_object(
    name = "fanuc_crx20ia_l_sdf",
    src = "model/fanuc_crx20ia_l.sdf",
    sdf_assets = glob(["model/meshes/**"]),
    updates_pbtxts = [
        "fanuc_crx20ia_l_limits.pbtxt",
    ],
)

# Step B: Scene Object Asset
# (Pre-configured scene object models can also be imported from the Intrinsic Core repository)
intrinsic_scene_object(
    name = "fanuc_crx20ia_l_scene_object",
    manifest = "fanuc_crx20ia_l_scene_object_manifest.textproto",
    scene_object = ":fanuc_crx20ia_l_sdf",
)

# Step C: Combined Hardware Device Asset
intrinsic_hardware_device(
    name = "fanuc_crx20ia_l_hardware_device",
    assets = [
        ":fanuc_crx20ia_l_scene_object",
        ":fanuc_ros2_icon_hwm_service",
    ],
    manifest = "fanuc_crx20ia_l_hardware_device_manifest.textproto",
)
```
</details>

---

## Deployment and Flowstate Configuration

### Option A: Deploying as a Hardware Device (Recommended)

Packaging and installing the robot as an `intrinsic_hardware_device` simultaneously adds the robot kinematics into the 3D scene and connects the ROS 2 driver service.

1. **Build the Hardware Device Asset**:
   ```bash
   bazel build //icon_hwm_controller_examples/fanuc_ros2_icon_hwm/hardware_devices/fanuc_crx20ia_l:fanuc_crx20ia_l_hardware_device
   ```

2. **Install the Asset into your Flowstate Solution**:
   ```bash
   inctl asset install bazel-bin/icon_hwm_controller_examples/fanuc_ros2_icon_hwm/hardware_devices/fanuc_crx20ia_l/fanuc_crx20ia_l_hardware_device.bundle.tar \
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
