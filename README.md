# ICON ROS 2 Control Hardware Module Controller
Owner: nilsb@

This repository contains a **[ROS 2 Control Controller](https://control.ros.org/kilted/doc/ros2_controllers/doc/writing_new_controller.html) which allows [ROS 2 Control Hardware Components](https://control.ros.org/kilted/doc/ros2_control/hardware_interface/doc/hardware_components_userdoc.html) to be used as [ICON Hardware Modules](https://flowstate.intrinsic.ai/docs/apis/client_libraries/icon_extensions/custom_hardware_modules/)** as well as the tools to build [Intrinsic Services](https://flowstate.intrinsic.ai/docs/assets/create_new_assets/create_services/overview/) from it that can be side-loaded into the [Intrinsic Platform](https://flowstate.intrinsic.ai/docs/guides/get_started/overview/). Furthermore it contains two examples for creating hardware modules based on the official ROS 2 drivers for [Universal Robots](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) as well as [FANUC](https://github.com/FANUC-CORPORATION/fanuc_driver) and outlines best practices on how to develop your own hardware modules based on the ICON HWM Controller.

We are currently targetting [ROS 2 Kilted Kaiju](https://docs.ros.org/en/kilted/index.html) which is based on [Ubuntu 24.04 (Noble)](https://ubuntu.com/blog/tag/ubuntu-24-04-lts).

## Step-by-step Guide

### Prerequisites

This repository can be used as a standalone without having ROS 2 installed on your host system. Instead the workflow is based on Docker containers.

In order to follow this guide, you will need to **install the following software packages on your host system**. The installation procedure for each of these tools is linked below:

* [**Git**](https://git-scm.com/install/): Git is used for version control.
* [**Docker**](https://docs.docker.com/engine/install/): Docker is used to create containers which can run as Intrinsic Services on the Intrinsic platform.
* [**Bazelisk**](https://bazel.build/install/bazelisk): Bazel is used to build Intrinsic Services for the Intrinsic platform. Alternatively to installing Bazel on your host system you can also use the [Intrinsic DevContainer](https://flowstate.intrinsic.ai/docs/guides/build_with_code/set_up_your_development_environment/local_environment/) which already ships with Bazel.


### Running the examples

This repository contains two readily-available examples based on the official ROS 2 drivers for [Universal Robots](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) as well as [FANUC](https://github.com/FANUC-CORPORATION/fanuc_driver). The following section walks you through the steps necessary for building these examples on your system.

#### Build the Docker images

As a first step we will have to build the Docker images containing the ROS 2 driver and save them as tarballs. The resulting tarballs can then be used to create Intrinsic services using Bazel.

As a first step build the base image. This image is based on [ROS 2 Kilted Kaiju](https://docs.ros.org/en/kilted/index.html) and contains the ICON HWM ROS 2 Controller as well as its base dependencies. Once the image is built the image will be automatically saved as `icon_hwm_base:latest`. This image is then referenced in the following examples.
```bash
docker compose -f icon_hwm_controller/docker/docker-compose.yml build
```
Then build the example image for the robot of interest, in the following, the FANUC. This contains the FANUC-specific dependencies (such as the `fanuc_driver`) and launch files:
```bash
docker compose -f icon_hwm_controller_examples/fanuc_ros2_icon_hwm/docker/docker-compose.yml build
```
Finally we will have to manually save the resulting image to disk so that we can use it as an Intrinsic Service. The name that was given to the image, in this case `fanuc_ros2_icon_hwm:latest`, can be found in the Docker-Compose file. The output name `icon_hwm.tar` and its location are also crucial as they are referenced in the Bazel `BUILD` file used to build the Intrinsic Service.
```bash
docker image save fanuc_ros2_icon_hwm:latest -o icon_hwm_controller_examples/fanuc_ros2_icon_hwm/icon_hwm.tar
```

#### Build the Intrinsic Service

After having saved the image to disk, make sure that the file `icon_hwm.tar` was correctly written to the root directory of the corresponding example folder. Now you can proceed to build the Intrinsic Service following these instructions. This requires Bazel.
```bash
bazel build icon_hwm_controller_examples/fanuc_ros2_icon_hwm:fanuc_ros2_icon_hwm_service
```
This should create a Service Asset tarball which is located in
`./bazel-bin/icon_hwm_controller_examples/<example_dir>/fanuc_ros2_icon_hwm_service.bundle.tar`.

#### Configuring the robot controller

Before proceeding to install the Intrinsic Service into your solution, let's make sure that the robot controller has been set up correctly. This is specific to the robot manufacturer and generally outlined on their ROS 2 driver documentation.

You can set up the FANUC robot controller by following the [official guide](https://fanuc-corporation.github.io/fanuc_driver_doc/main/docs/quick_start/quick_start.html#robot-controller-setup). In case you have been running the robot with Flowstate previously (see [here](https://flowstate.intrinsic.ai/docs/guides/commission_solution/set_up_robot/fanuc_stream_motion_setup/)), be sure to disable the `ENABLE_UI_SIGNALS` setting. Refer to the linked manual on how to perform these change depending on the controller generation at hand.

#### Add the Intrinsic Service to your solution

Make sure a solution is running on the cluster of interest.
You can sideload the previously compiled Service into the solution by running the following command:
```bash
inctl asset install ./bazel-bin/icon_hwm_controller_examples/fanuc_ros2_icon_hwm/fanuc_ros2_icon_hwm_service.bundle.tar --org=<org>@<project> --cluster=<cluster>
```
Finally you can add the service to the running solution by referencing the service name found in its manifest. The name you give the instance itself, in this case `fanuc_hwm` is arbitrary:
```bash
inctl service add ai.intrinsic.fanuc_ros2_icon_hwm --name="fanuc_hwm" --org=<org>@<project> --cluster=<cluster>
```
Finally in the service configuration in Flowstate you will have to fill the name parameter with `fanuc_hwm`.

To actually control the robot, you need to add two more things to your solution:
* A realtime control service 
* A SceneObject that represents your robot's kinematics

To add a realtime control service, you can either use the Flowstate frontend, or the command line.

To create a SceneObject for your robot, follow [these instructions](https://flowstate.intrinsic.ai/docs/guides/design_a_workcell/set_up_hardware_modules/install_custom_robot_kinematics/).


Alternatively, you can use the [`sdf_scene_object`](https://github.com/intrinsic-ai/sdk/blob/0e243aed63364eafcd41082d1c7aad67e844b876/intrinsic/scene/build_defs/sdf_scene_object.bzl#L133) and [`intrinsic_scene_object`](https://github.com/intrinsic-ai/sdk/blob/0e243aed63364eafcd41082d1c7aad67e844b876/intrinsic/assets/scene_objects/build_defs/scene_object.bzl#L130) rules from https://github.com/intrinsic-ai/sdk. Expand the section below for an example.

<details>
<summary>Example</summary>

```python
load("//intrinsic/scene/build_defs:sdf_scene_object.bzl", "sdf_scene_object")
load("//intrinsic/assets/scene_objects/build_defs:scene_object.bzl", "intrinsic_scene_object")

sdf_scene_object(
  name = "my_robot_sdf",
  src = "robot_model.sdf",
  sdf_assets = [
    "mesh1.stl",
    "mesh2.stl",
    "mesh3.stl",
  ],
)

intrinsic_scene_object(
  name = "my_robot",
  # This is an `intrinsic_proto.scene_objects.SceneObjectManifest`
  # textproto.
  manifest = "my_robot_manifest.textproto",
  scene_object = ":my_robot_sdf",
)
```

Check the [definition of the SceneObjectManifest proto](https://github.com/intrinsic-ai/insrc/blob/f105f45a63797f323f4abfa2fcf24a4226eed484/google3/intrinsic/assets/scene_objects/proto/scene_object_manifest.proto) to fill that in.

</details>

### Developing your own ICON Hardware Module based on an existing ROS 2 driver

Now that you have tried the examples that we provided, you might want to create your own ICON Hardware Module based on an existing open source ROS 2 driver.

Since we are **targetting ROS 2 Kilted**, also your ROS 2 driver will have to be available for this ROS 2 distribution either as a Debian package or by compiling it from source. Sometimes also using another ROS 2 distribution (and potentially manually applying [Git patch files](https://git-scm.com/docs/git-apply)) might work as well.

When developing a new ICON Hardware Module based on a ROS 2 driver it is recommended to have ROS 2 installed on your system locally. This makes the following procedure much more convenient. We will start by stripping down an existing launch file so that it just contains a single joint trajectory controller and will then convert this to a launch file for the ICON HWM controller in the next step.

#### Build a minimal launch file with a joint trajectory controller

Look for the existing launch file from the official driver. For the FANUC driver this is [`fanuc_physical_control.launch.py`](https://github.com/FANUC-CORPORATION/fanuc_driver/blob/main/fanuc_hardware_interface/launch/fanuc_physical_control.launch.py) and the corresponding [`ros2_controllers.yaml`](https://github.com/FANUC-CORPORATION/fanuc_driver/blob/main/fanuc_hardware_interface/config/ros2_controllers.yaml).

Copy these files to a new package and strip them down to a bare minimum. We will want to remove all references to any unnecessary controllers. You essentially only need the joint state publisher and the joint trajectory controller.

Compile the workspace and launch the launch file. For testing whether everything works as expected, jog the robot with the `rqt_joint_trajectory-controller:
```bash
sudo apt-get install ros-$ROS_DISTRO-rqt-joint-trajectory-controller
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

#### Create a Dockerfile

Create a Dockerfile which is based on the `icon_hwm_base:latest` image. Install all required dependencies into the image. You should be able to perform the vast majority of steps with `rosdep`. Copy the newly created package into the image and compile it. Make sure the Dockerfile is able to start correctly by creating a Docker Compose file and executing `docker compose up`.

#### **Optional:** Create an OperationalStatus node for improved error handling

Since error reporting and recovery are not standardized across ros2_control drivers, the ICON HWM Controller does not handle errors gracefully out of the box.

To improve error handling, create a custom node that 

* publishes an [`OperationalStatus`](icon_hwm_controller_msgs/msg/OperationalStatus.msg) topic
* offers an `std_srvs/srv/Trigger` service that clears any errors/faults (if possible)

See [the UR OperationalStatus node](icon_hwm_controller_examples/ur_ros2_icon_hwm/src/ur_operational_state_node.cpp) for an example.

Add this node to your launch file, and configure it to talk to your robot driver.

#### Replace the Joint Trajectory Controller with the ICON HWM Controller

Now that you managed to create an image that is able to launch a Joint Trajectory Controller, let's modify the launch and config files to configure the ICON HWM Controller instead. 

This requires:

- Modifications to the launch file:
  - Additional launch file arguments specific to the ROS 2 Controller. See [`icon_hwm_controller_parameters.yaml`](icon_hwm_controller/src/icon_hwm_controller_parameters.yaml) for detailed descriptions of these parameters.

    * `hwm_name` (`string`)
    * `shm_namespace` (`string`)
    * `context_name` (`string`)
    * `lock_memory` (`bool`)
    * `cpu_affinity` (`[int]`)
    * `realtime_priority_low` (`int`)
    * `realtime_priority_high` (`int`)
    * `control_frequency_hz`(`int`)
    * `drives_realtime_clock` (`bool`)

    Don't worry about setting these arguments yourself, the Intrinsic service we build later automatically populates them.

  - Add the option `allow_substs=True` to the ROS 2 controllers parameters file
  - Spawn the ICON controller instead of the Joint Trajectory Controller. You will have to spawn it as `inactive` instead of `active`.
- Modifications to the configuration file:
  - Remove all settings for the Joint Trajectory Controller and instead add the settings for the ICON HWM Controller. Be sure to adapt the DOF names.
  - Use the arguments from the launch file in the controller configuration:
    <details>
    <summary>Example (based on the UR driver)</summary>
    ```yaml
    icon_controller:
      ros__parameters:
        type: icon_hwm_controller/IconHwmController
        name: "$(var hwm_name)"
        context_name: "$(var context_name)"
        shm_namespace: "$(var shm_namespace)"
        lock_memory: $(var lock_memory)
        cpu_affinity: $(var cpu_affinity)
        realtime_priority_low: $(var realtime_priority_low)
        realtime_priority_high: $(var realtime_priority_high)
        drives_realtime_clock: $(var drives_realtime_clock)
        control_frequency_hz: $(var control_frequency_hz)
        dof_names:
          # Adjust these in to match the joint names
          # of the ROS2 driver.
          - $(var tf_prefix)shoulder_pan_joint
          - $(var tf_prefix)shoulder_lift_joint
          - $(var tf_prefix)elbow_joint
          - $(var tf_prefix)wrist_1_joint
          - $(var tf_prefix)wrist_2_joint
          - $(var tf_prefix)wrist_3_joint
        command_interfaces:
          - position
        reference_and_state_interfaces:
          - position
          - velocity
        # Update this to match your robot
        hardware_component_name: $(var ur_type)
        # If you have a custom OperationalStatus node, configure the topic and service
        # here. If you do not, omit these lines.
        operational_status_topic: /ur_operational_state_node/ur_operational_status
        clear_faults_trigger_service: /ur_operational_state_node/clear_faults    
    </details>

After performing these changes, you should still be able to launch your container successfully.

#### Create an Intrinsic Service

Finally let's wrap this image in an Intrinsic Service. For this purpose you will have to create a new manifest, a default configuration as well as a `BUILD` file that creates the Intrinsic Service.

Once that build finishes, you can install and add it to your solution in the same way described above.