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

from typing import Dict, List

from icon_hwm_controller.launch import get_icon_hwm_launch_arguments
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context: LaunchContext) -> List[Node]:
    """
    Evaluate runtime launch configurations and initialize the ROS 2 nodes.

    This function is called as an OpaqueFunction, allowing it to access the
    runtime LaunchContext. It uses this context to evaluate string values for
    launch configurations (like robot_model) to dynamically build the URDF via
    Xacro and configure the controller nodes.
    """
    controller_spawner_timeout = LaunchConfiguration('controller_spawner_timeout')
    gpio_config_package = LaunchConfiguration('gpio_config_package')
    gpio_config_path = LaunchConfiguration('gpio_config_path')
    robot_ip = LaunchConfiguration('robot_ip')
    robot_model = LaunchConfiguration('robot_model')
    robot_series = LaunchConfiguration('robot_series')

    robot_model_str: str = robot_model.perform(context)
    robot_series_str: str = robot_series.perform(context)

    if robot_series_str == 'crx':
        urdf_xacro_file: str = robot_model_str + '.urdf.xacro'
    else:
        urdf_xacro_file: str = '6dof_robot.urdf.xacro'

    controllers_file = PathJoinSubstitution(
        [FindPackageShare('fanuc_ros2_icon_hwm'), 'config', 'controllers.yaml']
    )
    gpio_config_file = PathJoinSubstitution(
        [FindPackageShare(gpio_config_package), gpio_config_path]
    )

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('fanuc_hardware_interface'), 'robot', urdf_xacro_file]
            ),
            ' ',
            "prefix:=''",
            ' ',
            'robot_series:=',
            robot_series,
            ' ',
            'robot_ip:=',
            robot_ip,
            ' ',
            'robot_model:=',
            robot_model,
            ' ',
            'gpio_configuration:=',
            gpio_config_file,
        ]
    )
    robot_description: Dict[str, ParameterValue] = {
        'robot_description': ParameterValue(value=robot_description, value_type=str)
    }
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            ParameterFile(controllers_file, allow_substs=True),
        ],
        output='screen',
    )

    def controller_spawner(controllers: List[str], active: bool = True) -> Node:
        """
        Create a controller manager spawner node.

        If active is False, appends the '--inactive' flag so the controller is
        spawned in an inactive state.
        """
        inactive_flags: List[str] = ['--inactive'] if not active else []
        return Node(
            package='controller_manager',
            executable='spawner',
            parameters=[
                {'verify_payload_on_set': True},
                # We substitute the arguments into the controller configuration.
                ParameterFile(controllers_file, allow_substs=True),
            ],
            arguments=[
                '--controller-manager',
                '/controller_manager',
                '--controller-manager-timeout',
                controller_spawner_timeout,
            ]
            + inactive_flags
            + controllers,
        )

    controllers_active: List[str] = [
        'joint_state_broadcaster',
        # This is required for publishing the robot_status for error handling.
        'fanuc_gpio_controller',
    ]
    # The ICON controller has to be started in an inactive state. The controller
    # is designed to self-activate during `Prepare()`.
    controllers_inactive: List[str] = [
        'icon_controller',
    ]
    controller_spawner_nodes: List[Node] = [
        controller_spawner(controllers_active, active=True),
        controller_spawner(controllers_inactive, active=False),
    ]

    fanuc_operational_state_node = Node(
        package='fanuc_ros2_icon_hwm',
        executable='fanuc_operational_state_node',
        output='screen',
    )

    nodes_to_start: List[Node] = [
        robot_state_publisher_node,
        control_node,
        fanuc_operational_state_node,
    ] + controller_spawner_nodes
    return nodes_to_start


def generate_launch_description() -> LaunchDescription:
    """
    Generate the launch description for the FANUC ROS 2 controller.

    Declares all available launch arguments that can be passed via the command
    line, and returns a LaunchDescription containing these arguments alongside
    an OpaqueFunction that defers node creation to `launch_setup`.
    """
    declared_arguments: List[DeclareLaunchArgument] = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'controller_spawner_timeout',
            default_value='10',
            description='Timeout used when spawning controllers.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'gpio_config_package',
            default_value='fanuc_ros2_icon_hwm',
            description='The package name where gpio_configuration file exists',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'gpio_config_path',
            default_value='config/gpio_controller.yaml',
            description='The gpio_configuration file path in gpio_config_package',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_ip',
            description='IP address by which the robot can be reached.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model',
            description='The robot model.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_series',
            description="The robot series such as 'crx'.",
        )
    )
    # The following arguments are required by the ICON HWM controller.
    declared_arguments.extend(get_icon_hwm_launch_arguments())

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
