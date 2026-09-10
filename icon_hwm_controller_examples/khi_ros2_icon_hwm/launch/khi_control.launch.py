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

    This function is called as an `OpaqueFunction`, allowing it to access the
    runtime `LaunchContext`. It uses this context to evaluate string values for
    launch configurations (like robot_model) to dynamically build the URDF via
    Xacro and configure the controller nodes.
    """
    controller_no = LaunchConfiguration('controller_no')
    controller_spawner_timeout = LaunchConfiguration('controller_spawner_timeout')
    robot = LaunchConfiguration('robot')
    robot_controller = LaunchConfiguration('robot_controller')
    robot_ip = LaunchConfiguration('robot_ip')

    # Brittle check but in line with the `khi_bringup.launch.py` launch file
    # from `khi_hardware`.
    robot_series = ''
    if 'rs' in str(robot.perform(context)):
        robot_series = 'rs'
    if 'bx' in str(robot.perform(context)):
        robot_series = 'bx'
    if 'bxp' in str(robot.perform(context)):
        robot_series = 'bxp'
    if 'wd' in str(robot.perform(context)):
        robot_series = 'wd'

    # The hardware component name has to be underscored instead of dashed.
    robot_model: str = robot.perform(context)
    hardware_component_name: str = robot_model.replace('-', '_')
    # The hardware component name is substituted inside the `controllers.yaml`.
    context.launch_configurations['hardware_component_name'] = hardware_component_name

    controllers_file = PathJoinSubstitution(
        [FindPackageShare('khi_ros2_icon_hwm'), 'config', 'controllers.yaml']
    )

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('khi_description'), 'urdf', 'khi.urdf.xacro']
            ),
            ' controller_no:=',
            controller_no,
            ' robot_controller:=',
            robot_controller,
            ' robot_ip:=',
            robot_ip,
            ' robot_name:=',
            robot,
            ' robot_series:=',
            robot_series,
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

    khi_operational_state_node = Node(
        package='khi_ros2_icon_hwm',
        executable='khi_operational_state_node',
        parameters=[
            {
                'controller_no': controller_no,
            }
        ],
        output='screen',
    )

    nodes_to_start: List[Node] = [
        robot_state_publisher_node,
        control_node,
        khi_operational_state_node,
    ] + controller_spawner_nodes
    return nodes_to_start


def generate_launch_description() -> LaunchDescription:
    """
    Generate the launch description for the KHI ROS 2 controller.

    Declares all available launch arguments that can be passed via the command
    line, and returns a `LaunchDescription` containing these arguments alongside
    an `OpaqueFunction` that defers node creation to `launch_setup`.
    """
    declared_arguments: List[DeclareLaunchArgument] = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'controller_no',
            choices=[
                '0',
                # '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
                # '10', '11', '12', '13', '14', '15'
            ],
            default_value='0',
            description=(
                'Controller number if connecting multiple controllers from a '
                'single PC.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'controller_spawner_timeout',
            default_value='10',
            description='Timeout used when spawning controllers.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot',
            choices=[
                'rs007l-b001',
                'rs015x-a001',
                'rs013n-a001',
                'rs025n-a001',
                'rs080n-a001',
                'bx300l-b001',
                'bxp135x-a001',
                'wd003h-f502',
                'bxp210l-a001',
            ],
            description='Robot model.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_controller',
            choices=['f', 'f_duaro'],
            default_value='f',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_ip',
            description='IP address by which the robot can be reached.',
        )
    )
    # The following arguments are required by the ICON HWM controller.
    declared_arguments.extend(get_icon_hwm_launch_arguments())

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
