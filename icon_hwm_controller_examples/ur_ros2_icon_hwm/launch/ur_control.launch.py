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

from typing import List

from icon_hwm_controller.launch import get_icon_hwm_launch_arguments
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def launch_setup(context: LaunchContext):
    """
    Evaluate runtime launch configurations and initialize the ROS 2 nodes.

    This function is called as an OpaqueFunction, allowing it to access the
    runtime LaunchContext. It uses this context to evaluate string values for
    launch configurations (like robot_model) to dynamically build the URDF via
    Xacro and configure the controller nodes.
    """
    controller_spawner_timeout = LaunchConfiguration('controller_spawner_timeout')
    robot_ip = LaunchConfiguration('robot_ip')
    ur_type = LaunchConfiguration('ur_type')

    controllers_file = PathJoinSubstitution(
        [FindPackageShare('ur_ros2_icon_hwm'), 'config', 'controllers.yaml']
    )
    description_launchfile = PathJoinSubstitution(
        [FindPackageShare('ur_robot_driver'), 'launch', 'ur_rsp.launch.py']
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            ParameterFile(controllers_file, allow_substs=True),
        ],
        output='screen',
    )
    robot_description_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(description_launchfile),
        launch_arguments={
            'headless_mode': 'true',
            'robot_ip': robot_ip,
            'ur_type': ur_type,
        }.items(),
    )
    dashboard_client_node = IncludeLaunchDescription(
        launch_description_source=AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ur_robot_driver'), 'launch', 'ur_dashboard_client.launch.py']
            )
        ),
        launch_arguments={
            'robot_ip': robot_ip,
        }.items(),
    )
    robot_state_helper_node = Node(
        package='ur_robot_driver',
        executable='robot_state_helper',
        name='ur_robot_state_helper',
        output='screen',
        parameters=[
            {'headless_mode': True},
            {'robot_ip': robot_ip},
        ],
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
        'io_and_status_controller',
    ]
    # The ICON controller has to be started in an inactive state. The controller
    # is designed to self-activate during `Prepare()`.
    controllers_inactive: List[str] = [
        'icon_controller',
    ]
    controller_spawners: List[Node] = [
        controller_spawner(controllers_active, active=True),
        controller_spawner(controllers_inactive, active=False),
    ]

    ur_operational_state_node = Node(
        package='ur_ros2_icon_hwm',
        executable='ur_operational_state_node',
        output='screen',
    )

    nodes_to_start = [
        control_node,
        robot_description_launch,
        dashboard_client_node,
        robot_state_helper_node,
        ur_operational_state_node,
    ] + controller_spawners

    return nodes_to_start


def generate_launch_description() -> LaunchDescription:
    """
    Generate the launch description for the UR ROS 2 controller.

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
            'robot_ip', description='IP address by which the robot can be reached.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'ur_type',
            description='Type/series of used UR robot.',
            choices=[
                'ur3',
                'ur5',
                'ur10',
                'ur3e',
                'ur5e',
                'ur7e',
                'ur10e',
                'ur12e',
                'ur16e',
                'ur8long',
                'ur15',
                'ur18',
                'ur20',
                'ur30',
            ],
        )
    )
    # The following arguments are required by the ICON HWM controller.
    declared_arguments.extend(get_icon_hwm_launch_arguments())

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
