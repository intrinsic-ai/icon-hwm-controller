from typing import Dict, List

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
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
    Evaluates runtime launch configurations and initializes the ROS 2 nodes.

    This function is called as an OpaqueFunction, allowing it to access the
    runtime LaunchContext. It uses this context to evaluate string values
    for launch configurations (like robot_model) to dynamically build the
    URDF via Xacro and configure the controller nodes.

    Args:
        context: The current launch context containing runtime variables.

    Returns:
        A list containing the robot_state_publisher, ros2_control_node,
        and controller spawner nodes to be started.
    """
    robot_model = LaunchConfiguration("robot_model")
    robot_series = LaunchConfiguration("robot_series")
    robot_ip = LaunchConfiguration("robot_ip")
    prefix = LaunchConfiguration("prefix")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")

    robot_model_str: str = robot_model.perform(context)
    robot_series_str: str = robot_series.perform(context)

    if robot_series_str == "crx":
        urdf_xacro_file: str = robot_model_str + ".urdf.xacro"
    else:
        urdf_xacro_file: str = "6dof_robot.urdf.xacro"
    
    controllers_file = PathJoinSubstitution(
        [FindPackageShare("fanuc_ros2_icon_hwm"), "config", "controllers.yaml"]
    )

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("fanuc_hardware_interface"), "robot", urdf_xacro_file]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            "robot_series:=",
            robot_series,
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "robot_model:=",
            robot_model,
        ]
    )
    robot_description: Dict[str, ParameterValue] = {
        "robot_description": ParameterValue(value=robot_description, value_type=str)
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ParameterFile(controllers_file, allow_substs=True),
        ],
        output="screen",
    )

    def controller_spawner(controllers: List[str], active: bool = True) -> Node:
        """
        Helper function to generate a controller manager spawner node.

        Args:
            controllers: A list of controller names to be spawned.
            active: If True, spawns the controller in an active state.
                    If False, appends the '--inactive' flag. Defaults to True.

        Returns:
            The configured spawner node for the provided controllers.
        """
        inactive_flags: List[str] = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",
            executable="spawner",
            parameters=[
                {"verify_payload_on_set": True},
                # We substitute the arguments into the controller configuration.
                ParameterFile(controllers_file, allow_substs=True),
            ],
            arguments=[
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                controller_spawner_timeout,
            ]
            + inactive_flags
            + controllers,
        )

    controllers_active: List[str] = [
        "joint_state_broadcaster",
    ]
    # The ICON controller has to be started in an inactive state. The controller
    # is designed to self-activate during Prepare().
    controllers_inactive: List[str] = [
        "icon_controller",
    ]
    controller_spawner_nodes: List[Node] = [
        controller_spawner(controllers_active, active=True),
        controller_spawner(controllers_inactive, active=False),
    ]

    nodes_to_start: List[Node] = [
        robot_state_publisher_node,
        control_node,
    ] + controller_spawner_nodes
    return nodes_to_start


def generate_launch_description() -> LaunchDescription:
    """
    Main entry point for the ROS 2 launch file.

    Declares all available launch arguments that can be passed via the command line,
    and returns a LaunchDescription containing these arguments alongside an
    OpaqueFunction that defers node creation to `launch_setup`.

    Returns:
        The launch description.
    """
    declared_arguments: List[DeclareLaunchArgument] = []

    # The following arguments are required by the FANUC ROS 2 driver.
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_model",
            description="The robot model.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_series",
            default_value="crx",
            description='The robot series such as "crx".',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            description="IP address by which the robot can be reached."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "control_frequency",
            description="The control frequency in Hz that the controller should run with.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix of the joint names, useful for multi-robot setup. "
            "If changed, also the joint names in the controllers' configuration "
            "have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="10",
            description="Timeout used when spawning controllers.",
        )
    )

    # The following arguments are required by the ICON HWM controller.
    declared_arguments.append(
        DeclareLaunchArgument(
            "hwm_name",
            description="The ICON hardware module name that this controller uses to talk to ICON.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "shm_namespace",
            default_value="",
            description="The shared memory namespace. If empty, uses the default namespace.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "context_name",
            default_value="",
            description="The context name used for error reporting.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "lock_memory",
            default_value="True",
            description="Whether or not to lock the memory in realtime threads that this controller spawns.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "cpu_affinity",
            default_value="[0]",
            description="An array of CPU cores to pin the ControllerManager to. If empty, the ControllerManager will not be pinned to any CPU core.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "realtime_priority_low",
            default_value="-1",
            description="The realtime priority of the controller's low priority threads.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "realtime_priority_high",
            default_value="-1",
            description="The realtime priority of the controller's high priority threads.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'control_frequency_hz',
            default_value='-1',
            description=(
                'The realtime control frequency from the hardware module configuration. '
                'If this is greater than zero, it must match the update rate of the '
                'ControllerManager.'),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "drives_realtime_clock",
            default_value="true",
            description="Does this hardware module drive ICON's realtime clock?",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
