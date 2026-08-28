from typing import List

from launch.actions import DeclareLaunchArgument


def get_icon_hwm_launch_arguments(
    default_shm_namespace: str = '',
    default_context_name: str = '',
    default_lock_memory: str = 'true',
    default_cpu_affinity: str = '[0]',
    default_realtime_priority_low: str = '-1',
    default_realtime_priority_high: str = '-1',
    default_control_frequency_hz: str = '-1',
    default_drives_realtime_clock: str = 'true',
) -> List[DeclareLaunchArgument]:
    """Get the list of DeclareLaunchArgument actions required for the ICON HWM Controller."""
    declared_arguments: List[DeclareLaunchArgument] = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'hwm_name',
            description=(
                'The ICON hardware module name that this Controller uses to talk to the '
                'ICON service. The name has to be unique across all hardware module instances '
                'that are connecting to the same ICON instance. '
                'Shared memory modules are indexed by this name.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'shm_namespace',
            default_value=default_shm_namespace,
            description='The shared memory namespace. If empty, uses the default namespace.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'context_name',
            default_value=default_context_name,
            description=(
                'The context name. Used for error reporting. This is typically the instance name '
                'of the module under which the hardware module is shown in Flowstate. '
                'If omitted, this defaults to `hwm_name`.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'lock_memory',
            default_value=default_lock_memory,
            choices=['true', 'false', 'True', 'False'],
            description=(
                'Whether or not to lock the memory in realtime threads that this controller '
                'spawns. On non-realtime kernels, this option has no effect.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'cpu_affinity',
            default_value=default_cpu_affinity,
            description=(
                'An array of CPU cores to pin the ControllerManager to. '
                'If empty, the ControllerManager will not be pinned to any CPU core.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'realtime_priority_low',
            default_value=default_realtime_priority_low,
            description=(
                "The realtime priority of the controller's low priority threads. "
                'If -1, all controller threads will run at default priority.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'realtime_priority_high',
            default_value=default_realtime_priority_high,
            description=(
                "The realtime priority of the controller's high priority threads. "
                'If -1, all controller threads will run at default priority.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'control_frequency_hz',
            default_value=default_control_frequency_hz,
            description=(
                'The realtime control frequency from the hardware module configuration. '
                'If this is greater than zero, it must match the update rate of the '
                'ControllerManager.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'drives_realtime_clock',
            default_value=default_drives_realtime_clock,
            choices=['true', 'false', 'True', 'False'],
            description=(
                "Does this hardware module drive ICON's realtime clock? "
                "If true, then the hardware module's Init method will be provided a "
                'RealtimeClockInterface (via module_config.GetRealtimeClock()), and the '
                'hardware module is expected to call TickBlocking every control cycle. '
                'If false, then the hardware module_config.GetRealtimeClock() will return '
                'nullptr. Should always be true.'
            ),
        )
    )

    return declared_arguments
