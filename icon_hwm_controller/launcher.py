#!/usr/bin/env python3

import os
import sys

from intrinsic.icon.hal.proto import hardware_module_config_pb2
from intrinsic.resources.proto import runtime_context_pb2

try:
    from icon_hwm_controller.proto import ros2_hwm_config_pb2
except ImportError:
    import ros2_hwm_config_pb2


def main():
    print('--------------------------------')
    print('-- ROS2 HWM Launcher Starting')
    print('--------------------------------')

    config_path = '/etc/intrinsic/runtime_config.pb'
    if not os.path.exists(config_path):
        print(f'Runtime config file not found at {config_path}')
        sys.exit(1)

    with open(config_path, 'rb') as fin:
        context = runtime_context_pb2.RuntimeContext.FromString(fin.read())

    # Unpack HardwareModuleConfig
    hw_module_config = hardware_module_config_pb2.HardwareModuleConfig()
    if not context.config.Unpack(hw_module_config):
        print('Failed to unpack RuntimeContext.config into HardwareModuleConfig')
        sys.exit(1)

    # Unpack Ros2HwmConfig from HardwareModuleConfig.module_config
    ros2_hwm_config = ros2_hwm_config_pb2.Ros2HwmConfig()
    if not hw_module_config.module_config.Unpack(ros2_hwm_config):
        print('Failed to unpack HardwareModuleConfig.module_config into Ros2HwmConfig')
        sys.exit(1)

    print(f'Loaded Launch Package: {ros2_hwm_config.launch_package}')
    print(f'Loaded Launch File: {ros2_hwm_config.launch_file}')
    print(f'Launch Parameters: {ros2_hwm_config.launch_parameters}')

    # Construct the ros2 launch command
    cmd = [
        'ros2',
        'launch',
        ros2_hwm_config.launch_package,
        ros2_hwm_config.launch_file,
    ]

    # Append launch arguments for IconHwmController parameters
    icon_cfg = ros2_hwm_config.icon_hwm_controller_config

    if len(hw_module_config.realtime_cores) > 0:
        cores_str = f"[{','.join(str(c) for c in hw_module_config.realtime_cores)}]"
    else:
        cores_str = '[]'
    cmd.append(f'cpu_affinity:={cores_str}')
    if hw_module_config.name:
        cmd.append(f'hwm_name:={hw_module_config.name}')
    if icon_cfg.shm_namespace:
        cmd.append(f'shm_namespace:={icon_cfg.shm_namespace}')
    context_name = (
        hw_module_config.context_name
        if hw_module_config.context_name
        else hw_module_config.name
    )
    if context_name:
        cmd.append(f'context_name:={context_name}')

    if hw_module_config.HasField('cycle_time'):
        cycle_sec = hw_module_config.cycle_time.seconds + hw_module_config.cycle_time.nanos * 1e-9
        if cycle_sec > 0:
            freq = int(round(1.0 / cycle_sec))
            cmd.append(f'control_frequency_hz:={freq}')

    drives_clock = 'true' if hw_module_config.drives_realtime_clock else 'false'
    cmd.append(f'drives_realtime_clock:={drives_clock}')
    lock_memory = 'true' if icon_cfg.lock_memory else 'false'
    cmd.append(f'lock_memory:={lock_memory}')
    # Unset proto values come out as the default, 0. Translate that to -1 to signal the
    # controller to use default priorities
    realtime_priority_low = icon_cfg.realtime_priority_low
    if realtime_priority_low == 0:
        realtime_priority_low = -1
    realtime_priority_high = icon_cfg.realtime_priority_high
    if realtime_priority_high == 0:
        realtime_priority_high = -1
    cmd.append(f'realtime_priority_low:={realtime_priority_low}')
    cmd.append(f'realtime_priority_high:={realtime_priority_high}')

    # Append additional launch_parameters
    for key, value in ros2_hwm_config.launch_parameters.items():
        cmd.append(f'{key}:={value}')

    print(f'Executing Command: {" ".join(cmd)}')

    # Source install/setup.bash to make sure that all packages are visible
    ros_distro = os.environ.get('ROS_DISTRO', 'kilted')
    ament_ws_dir = os.environ.get('AMENT_WORKSPACE_DIR', '/ament_ws')
    ros_setup = f'/opt/ros/{ros_distro}/setup.bash'
    ament_setup = os.path.join(ament_ws_dir, 'install/setup.bash')

    cmd_str = ' '.join(cmd)
    bash_cmd = (
        'export PYTHONUNBUFFERED=1 && '
        'export RCUTILS_LOGGING_BUFFERED_STREAM=0 && '
        f'source {ros_setup} && '
        f'source {ament_setup} && '
        f'{cmd_str}'
    )

    print(f'Final Bash Command: {bash_cmd}')

    sys.stdout.flush()
    sys.stderr.flush()

    os.execvp('bash', ['bash', '-c', bash_cmd])


if __name__ == '__main__':
    main()
