#!/usr/bin/env python3

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


import argparse
import logging
import os
import sys
from typing import Dict, Tuple

from icon_hwm_controller.proto import ros2_hwm_config_pb2
from intrinsic.icon.hal.proto import hardware_module_config_pb2
from intrinsic.resources.proto import runtime_context_pb2

logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] [%(name)s] [%(levelname)s] %(message)s',
)
logger = logging.getLogger('icon_hwm_entrypoint')


def load_runtime_config(
    config_path: str,
) -> hardware_module_config_pb2.HardwareModuleConfig:
    """Load and unpack the `HardwareModuleConfig` from the runtime context."""
    if not os.path.exists(config_path):
        raise FileNotFoundError(
            f"Runtime config file not found at '{config_path}'."
        )

    with open(config_path, 'rb') as fin:
        context = runtime_context_pb2.RuntimeContext.FromString(fin.read())

    hw_module_config = hardware_module_config_pb2.HardwareModuleConfig()
    if not context.config.Unpack(hw_module_config):
        raise ValueError(
            'Failed to unpack RuntimeContext.config into HardwareModuleConfig.'
        )

    return hw_module_config


def extract_launch_arguments(
    hw_module_config: hardware_module_config_pb2.HardwareModuleConfig,
) -> Tuple[str, str, Dict[str, str]]:
    """Unpack Ros2HwmConfig and extract target launch package, file, and arguments."""
    ros2_hwm_config = ros2_hwm_config_pb2.Ros2HwmConfig()
    if not hw_module_config.module_config.Unpack(ros2_hwm_config):
        raise ValueError(
            'Failed to unpack HardwareModuleConfig.module_config into Ros2HwmConfig.'
        )

    control_frequency_hz = -1
    rate_type = hw_module_config.WhichOneof('control_rate')
    if rate_type == 'control_frequency_hz':
        if hw_module_config.control_frequency_hz > 0:
            control_frequency_hz = int(round(hw_module_config.control_frequency_hz))
        else:
            logger.warning(
                'Invalid non-positive control_frequency_hz (%s) in HardwareModuleConfig. '
                'Defaulting to -1.',
                hw_module_config.control_frequency_hz,
            )
    elif rate_type == 'control_period_ns':
        if hw_module_config.control_period_ns > 0:
            control_frequency_hz = int(round(1e9 / hw_module_config.control_period_ns))
        else:
            logger.warning(
                'Invalid non-positive control_period_ns (%s) in HardwareModuleConfig. '
                'Defaulting to -1.',
                hw_module_config.control_period_ns,
            )

    context_name = hw_module_config.context_name or hw_module_config.name

    icon_cfg = ros2_hwm_config.icon_hwm_controller_config
    # 0 is default unset in Protobuf. We map it to -1.
    prio_low = icon_cfg.realtime_priority_low if icon_cfg.realtime_priority_low != 0 else -1
    prio_high = icon_cfg.realtime_priority_high if icon_cfg.realtime_priority_high != 0 else -1

    launch_args: Dict[str, str] = {
        'control_frequency_hz': str(control_frequency_hz),
        'drives_realtime_clock': 'true' if hw_module_config.drives_realtime_clock else 'false',
        'lock_memory': 'true' if icon_cfg.lock_memory else 'false',
        'realtime_priority_high': str(prio_high),
        'realtime_priority_low': str(prio_low),
    }
    if hw_module_config.realtime_cores:
        cores = list(hw_module_config.realtime_cores)
        launch_args['cpu_affinity'] = f"[{','.join(str(c) for c in cores)}]"
    if hw_module_config.name:
        launch_args['hwm_name'] = hw_module_config.name
    if context_name:
        launch_args['context_name'] = context_name
    if icon_cfg.shm_namespace:
        launch_args['shm_namespace'] = icon_cfg.shm_namespace

    # Override defaults with non-empty launch_parameters.
    for key, value in ros2_hwm_config.launch_parameters.items():
        if value:
            launch_args[key] = str(value)

    return ros2_hwm_config.launch_package, ros2_hwm_config.launch_file, launch_args


def launch_ros2_hwm(
    launch_package: str,
    launch_file: str,
    launch_arguments: Dict[str, str],
) -> None:
    """
    Execute the ROS 2 launch file via process replacement.

    The entrypoint binary runs inside Bazel's hermetic Python environment,
    which does not bundle system packages (e.g. PyYAML) or ROS 2 launch tools.
    Sourcing the workspace and executing `ros2 launch` via `exec "$@"` hands off
    execution to the native system Python environment where all ROS 2 dependencies
    are available, while ensuring signals (SIGINT/SIGTERM) are delivered directly.
    """
    cmd = ['ros2', 'launch', launch_package, launch_file]
    for key, value in launch_arguments.items():
        if value != '':
            cmd.append(f'{key}:={value}')

    logger.info('Executing: %s', ' '.join(cmd))
    sys.stdout.flush()
    sys.stderr.flush()

    ament_ws_dir = os.environ.get('AMENT_WORKSPACE_DIR', '/ament_ws')
    setup_file = os.path.join(ament_ws_dir, 'install/setup.bash')

    bash_script = f'source {setup_file} && exec "$@"'
    os.execvp('bash', ['bash', '-c', bash_script, '--'] + cmd)


def main():
    parser = argparse.ArgumentParser(
        description='ICON ROS 2 Hardware Module (HWM) Entrypoint'
    )
    parser.add_argument(
        '--config',
        '-c',
        default=os.environ.get(
            'INTRINSIC_RUNTIME_CONFIG', '/etc/intrinsic/runtime_config.pb'
        ),
        help=(
            'Path to the runtime config Protobuf file '
            '(default: /etc/intrinsic/runtime_config.pb)'
        ),
    )
    args = parser.parse_args()

    # Ensure unbuffered stream for realtime logging in containers.
    os.environ['PYTHONUNBUFFERED'] = '1'
    os.environ['RCUTILS_LOGGING_BUFFERED_STREAM'] = '0'

    logger.info('Starting ICON ROS 2 HWM Entrypoint')
    logger.info('Loading runtime configuration from: %s', args.config)

    try:
        hw_module_config = load_runtime_config(args.config)
        launch_package, launch_file, launch_args = extract_launch_arguments(
            hw_module_config
        )
    except Exception as e:
        logger.error('Failed to load runtime configuration: %s', e)
        sys.exit(1)

    logger.info('Loaded Launch Package: %s', launch_package)
    logger.info('Loaded Launch File: %s', launch_file)

    launch_ros2_hwm(
        launch_package=launch_package,
        launch_file=launch_file,
        launch_arguments=launch_args,
    )


if __name__ == '__main__':
    main()
