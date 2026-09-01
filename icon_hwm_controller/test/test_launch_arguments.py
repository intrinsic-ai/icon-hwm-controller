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

from icon_hwm_controller.launch import get_icon_hwm_launch_arguments


def test_get_icon_hwm_launch_arguments_default():
    args = get_icon_hwm_launch_arguments()
    arg_names = [arg.name for arg in args]
    expected_names = [
        'hwm_name',
        'shm_namespace',
        'context_name',
        'lock_memory',
        'cpu_affinity',
        'realtime_priority_low',
        'realtime_priority_high',
        'control_frequency_hz',
        'drives_realtime_clock',
    ]
    assert arg_names == expected_names
    assert len(args) == 9
    # `hwm_name` must be mandatory (no default value).
    hwm_name_arg = next(arg for arg in args if arg.name == 'hwm_name')
    assert hwm_name_arg.default_value is None


def test_get_icon_hwm_launch_arguments_custom():
    args = get_icon_hwm_launch_arguments(
        default_lock_memory='false',
        default_cpu_affinity='[1]',
        default_realtime_priority_low='10',
    )
    arg_dict = {arg.name: arg for arg in args}
    assert arg_dict['hwm_name'].default_value is None
    assert arg_dict['lock_memory'].default_value[0].text == 'false'
    assert arg_dict['cpu_affinity'].default_value[0].text == '[1]'
    assert arg_dict['realtime_priority_low'].default_value[0].text == '10'
