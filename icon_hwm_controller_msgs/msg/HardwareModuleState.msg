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

# StateCode enum constants
uint8 DEACTIVATED = 0
uint8 DEACTIVATING = 1
uint8 ACTIVATED = 2
uint8 ACTIVATING = 3
uint8 MOTION_ENABLED = 4
uint8 MOTION_ENABLING = 5
uint8 MOTION_DISABLING = 6
uint8 FAULTED = 7
uint8 CLEARING_FAULTS = 8
uint8 INIT_FAILED = 9
uint8 FATALLY_FAULTED = 10
uint8 PREPARING = 11
uint8 PREPARED = 12

uint8 code
uint8[256] message
