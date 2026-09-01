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

# Required for transitive dependencies in downstream packages.
# For more details see https://docs.ros.org/en/jazzy/How-To-Guides/Ament-CMake-Documentation.html#basic-project-outline
find_package(Eigen3 REQUIRED)
find_package(flatbuffers REQUIRED)
find_package(icon_shared_memory REQUIRED)
find_package(tl-expected REQUIRED)

