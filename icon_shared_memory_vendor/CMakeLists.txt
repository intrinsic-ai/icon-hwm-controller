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

cmake_minimum_required(VERSION 3.10)
project(icon_shared_memory_vendor CXX)

# Default to C++20 standard.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

find_package(eigen_vendor REQUIRED)
find_package(flatbuffers_vendor REQUIRED)
find_package(tl_expected REQUIRED)

ament_vendor(icon_shared_memory_vendor
  VCS_TYPE path
  VCS_URL ./icon_shared_memory
  CMAKE_ARGS
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
)

ament_export_dependencies(
  eigen_vendor
  flatbuffers_vendor
  tl_expected
)

ament_package(
  CONFIG_EXTRAS "cmake/icon_shared_memory_vendor-extras.cmake"
)
