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

load("@com_google_protobuf//bazel:proto_library.bzl", "proto_library")
load("@com_google_protobuf//bazel:py_proto_library.bzl", "py_proto_library")
load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "icon_hwm_controller_config_proto",
    srcs = ["proto/icon_hwm_controller_config.proto"],
)
py_proto_library(
    name = "icon_hwm_controller_config_py_pb2",
    deps = [":icon_hwm_controller_config_proto"],
)
proto_library(
    name = "ros2_hwm_config_proto",
    srcs = ["proto/ros2_hwm_config.proto"],
    deps = [
        ":icon_hwm_controller_config_proto",
    ],
)
py_proto_library(
    name = "ros2_hwm_config_py_pb2",
    deps = [":ros2_hwm_config_proto"],
)
# The SDK currently does not provide a Python Protobuf library.
py_proto_library(
    name = "hardware_module_config_py_pb2",
    deps = ["@ai_intrinsic_sdks//intrinsic/icon/hal:hardware_module_config_proto"],
)

py_binary(
    name = "entrypoint_bin",
    srcs = ["entrypoint.py"],
    main = "entrypoint.py",
    deps = [
        ":hardware_module_config_py_pb2",
        ":icon_hwm_controller_config_py_pb2",
        ":ros2_hwm_config_py_pb2",
        "@ai_intrinsic_sdks//intrinsic/resources/proto:runtime_context_py_pb2",
        "@com_google_protobuf//:protobuf_python",
    ],
)

