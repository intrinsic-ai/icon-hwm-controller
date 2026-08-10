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
    name = "launcher_bin",
    srcs = ["launcher.py"],
    main = "launcher.py",
    deps = [
        ":hardware_module_config_py_pb2",
        ":icon_hwm_controller_config_py_pb2",
        ":ros2_hwm_config_py_pb2",
        "@ai_intrinsic_sdks//intrinsic/resources/proto:runtime_context_py_pb2",
        "@com_google_protobuf//:protobuf_python",
    ],
)

