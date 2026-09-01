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

load("@ai_intrinsic_sdks//bazel:container.bzl", "container_image", "container_import")
load("@ai_intrinsic_sdks//bazel:python_oci_image.bzl", "python_layers")
load("@ai_intrinsic_sdks//intrinsic/assets/services/build_defs:services.bzl", "intrinsic_service")
load("@ai_intrinsic_sdks//intrinsic/icon/hal/bzl:resources.bzl", "hardware_module_manifest")

package(default_visibility = ["//visibility:public"])

container_import(
    name = "fanuc_ros2_icon_hwm_oci",
    # This image currently has to be exported manually using docker image save.
    tarball = "icon_hwm.tar",
)
entrypoint_layers = python_layers(
    name = "entrypoint_layers",
    binary = "//icon_hwm_controller:entrypoint_bin",
)
container_image(
    name = "fanuc_ros2_icon_hwm_image",
    base = ":fanuc_ros2_icon_hwm_oci",
    entrypoint = ["/icon_hwm_controller/entrypoint_bin"],
    layers = entrypoint_layers,
)

hardware_module_manifest(
    name = "fanuc_ros2_icon_hwm_manifest",
    image = ":fanuc_ros2_icon_hwm_image.tar",
    image_sim = ":fanuc_ros2_icon_hwm_image.tar",
    manifest = "proto/fanuc_ros2_icon_hwm_manifest.textproto",
)
intrinsic_service(
    name = "fanuc_ros2_icon_hwm_service",
    default_config = "proto/fanuc_ros2_icon_hwm_default_config.textproto",
    images = [
        ":fanuc_ros2_icon_hwm_image.tar",
    ],
    manifest = ":fanuc_ros2_icon_hwm_manifest",
    deps = [
        "//icon_hwm_controller:ros2_hwm_config_proto",
        "@ai_intrinsic_sdks//intrinsic/assets/services/proto/v1:service_state_proto",
        "@ai_intrinsic_sdks//intrinsic/icon/hal:hardware_module_config_proto",
    ],
)
