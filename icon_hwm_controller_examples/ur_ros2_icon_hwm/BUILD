load("@ai_intrinsic_sdks//bazel:container.bzl", "container_image", "container_import")
load("@ai_intrinsic_sdks//bazel:python_oci_image.bzl", "python_layers")
load("@ai_intrinsic_sdks//intrinsic/assets/services/build_defs:services.bzl", "intrinsic_service")
load("@ai_intrinsic_sdks//intrinsic/icon/hal/bzl:resources.bzl", "hardware_module_manifest")

package(default_visibility = ["//visibility:public"])

container_import(
    name = "ur_ros2_icon_hwm_oci",
    tarball = "icon_hwm.tar",
)
launcher_layers = python_layers(
    name = "launcher_layers",
    binary = "//icon_hwm_controller:launcher_bin",
)
container_image(
    name = "ur_ros2_icon_hwm_image",
    base = ":ur_ros2_icon_hwm_oci",
    entrypoint = ["/icon_hwm_controller/launcher_bin"],
    layers = launcher_layers,
)

hardware_module_manifest(
    name = "ur_ros2_icon_hwm_manifest",
    image = ":ur_ros2_icon_hwm_image.tar",
    image_sim = ":ur_ros2_icon_hwm_image.tar",
    manifest = "ur_ros2_icon_hwm_manifest.textproto",
)
intrinsic_service(
    name = "ur_ros2_icon_hwm_service",
    default_config = "ur_ros2_icon_hwm_default_config.textproto",
    images = [
        ":ur_ros2_icon_hwm_image.tar",
    ],
    manifest = ":ur_ros2_icon_hwm_manifest",
    deps = [
        "//icon_hwm_controller:ros2_hwm_config_proto",
        "@ai_intrinsic_sdks//intrinsic/assets/services/proto/v1:service_state_proto",
        "@ai_intrinsic_sdks//intrinsic/icon/hal:hardware_module_config_proto",
    ],
)
