load("//bazel:cc_macros.bzl", "cc_library")

package(
    default_visibility = [
        "//incode/icon/no_absl:__subpackages__",
    ],
)

cc_library(
    name = "realtime_clock_interface",
    srcs = ["realtime_clock_interface.cc"],
    hdrs = ["realtime_clock_interface.h"],
    visibility = [
        "//incode/icon/no_absl:__subpackages__",
    ],
    deps = [
        "//icon/utils:status",
        "//icon/utils:time",
    ],
)
