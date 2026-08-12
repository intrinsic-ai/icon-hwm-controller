load("//bazel:cc_macros.bzl", "cc_library", "cc_test")

package(default_visibility = [
    "//incode/icon/no_absl:__subpackages__",
])

cc_library(
    name = "binary_futex",
    srcs = ["binary_futex.cc"],
    hdrs = [
        "binary_futex.h",
    ],
    deps = [
        "//icon/utils:attributes",
        "//icon/utils:status",
        "//icon/utils:strerror",
        "//icon/utils:time",
    ],
)

cc_test(
    name = "binary_futex_test",
    size = "small",
    srcs = ["binary_futex_test.cc"],
    deps = [
        ":binary_futex",
        "//icon/utils:status",
        "//icon/utils:status_and_expected_test_macros",
        "//icon/utils:time",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "lockable_binary_futex",
    hdrs = [
        "lockable_binary_futex.h",
    ],
    deps = [
        ":binary_futex",
        "//icon/utils:attributes",
        "//icon/utils:status",
        "//icon/utils:strerror",
        "//icon/utils:time",
    ],
)

cc_test(
    name = "lockable_binary_futex_test",
    size = "small",
    srcs = ["lockable_binary_futex_test.cc"],
    deps = [
        ":lockable_binary_futex",
        "//icon/utils:status",
        "//icon/utils:status_and_expected_test_macros",
        "//icon/utils:time",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)
