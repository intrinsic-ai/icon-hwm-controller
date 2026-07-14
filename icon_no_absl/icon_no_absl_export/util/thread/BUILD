load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//google3/bazel:cc_macros.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lockstep",
    srcs = ["lockstep.cc"],
    hdrs = ["lockstep.h"],
    deps = [
        "//icon/interprocess:binary_futex",
        "//icon/utils:log",
        "//icon/utils:status",
        "//icon/utils:time",
    ],
)

cc_test(
    name = "lockstep_test",
    srcs = ["lockstep_test.cc"],
    deps = [
        ":lockstep",
        "//icon/utils:log",
        "//icon/utils:status",
        "//icon/utils:status_and_expected_test_macros",
        "//icon/utils:time",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)
