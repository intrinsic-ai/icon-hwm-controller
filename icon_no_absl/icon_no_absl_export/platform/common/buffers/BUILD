load("//bazel:cc_macros.bzl", "cc_library", "cc_test")
load("//icon/testing:cc_test_and_malloc_test.bzl", "cc_test_and_malloc_test")
# Specialized Buffer Implementations

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rt_queue",
    hdrs = ["rt_queue.h"],
    deps = [
        ":rt_queue_buffer",
        "//icon/utils:attributes",
        "//icon/utils:realtime_guard",
    ],
)

# intrinsic:*:strip_begin
cc_test_and_malloc_test(
    name = "rt_queue_test",
    size = "small",
    srcs = ["rt_queue_test.cc"],
    deps = [
        ":rt_queue",
        "//icon/testing:malloc_test",
        "//icon/utils:time",
    ],
)
# intrinsic:*:strip_end

cc_library(
    name = "rt_queue_buffer",
    hdrs = ["rt_queue_buffer.h"],
    visibility = ["//visibility:private"],
    deps = [
        "//icon/utils:attributes",
    ],
)

cc_library(
    name = "rt_promise",
    hdrs = ["rt_promise.h"],
    deps = [
        ":rt_queue",
        "//icon/interprocess:binary_futex",
        "//icon/interprocess:lockable_binary_futex",
        "//icon/testing:realtime_annotations",
        "//icon/utils:attributes",
        "//icon/utils:log",
        "//icon/utils:status",
        "//icon/utils:status_and_expected_macros",
        "//icon/utils:status_helpers",
        "@tl-expected//:tl-expected",
    ],
)

# intrinsic:*:strip_begin
cc_test_and_malloc_test(
    name = "rt_promise_test",
    srcs = ["rt_promise_test.cc"],
    deps = [
        ":rt_promise",
        "//icon/testing:malloc_test",
        "//icon/utils:log",
        "//icon/utils:mock_log_sink",
        "//icon/utils:status",
        "//icon/utils:status_and_expected_test_macros",
        "//icon/utils:time",
    ],
)
# intrinsic:*:strip_end

cc_test(
    name = "rt_queue_buffer_test",
    size = "small",
    srcs = ["rt_queue_buffer_test.cc"],
    deps = [
        ":rt_queue_buffer",
        "//icon/utils:time",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "rt_queue_multi_writer",
    hdrs = ["rt_queue_multi_writer.h"],
    deps = [
        ":rt_queue",
        "//icon/utils:attributes",
        "//icon/utils:realtime_guard",
        "//icon/utils:status",
    ],
)

cc_test(
    name = "rt_queue_multi_writer_test",
    srcs = ["rt_queue_multi_writer_test.cc"],
    deps = [
        ":rt_queue",
        ":rt_queue_multi_writer",
        "//icon/utils:status_and_expected_test_macros",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)
