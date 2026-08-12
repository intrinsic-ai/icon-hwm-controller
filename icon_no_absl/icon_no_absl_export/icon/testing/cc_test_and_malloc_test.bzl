"""Macro to set up two cc_test from one source with and without malloc counting."""  # intrinsic:*:strip
# intrinsic:*:insert """Malloc test is not implemented externally, only invoke regular cc_test."""

load("//bazel:cc_macros.bzl", "cc_test")

def cc_test_and_malloc_test(name, deps = [], local_defines = [], tags = [], **kwargs):
    # intrinsic:*:strip_begin
    """Convenience macro for a C++ test with and without malloc counting.

    Because malloc counting requires some specific dependencies, and should not run in fastbuild
    mode (some things allocate memory in fastbuild, but do not in opt), we create two separate
    cc_test targets here:
    * One with the given name that skips malloc checking entirely
    * One configured for malloc counting
    Both tests can share the same source code when formulating malloc test expectations using
    intrinsic/icon/testing/malloc_test.h

    Args:
      name: The rule's name, usually ends with _test. The extra malloc test will use the same name
            with the suffix '_malloc' appended to it.
      deps: Specify all deps except gunit_main.
      local_defines: Additional local_defines.
        INTRINSIC_MALLOC_TEST=1 will be set for the malloc counting test.
      tags: Additional tags.
        noasan, nomsan, notsan are added for the malloc test, because those
        are already covered by the regular test.
      **kwargs: srcs is required.
        All other tags/visibility/compatible_with/etc. apply to both rules.
    """

    # intrinsic:*:strip_end
    cc_test(
        name = name,
        local_defines = local_defines,
        tags = tags,
        deps = deps + [
            "@com_google_googletest//:gtest",
            "@com_google_googletest//:gtest_main",
        ],
        **kwargs
    )

    # intrinsic:*:strip_begin

    # TODO(b/542544362): Uncomment implementation once MallocGuard is available via
    # BCR

    # tags_set = sets.make(tags + [
    #     # Some dependencies allocate memory when they're *not* built with `-c opt`
    #     "nofastbuild",
    #     # Sanitizers and coverage are already covered by the test above, and
    #     # malloc checking is disabled in these modes anyways.
    #     "notsan",
    # ])
    # cc_test(
    #     name = name + "_malloc",
    #     local_defines = local_defines + ["INTRINSIC_MALLOC_TEST=1"],
    #     tags = sets.to_list(tags_set),
    #     deps = deps + [
    #         "@com_google_googletest//:gtest",
    #         "//icon/testing:malloc_test_main",
    #     ],
    #     **kwargs
    # )
    # intrinsic:*:strip_end
