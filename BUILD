exports_files(["0001-intrinsic-sdk-don-t-compress-python-OCI-layers.patch"])

filegroup(
    name = "all_sources",
    srcs = glob(
        include = ["**"],
        exclude = [
            ".git/**",
            "MODULE.bazel.lock",
        ],
    ),
    visibility = ["//visibility:public"],
)
