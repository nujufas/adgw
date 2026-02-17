# Test targets

load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_test")

cc_test(
    name = "test_data_structures",
    srcs = ["test_data_structures.cpp"],
    deps = [
        "//middleware/data:data",
        "//middleware/utils:serialization",
    ],
)

cc_binary(
    name = "size_report",
    srcs = ["size_report.cpp"],
    deps = [
        "//middleware/data:data",
    ],
)
