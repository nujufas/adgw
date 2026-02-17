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

cc_test(
    name = "test_mock_simulator",
    srcs = ["test_mock_simulator.cpp"],
    deps = [
        "//middleware/simulator:mock_simulator",
        "//middleware/data:data",
    ],
)

cc_test(
    name = "test_udp_adapter",
    srcs = ["test_udp_adapter.cpp"],
    deps = [
        "//middleware/ipc:udp_adapter",
        "//middleware/data:data",
    ],
)

cc_test(
    name = "test_middleware_engine",
    srcs = ["test_middleware_engine.cpp"],
    deps = [
        "//middleware/core:middleware_engine",
        "//middleware/simulator:mock_simulator",
        "//middleware/ipc:udp_adapter",
        "//middleware/data:data",
    ],
)

