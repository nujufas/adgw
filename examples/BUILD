load("@rules_cc//cc:defs.bzl", "cc_binary")

cc_binary(
    name = "simple_demo",
    srcs = ["simple_demo.cpp"],
    deps = [
        "//middleware/core:middleware_engine",
        "//middleware/simulator:mock_simulator",
        "//middleware/ipc:udp_adapter",
    ],
)
