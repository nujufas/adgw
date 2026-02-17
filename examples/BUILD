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

cc_binary(
    name = "esmini_demo",
    srcs = ["esmini_demo.cpp"],
    data = [
        "//third_party/esmini:resources",
    ],
    deps = [
        "//middleware/core:middleware_engine",
        "//middleware/simulator:esmini_adapter",
        "//middleware/ipc:udp_adapter",
    ],
)
