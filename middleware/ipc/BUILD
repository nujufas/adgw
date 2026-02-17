load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "ipc_interface",
    hdrs = ["ipc_interface.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//middleware/data:vehicle_state",
        "//middleware/data:scene_state",
        "//middleware/data:control_command",
    ],
)

cc_library(
    name = "udp_adapter",
    srcs = ["udp_adapter.cpp"],
    hdrs = ["udp_adapter.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":ipc_interface",
        "//middleware/utils:serialization",
    ],
)
