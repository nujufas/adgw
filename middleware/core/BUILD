load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "middleware_engine",
    srcs = ["middleware_engine.cpp"],
    hdrs = ["middleware_engine.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//middleware/simulator:simulator_interface",
        "//middleware/ipc:ipc_interface",
        "//middleware/data:data",
    ],
)
