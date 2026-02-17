# Simulator components

load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "simulator_interface",
    hdrs = ["simulator_interface.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//middleware/data:data",
    ],
)

cc_library(
    name = "mock_simulator",
    srcs = ["mock_simulator.cpp"],
    hdrs = ["mock_simulator.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":simulator_interface",
        "//middleware/data:data",
    ],
)

cc_library(
    name = "esmini_adapter",
    srcs = ["esmini_adapter.cpp"],
    hdrs = ["esmini_adapter.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":simulator_interface",
        "//middleware/data:data",
        "//third_party/esmini:esminiLib",
    ],
)

cc_library(
    name = "simulator",
    visibility = ["//visibility:public"],
    deps = [
        ":simulator_interface",
        ":mock_simulator",
        ":esmini_adapter",
    ],
)
