# BUILD file for esmini third-party library

load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "esminiLib",
    srcs = ["bin/libesminiLib.so"],
    hdrs = ["EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp"],
    includes = ["EnvironmentSimulator/Libraries/esminiLib"],
    visibility = ["//visibility:public"],
    strip_include_prefix = "EnvironmentSimulator/Libraries/esminiLib",
)

# Prebuilt esmini executable
filegroup(
    name = "esmini_bin",
    srcs = ["bin/esmini"],
    visibility = ["//visibility:public"],
)

# Resources (scenarios, models, etc.)
filegroup(
    name = "resources",
    srcs = glob([
        "resources/**/*",
    ]),
    visibility = ["//visibility:public"],
)
