# BUILD file for esmini external dependency

cc_library(
    name = "esminiLib",
    srcs = ["lib/libesminiLib.so"],
    hdrs = glob([
        "include/esminiLib.hpp",
        "include/esminiRMLib.hpp",
    ]),
    includes = ["include"],
    visibility = ["//visibility:public"],
    strip_include_prefix = "include",
)

cc_library(
    name = "esminiRMLib",
    srcs = ["lib/libesminiRMLib.so"],
    hdrs = glob([
        "include/esminiRMLib.hpp",
    ]),
    includes = ["include"],
    visibility = ["//visibility:public"],
    strip_include_prefix = "include",
)

# Prebuilt binaries for testing/demonstration
filegroup(
    name = "esmini_bin",
    srcs = ["bin/esmini"],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "resources",
    srcs = glob([
        "resources/**/*",
    ]),
    visibility = ["//visibility:public"],
)
