# ACC SomeIP Service - Bazel Build Configuration
# Builds the FIDL-based SomeIP service for ACC integration

load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load(":commonapi_codegen.bzl", "commonapi_library")

package(default_visibility = ["//visibility:public"])

# Code generation from FIDL files using custom Starlark rule
commonapi_library(
    name = "acc_services_codegen",
    fidl = "fidl/accServices.fidl",
    fdepl = "fdepl/accServices-someip.fdepl",
    core_generator = "//third_party/commonapi:core_generator",
    someip_generator = "//third_party/commonapi:someip_generator",
)

# Generated code library
cc_library(
    name = "acc_services_generated",
    srcs = [
        ":acc_services_codegen",  # All .cpp files from codegen
    ],
    hdrs = [
        ":acc_services_codegen",  # All .hpp files from codegen
    ],
    includes = ["acc_services_codegen"],
    strip_include_prefix = "acc_services_codegen",
    deps = [
        "//third_party/commonapi",
        "//third_party/commonapi:commonapi-someip",
    ],
)

# ACC Server application stub implementation
cc_library(
    name = "acc_server_impl",
    srcs = [
        "src/accServer/accInputServiceStubImpl.cpp",
    ],
    hdrs = [
        "src/accServer/accInputServiceStubImpl.hpp",
    ],
    deps = [
        ":acc_services_generated",
        "//third_party/commonapi",
    ],
)

# ACC Server executable
cc_binary(
    name = "accServer",
    srcs = [
        "src/accServer/accServerMain.cpp",
    ],
    deps = [
        ":acc_server_impl",
        ":acc_services_generated",
        "//third_party/commonapi",
        "//third_party/commonapi:commonapi-someip",
    ],
    linkopts = ["-pthread"],
)

# Export configuration files
exports_files([
    "config/vsomeip.json",
    "config/commonapi.ini",
])
