# ACC SomeIP Service - Bazel Build Configuration
# Builds the FIDL-based SomeIP service for ACC integration

load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

package(default_visibility = ["//visibility:public"])

# Code generation from FIDL files
# This genrule runs the CommonAPI generators to create proxy/stub code
genrule(
    name = "generate_commonapi_code",
    srcs = [
        "fidl/accServices.fidl",
        "fdepl/accServices-someip.fdepl",
    ],
    outs = [
        # Core generated files
        "src-gen/v1/com/kpit/acc/services/accInputServiceInterface.hpp",
        "src-gen/v1/com/kpit/acc/services/accInputServiceInterfaceProxy.hpp",
        "src-gen/v1/com/kpit/acc/services/accInputServiceInterfaceProxyBase.hpp",
        "src-gen/v1/com/kpit/acc/services/accInputServiceInterfaceStub.hpp",
        "src-gen/v1/com/kpit/acc/services/accInputServiceInterfaceStubDefault.hpp",
        "src-gen/v1/com/kpit/acc/services/accOutputServiceInterface.hpp",
        "src-gen/v1/com/kpit/acc/services/accOutputServiceInterfaceProxy.hpp",
        "src-gen/v1/com/kpit/acc/services/accOutputServiceInterfaceProxyBase.hpp",
        "src-gen/v1/com/kpit/acc/services/accOutputServiceInterfaceStub.hpp",
        "src-gen/v1/com/kpit/acc/services/accOutputServiceInterfaceStubDefault.hpp",
        
        # Core implementation files
        "src-gen/v1/com/kpit/acc/services/accInputServiceInterfaceProxy.cpp",
        "src-gen/v1/com/kpit/acc/services/accInputServiceInterfaceStubDefault.cpp",
        "src-gen/v1/com/kpit/acc/services/accOutputServiceInterfaceProxy.cpp",
        "src-gen/v1/com/kpit/acc/services/accOutputServiceInterfaceStubDefault.cpp",
        
        # SomeIP generated files
        "src-gen/v1/com/kpit/acc/services/accInputServiceInterfaceSomeIPProxy.hpp",
        "src-gen/v1/com/kpit/acc/services/accInputServiceInterfaceSomeIPProxy.cpp",
        "src-gen/v1/com/kpit/acc/services/accInputServiceInterfaceSomeIPStubAdapter.hpp",
        "src-gen/v1/com/kpit/acc/services/accInputServiceInterfaceSomeIPStubAdapter.cpp",
        "src-gen/v1/com/kpit/acc/services/accInputServiceInterfaceSomeIPDeployment.hpp",
        "src-gen/v1/com/kpit/acc/services/accInputServiceInterfaceSomeIPDeployment.cpp",
        "src-gen/v1/com/kpit/acc/services/accOutputServiceInterfaceSomeIPProxy.hpp",
        "src-gen/v1/com/kpit/acc/services/accOutputServiceInterfaceSomeIPProxy.cpp",
        "src-gen/v1/com/kpit/acc/services/accOutputServiceInterfaceSomeIPStubAdapter.hpp",
        "src-gen/v1/com/kpit/acc/services/accOutputServiceInterfaceSomeIPStubAdapter.cpp",
        "src-gen/v1/com/kpit/acc/services/accOutputServiceInterfaceSomeIPDeployment.hpp",
        "src-gen/v1/com/kpit/acc/services/accOutputServiceInterfaceSomeIPDeployment.cpp",
    ],
    cmd = """
        # Get absolute paths for input files
        FIDL_FILE=$$(realpath $(location fidl/accServices.fidl))
        FDEPL_FILE=$$(realpath $(location fdepl/accServices-someip.fdepl))
        
        #Create output directory in Bazel's build directory
        OUT_DIR=$(@D)/src-gen
        mkdir -p "$$OUT_DIR"
        
        # Copy generator directories to a writable location (needed for Eclipse workspace)
        WORK_DIR=$$(mktemp -d)
        trap "rm -rf $$WORK_DIR" EXIT
        
        # Find and copy core generator with all dependencies
        CORE_GEN_SRC=$$(dirname $$(find . -path "*/generators/core/commonapi-core-generator-linux-x86_64" | head -1))
        cp -r "$$CORE_GEN_SRC" "$$WORK_DIR/core_gen"
        
        # Find and copy someip generator with all dependencies  
        SOMEIP_GEN_SRC=$$(dirname $$(find . -path "*/generators/someip/commonapi-someip-generator-linux-x86_64" | head -1))
        cp -r "$$SOMEIP_GEN_SRC" "$$WORK_DIR/someip_gen"
        
        # Create Eclipse workspace and configuration directories
        mkdir -p "$$WORK_DIR/core_workspace"
        mkdir -p "$$WORK_DIR/core_config"
        mkdir -p "$$WORK_DIR/someip_workspace"
        mkdir -p "$$WORK_DIR/someip_config"
        
        # Run core generator from its directory with workspace and configuration
        cd "$$WORK_DIR/core_gen"
        ./commonapi-core-generator-linux-x86_64 \
            -data "$$WORK_DIR/core_workspace" \
            -configuration "$$WORK_DIR/core_config" \
            -sk -dest "$$OUT_DIR" "$$FIDL_FILE" || exit 1
        
        # Run SomeIP generator from its directory with workspace and configuration
        cd "$$WORK_DIR/someip_gen"
        ./commonapi-someip-generator-linux-x86_64 \
            -data "$$WORK_DIR/someip_workspace" \
            -configuration "$$WORK_DIR/someip_config" \
            -dest "$$OUT_DIR" "$$FDEPL_FILE" || exit 1
        
        # Verify critical files were generated
        if [ ! -f "$$OUT_DIR/v1/com/kpit/acc/services/accInputServiceInterfaceStubDefault.hpp" ]; then
            echo "ERROR: Code generation failed - stub files not found"
            ls -R "$$OUT_DIR"
            exit 1
        fi
        
        echo "✓ Code generation completed successfully in Bazel build directory"
    """,
    tools = [
        "//third_party/commonapi:core_generator",
        "//third_party/commonapi:someip_generator",
    ],
)

# Generated code library
cc_library(
    name = "acc_services_generated",
    srcs = [
        "src-gen/v1/com/kpit/acc/services/accInputServiceInterfaceProxy.cpp",
        "src-gen/v1/com/kpit/acc/services/accInputServiceInterfaceStubDefault.cpp",
        "src-gen/v1/com/kpit/acc/services/accInputServiceInterfaceSomeIPProxy.cpp",
        "src-gen/v1/com/kpit/acc/services/accInputServiceInterfaceSomeIPStubAdapter.cpp",
        "src-gen/v1/com/kpit/acc/services/accInputServiceInterfaceSomeIPDeployment.cpp",
        "src-gen/v1/com/kpit/acc/services/accOutputServiceInterfaceProxy.cpp",
        "src-gen/v1/com/kpit/acc/services/accOutputServiceInterfaceStubDefault.cpp",
        "src-gen/v1/com/kpit/acc/services/accOutputServiceInterfaceSomeIPProxy.cpp",
        "src-gen/v1/com/kpit/acc/services/accOutputServiceInterfaceSomeIPStubAdapter.cpp",
        "src-gen/v1/com/kpit/acc/services/accOutputServiceInterfaceSomeIPDeployment.cpp",
    ],
    hdrs = [
        "src-gen/v1/com/kpit/acc/services/accInputServiceInterface.hpp",
        "src-gen/v1/com/kpit/acc/services/accInputServiceInterfaceProxy.hpp",
        "src-gen/v1/com/kpit/acc/services/accInputServiceInterfaceProxyBase.hpp",
        "src-gen/v1/com/kpit/acc/services/accInputServiceInterfaceStub.hpp",
        "src-gen/v1/com/kpit/acc/services/accInputServiceInterfaceStubDefault.hpp",
        "src-gen/v1/com/kpit/acc/services/accInputServiceInterfaceSomeIPProxy.hpp",
        "src-gen/v1/com/kpit/acc/services/accInputServiceInterfaceSomeIPStubAdapter.hpp",
        "src-gen/v1/com/kpit/acc/services/accInputServiceInterfaceSomeIPDeployment.hpp",
        "src-gen/v1/com/kpit/acc/services/accOutputServiceInterface.hpp",
        "src-gen/v1/com/kpit/acc/services/accOutputServiceInterfaceProxy.hpp",
        "src-gen/v1/com/kpit/acc/services/accOutputServiceInterfaceProxyBase.hpp",
        "src-gen/v1/com/kpit/acc/services/accOutputServiceInterfaceStub.hpp",
        "src-gen/v1/com/kpit/acc/services/accOutputServiceInterfaceStubDefault.hpp",
        "src-gen/v1/com/kpit/acc/services/accOutputServiceInterfaceSomeIPProxy.hpp",
        "src-gen/v1/com/kpit/acc/services/accOutputServiceInterfaceSomeIPStubAdapter.hpp",
        "src-gen/v1/com/kpit/acc/services/accOutputServiceInterfaceSomeIPDeployment.hpp",
    ],
    includes = ["src-gen"],
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
