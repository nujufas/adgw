cmake_minimum_required(VERSION 3.10)
project(acc_someip_service)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# CommonAPI paths
set(COMMONAPI_INSTALL_PATH "/usr" CACHE PATH "Path to CommonAPI installation")
set(COMMONAPI_SOMEIP_INSTALL_PATH "/usr" CACHE PATH "Path to CommonAPI-SomeIP installation")
set(VSOMEIP_INSTALL_PATH "/usr" CACHE PATH "Path to vsomeip installation")

# Find required packages
find_package(PkgConfig REQUIRED)
pkg_check_modules(COMMONAPI REQUIRED CommonAPI>=3.2)
pkg_check_modules(COMMONAPI_SOMEIP REQUIRED CommonAPI-SomeIP>=3.2)
pkg_check_modules(VSOMEIP3 REQUIRED vsomeip3)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src-gen
    ${COMMONAPI_INCLUDE_DIRS}
    ${COMMONAPI_SOMEIP_INCLUDE_DIRS}
    ${VSOMEIP3_INCLUDE_DIRS}
)

# Link directories
link_directories(
    ${COMMONAPI_LIBRARY_DIRS}
    ${COMMONAPI_SOMEIP_LIBRARY_DIRS}
    ${VSOMEIP3_LIBRARY_DIRS}
)

# Generated source files (will be created by code generator)
set(GENERATED_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src-gen/v1/com/kpit/acc/services/accInputServiceInterfaceProxy.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src-gen/v1/com/kpit/acc/services/accInputServiceInterfaceSomeIPProxy.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src-gen/v1/com/kpit/acc/services/accInputServiceInterfaceStubDefault.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src-gen/v1/com/kpit/acc/services/accInputServiceInterfaceSomeIPStubAdapter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src-gen/v1/com/kpit/acc/services/accInputServiceInterfaceSomeIPDeployment.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src-gen/v1/com/kpit/acc/services/accOutputServiceInterfaceProxy.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src-gen/v1/com/kpit/acc/services/accOutputServiceInterfaceSomeIPProxy.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src-gen/v1/com/kpit/acc/services/accOutputServiceInterfaceStubDefault.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src-gen/v1/com/kpit/acc/services/accOutputServiceInterfaceSomeIPStubAdapter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src-gen/v1/com/kpit/acc/services/accOutputServiceInterfaceSomeIPDeployment.cpp
)

# Server executable
add_executable(accServer
    src/accServer/accServerMain.cpp
    src/accServer/accInputServiceStubImpl.cpp
    ${GENERATED_SOURCES}
)

target_link_libraries(accServer
    ${COMMONAPI_LIBRARIES}
    ${COMMONAPI_SOMEIP_LIBRARIES}
    ${VSOMEIP3_LIBRARIES}
    pthread
)

# Install targets
install(TARGETS accServer DESTINATION bin)
install(FILES config/vsomeip.json DESTINATION /etc)
install(FILES config/commonapi.ini DESTINATION /etc)

# Code generation target (requires CommonAPI generators)
add_custom_target(generate
    COMMAND echo "Run code generation manually with:"
    COMMAND echo "  commonapi-core-generator-linux-x86_64 -sk -d src-gen fidl/accServices.fidl"
    COMMAND echo "  commonapi-someip-generator-linux-x86_64 -d src-gen fdepl/accServices-someip.fdepl"
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
