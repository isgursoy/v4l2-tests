cmake_minimum_required(VERSION 3.10)

set(CMAKE_C_COMPILER "clang")
set(CMAKE_CXX_COMPILER "clang++")

project(V4l2_Test)


set(ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(DEVICE TRUE)


if(NOT CMAKE_SYSTEM_PROCESSOR)
    set(ARCHITECTURE "x86_64")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
    set(ARCHITECTURE "larch64")
else()
    set(ARCHITECTURE ${CMAKE_SYSTEM_PROCESSOR})
endif()

set(SOURCES
    "${ROOT_DIR}/Abstract_Capture_Backend.hpp"
    "${ROOT_DIR}/isgursoy_V4L2.hpp"
    "${ROOT_DIR}/main.cpp"
)

add_executable(${PROJECT_NAME} ${SOURCES})

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)

target_compile_options(${PROJECT_NAME} PRIVATE -ftemplate-backtrace-limit=0)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(${PROJECT_NAME} PRIVATE
        -fcolor-diagnostics
     #   -fexperimental-library
        -stdlib=libstdc++
        -fopenmp=libgomp
        -std=c++20
    )
    set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_LIBRARY "libstdc++")
endif()

#if(ARCHITECTURE MATCHES "arch64")
#    target_compile_options(${PROJECT_NAME} PRIVATE -mcpu=cortex-a57)
#endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(${PROJECT_NAME} PRIVATE -O0)
    target_compile_definitions(${PROJECT_NAME} PRIVATE DEBUGGING)
else()
    target_compile_options(${PROJECT_NAME} PRIVATE -Ofast)
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE m pthread png OpenCL atomic)

target_compile_definitions(${PROJECT_NAME} PRIVATE LIBPNG_AVAILABLE)
if(DEVICE)
    target_compile_definitions(${PROJECT_NAME} PRIVATE ON_DEVICE)
endif()

target_compile_options(${PROJECT_NAME} PRIVATE
    -Wno-designated-init
    -Wall
    -Wextra
    -Wc++20-compat
    -Wno-error=deprecated-declarations
    -Wno-unknown-warning-option
    -Wno-deprecated-register
    -Wno-inconsistent-missing-override
    -Wno-c99-designator
    -Wno-reorder-init-list
    -Wno-error=unused-but-set-variable
)

message(STATUS "${PROJECT_NAME} ${PROJECT_VERSION} is being configured")
