cmake_minimum_required(VERSION 3.20)
project(autonomy_scenario_validator VERSION 2.0.0 LANGUAGES CXX)

if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()
include(CTest)
option(ASV_WARNINGS_AS_ERRORS "Treat project compiler warnings as errors" OFF)
option(ASV_ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(asv_options INTERFACE)
target_compile_features(asv_options INTERFACE cxx_std_17)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(asv_options INTERFACE -Wall -Wextra -Wpedantic)
    if(ASV_WARNINGS_AS_ERRORS)
        target_compile_options(asv_options INTERFACE -Werror)
    endif()
    if(ASV_ENABLE_SANITIZERS)
        target_compile_options(asv_options INTERFACE -fsanitize=address,undefined -fno-omit-frame-pointer)
        target_link_options(asv_options INTERFACE -fsanitize=address,undefined)
    endif()
elseif(MSVC)
    target_compile_options(asv_options INTERFACE /W4)
    if(ASV_WARNINGS_AS_ERRORS)
        target_compile_options(asv_options INTERFACE /WX)
    endif()
    if(ASV_ENABLE_SANITIZERS)
        message(FATAL_ERROR "ASV_ENABLE_SANITIZERS requires GCC or Clang")
    endif()
endif()


add_library(asv
    src/core/report.cpp src/core/validation.cpp src/geometry/obb.cpp
    src/io/csv_reader.cpp src/io/numeric.cpp src/io/scenario_loader.cpp
    src/validators/speed_limit.cpp src/validators/acceleration_limit.cpp
    src/validators/jerk_limit.cpp src/validators/lane_boundary.cpp
    src/validators/collision_aabb.cpp src/validators/collision_obb.cpp src/validators/ttc.cpp)
add_library(asv::core ALIAS asv)
target_include_directories(asv PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(asv PUBLIC asv_options)
add_executable(scenario_playback apps/scenario_playback.cpp)
target_link_libraries(scenario_playback PRIVATE asv::core)
