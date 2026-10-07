cmake_minimum_required(VERSION 3.20)
project(autonomy_scenario_validator VERSION 2.0.0 LANGUAGES CXX)

if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()
include(CTest)
option(ASV_WITH_OPENCV "Build headless playback and classical vision demos" OFF)
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
add_executable(asv_benchmark benchmarks/validation_benchmark.cpp)
target_include_directories(asv_benchmark PRIVATE apps)
target_link_libraries(asv_benchmark PRIVATE asv::core)

if(ASV_WITH_OPENCV)
    find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs videoio)
    add_library(asv_vision src/vision/renderer.cpp src/vision/lane_detector.cpp)
    target_include_directories(asv_vision SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(asv_vision PUBLIC asv::core ${OpenCV_LIBS})
    add_executable(scenario_visualize apps/scenario_visualize.cpp)
    target_link_libraries(scenario_visualize PRIVATE asv_vision)
    add_executable(lane_detection_demo apps/lane_detection_demo.cpp)
    target_link_libraries(lane_detection_demo PRIVATE asv_vision)
endif()

if(BUILD_TESTING)
    include(FetchContent)
    FetchContent_Declare(googletest
        URL https://codeload.github.com/google/googletest/tar.gz/refs/tags/v1.17.0
        URL_HASH SHA256=65fab701d9829d38cb77c14acdc431d2108bfdbf8979e40eb8ae567edf10b27c)
    set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
    add_executable(asv_tests tests/core_tests.cpp tests/io_tests.cpp)
    target_link_libraries(asv_tests PRIVATE asv::core GTest::gtest_main)
    include(GoogleTest)
    gtest_discover_tests(asv_tests)
    add_test(NAME cli_integration COMMAND ${CMAKE_COMMAND}
        -DCLI=$<TARGET_FILE:scenario_playback> -DSOURCE=${CMAKE_CURRENT_SOURCE_DIR}
        -DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/cli-report.json -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/cli_integration.cmake)
    if(ASV_WITH_OPENCV)
        add_executable(asv_vision_tests tests/vision_tests.cpp)
        target_link_libraries(asv_vision_tests PRIVATE asv_vision GTest::gtest_main)
        gtest_discover_tests(asv_vision_tests)
        add_test(NAME vision_cli_integration COMMAND ${CMAKE_COMMAND}
            -DLANE=$<TARGET_FILE:lane_detection_demo> -DVISUALIZE=$<TARGET_FILE:scenario_visualize>
            -DSOURCE=${CMAKE_CURRENT_SOURCE_DIR} -DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/vision-cli
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/vision_cli.cmake)
    endif()
endif()
