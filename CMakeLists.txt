cmake_minimum_required(VERSION 3.12)
project(autonomy_scenario_validator CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(asv
  src/core/report.cpp
  src/io/csv_reader.cpp
  src/io/scenario_loader.cpp
  src/validators/speed_limit.cpp
  src/validators/acceleration_limit.cpp
  src/validators/lane_boundary.cpp
  src/validators/collision_aabb.cpp
)

target_include_directories(asv PUBLIC include)

add_executable(scenario_playback apps/scenario_playback.cpp)
target_link_libraries(scenario_playback PRIVATE asv)

add_executable(smoke_test tests/smoke_test.cpp)
target_link_libraries(smoke_test PRIVATE asv)