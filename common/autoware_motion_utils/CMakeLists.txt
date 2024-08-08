cmake_minimum_required(VERSION 3.14)
project(autoware_motion_utils)

option(BUILD_EXAMPLES "Build examples" OFF)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Boost REQUIRED)
find_package(fmt REQUIRED)

ament_auto_add_library(autoware_motion_utils SHARED
  DIRECTORY src
)

target_link_libraries(autoware_motion_utils
  fmt::fmt
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)

  file(GLOB_RECURSE test_files test/**/*.cpp)

  ament_add_ros_isolated_gtest(test_autoware_motion_utils ${test_files})

  target_link_libraries(test_autoware_motion_utils
    autoware_motion_utils
  )
endif()

if(BUILD_EXAMPLES)
  include(FetchContent)
  FetchContent_Declare(
    matplotlibcpp17
    GIT_REPOSITORY https://github.com/soblin/matplotlibcpp17.git
    GIT_TAG master
  )
  FetchContent_MakeAvailable(matplotlibcpp17)

  ament_auto_add_executable(example_trajectory_container
    examples/example_trajectory_container.cpp
  )

  target_link_libraries(example_trajectory_container
    autoware_motion_utils
    matplotlibcpp17::matplotlibcpp17
  )
endif()

ament_auto_package()
