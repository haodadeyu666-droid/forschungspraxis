cmake_minimum_required(VERSION 3.10)
project(med_motions_experiments_jasmin LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)          # std::filesystem
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# libfranka: several installs exist on this machine. Override with
#   cmake -DFranka_DIR=<path/to/cmake/Franka> ..
# Default: the ROS Noetic package (consistent 0.9.2 headers + lib + config).
set(Franka_DIR "/opt/ros/noetic/lib/x86_64-linux-gnu/cmake/Franka"
    CACHE PATH "Directory containing FrankaConfig.cmake")
find_package(Franka 0.9.2 REQUIRED)
message(STATUS "Using libfranka ${Franka_VERSION} from ${Franka_DIR}")

find_package(Eigen3 REQUIRED NO_MODULE)

# Shared motion code: S-curve planning, move-to-start, insertion/retraction, logging.
add_library(injection_common STATIC injection_common.cpp)
target_include_directories(injection_common PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(injection_common PUBLIC Franka::Franka Eigen3::Eigen)
# Data is saved relative to the project folder, no matter where the program is started.
target_compile_definitions(injection_common PRIVATE PROJECT_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_options(injection_common PRIVATE -Wall -Wextra -Wreturn-type)

# One program per start orientation; they differ only in the start pose.
foreach(program injection_0deg injection_45deg)
  add_executable(${program} ${program}.cpp)
  target_link_libraries(${program} PRIVATE injection_common)
  target_compile_options(${program} PRIVATE -Wall -Wextra -Wreturn-type)
endforeach()
