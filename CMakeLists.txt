cmake_minimum_required(VERSION 3.10)
project(Arm_kinematics)

include_directories(src/eigen)
add_subdirectory(src/orocos_kinematics_dynamics/orocos_kdl EXCLUDE_FROM_ALL)
set(KDL_PATH "src/orocos_kinematics_dynamics/orocos_kdl")
include_directories(${KDL_PATH}/src)
include_directories(${KDL_PATH}/utilities)
# Path to generated config.h
include_directories(${CMAKE_CURRENT_BINARY_DIR}/src/orocos_kinematics_dynamics/orocos_kdl/src)

file(GLOB kdl_SRC
    "${KDL_PATH}/src/*.hpp"
    "${KDL_PATH}/src/*.cpp"
    "${KDL_PATH}/src/*.inl"
    "${KDL_PATH}/src/utilities/*.cpp"
    "${KDL_PATH}/src/utilities/*.cxx"
    "${KDL_PATH}/src/utilities/*.h"
)

add_library(Arm_kinematics SHARED src/models.hpp src/reachy.cpp src/pollen_kinematics.cpp ${kdl_SRC})

set_target_properties(Arm_kinematics PROPERTIES PUBLIC_HEADER src/pollen_kinematics.hpp)