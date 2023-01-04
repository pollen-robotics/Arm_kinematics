cmake_minimum_required(VERSION 3.10)
project(Arm_kinematics)

include_directories(src/eigen)
include_directories(src/kdl)
include_directories(src/kdl/utilities)

file(GLOB kdl_SRC
    "src/kdl/*.hpp"
    "src/kdl/*.cpp"
    "src/kdl/*.inl"
    "src/kdl/utilities/*.cpp"
    "src/kdl/utilities/*.cxx"
    "src/kdl/utilities/*.h"
    "src/kdl/models/*.cpp"
    "src/kdl/models/*.hpp"
)

add_library(Arm_kinematics SHARED src/pollen_kinematics.cpp ${kdl_SRC})

set_target_properties(Arm_kinematics PROPERTIES PUBLIC_HEADER src/pollen_kinematics.hpp)