cmake_minimum_required(VERSION 3.10)

project(Godot_Haptic_Plugin VERSION 1.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)

add_library(Godot_Haptic_Plugin SHARED ${PROJECT_SOURCE_DIR}/src/HapticsPlugin.cpp)


target_include_directories(Godot_Haptic_Plugin PUBLIC ./src/HapticsPlugin.h)
include_directories(${PROJECT_SOURCE_DIR}/include/Eigen)
include_directories(${PROJECT_SOURCE_DIR}/include/CHAI3D)
include_directories(${PROJECT_SOURCE_DIR}/include/glm-0.9.9.0)
