cmake_minimum_required(VERSION 3.10)

project(Godot_Haptic_Plugin VERSION 1.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)


add_library(Godot_Haptic_Plugin SHARED ${PROJECT_SOURCE_DIR}/src/HapticsPlugin.cpp)

target_include_directories(Godot_Haptic_Plugin PRIVATE
   ${PROJECT_SOURCE_DIR}/src
   ${PROJECT_SOURCE_DIR}/include/glm-0.9.9.0  # Use variable from find_package
   ${PROJECT_SOURCE_DIR}/include/Eigen
   ${PROJECT_SOURCE_DIR}/include/CHAI3D
)

target_link_libraries(Godot_Haptic_Plugin PRIVATE
   ${PROJECT_SOURCE_DIR}/lib/libchai3d.a
   ${PROJECT_SOURCE_DIR}/lib/libchai3d.so.3
)

add_executable(Test_Haptics_Plugin ${PROJECT_SOURCE_DIR}/test/TestHapticPlugin.cpp)
target_link_libraries(Test_Haptics_Plugin PRIVATE Godot_Haptic_Plugin)
