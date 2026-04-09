cmake_minimum_required(VERSION 3.10.0)
project(pose_test VERSION 0.1.0 LANGUAGES C CXX)

SET(CMAKE_CXX_STANDARD 23)

#Replace these definitions with the paths to the src and include directories. 
set(aer627_src_dir "C:/Users/roche/OneDrive/Documents/GitHub/AER627-RoboCraft/Project4Rover/src/")
set(aer627_inc_dir "C:/Users/roche/OneDrive/Document/GitHub/AER627-RoboCraft/Project4Rover/include/")

#These are two sub-libraries
add_library(vex_apriltag)
add_library(pose_math)

include_directories(${aer627_inc_dir})

target_sources(vex_apriltag PRIVATE  
                ${aer627_src_dir}apriltag_aux.c 
                ${aer627_src_dir}apriltag_pose.c
                ${aer627_src_dir}homography.c
                ${aer627_src_dir}matd.c
                ${aer627_src_dir}svd22.c
                ${aer627_src_dir}zarray.c)

target_sources(pose_math PRIVATE 
                ${aer627_src_dir}Pose3D.cpp
                ${aer627_src_dir}obstacle.cpp) 

add_executable(pose_test src/pose_test.cpp)
target_sources(pose_test PRIVATE src/tag_map.cpp)
target_link_libraries(pose_test PRIVATE vex_apriltag pose_math)

include(CTest)
enable_testing()

