cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(MY_GRAND_PROJECT)

set(INCLUDE "${CMAKE_SOURCE_DIR}/include")

find_package(PCL)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



add_executable (alignment_prerejective src/alignment_prerejective.cpp)
add_executable (ply2pcd src/ply2pcd.cpp)
add_executable (alignment_ransac src/alignment_ransac.cpp)
add_executable (corr_recog src/corr_recog.cpp)


target_link_libraries(alignment_prerejective ${PCL_LIBRARIES})
target_link_libraries(ply2pcd ${PCL_LIBRARIES})
target_link_libraries(alignment_ransac ${PCL_LIBRARIES})
target_link_libraries(corr_recog ${PCL_LIBRARIES})