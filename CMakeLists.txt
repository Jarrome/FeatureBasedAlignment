cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(3D_REGISTRATION)
#find pcl libraries
set(CMAKE_BUILD_TYPE Debug)
#SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -march=native")

find_package(PCL 1.9 REQUIRED)
#include files
include_directories(${PCL_INCLUDE_DIRS})
#lib files
link_directories(${PCL_LIBRARY_DIRS})
#definitions
add_definitions(${PCL_DEFINITIONS})

#necessary libs

################################
#Alignment library
set(alignment_sources
    alignment.h                 alignment.cxx
    util.h                      util.cxx
    )
add_library(pcl_alignment ${alignment_sources})
target_link_libraries(pcl_alignment  ${PCL_LIBRARIES})

################################
#Executables
add_executable(align align_exe.cxx)
target_link_libraries(align pcl_alignment  ${PCL_LIBRARIES})

add_executable(corrs save_correspondences.cxx)
target_link_libraries(corrs pcl_alignment  ${PCL_LIBRARIES})

#add_executable(visualize visualize.cxx)
#target_link_libraries(visualize pcl_alignment ${pcl_libs})

#add_executable(compare compare_correspondences.cxx)
#target_link_libraries(compare pcl_alignment ${pcl_libs})

#add_executable(bbox bbox.cxx)
#target_link_libraries(bbox pcl_alignment ${pcl_libs})

#add_executable(crop cropper.cxx)
#target_link_libraries(crop pcl_alignment ${pcl_libs})

#add_executable(distance distance.cxx)
#target_link_libraries(distance pcl_alignment ${pcl_libs})


