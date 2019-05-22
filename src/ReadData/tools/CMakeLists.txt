cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

set(PROJECT_COLORS pcl_add_colors)

project(${PROJECT_COLORS})

set(PROJECT_COLORS_SRCS colors.cpp)
set(PROJECT_COLORS_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/../src)


find_package(PCL REQUIRED)

include_directories(${PROJECT_COLORS_INCLUDE} ${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})


add_executable(${PROJECT_COLORS} ${PROJECT_COLORS_SRCS})

target_link_libraries(${PROJECT_COLORS} ${PCL_LIBRARIES})



