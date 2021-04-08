cmake_minimum_required (VERSION 2.6) 
project (route_planner)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}")

add_library(KDTree KDTree.cpp)
add_library(AStar AStar.cpp)
target_link_libraries(AStar KDTree)
add_library(route_planner route_planner.cpp)
target_link_libraries(route_planner AStar KDTree)

