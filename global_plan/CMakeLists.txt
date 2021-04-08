cmake_minimum_required (VERSION 2.6) 
project (global_plan)

#set(CMAKE_C_COMPILER gcc)
add_library(global_plan global_plan.cpp conventional.c planner.c trajectory.c map_modify.c)
if(UNIX)
target_link_libraries(global_plan m)
endif(UNIX)
