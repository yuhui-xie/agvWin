cmake_minimum_required (VERSION 2.6) 
project (local_plan)

#set(CMAKE_C_COMPILER gcc)
add_library(local_plan local_plan.cpp )
if(UNIX)
target_link_libraries(local_plan m)
endif(UNIX)
