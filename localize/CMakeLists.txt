cmake_minimum_required (VERSION 2.6) 
project (localize)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}")

option (BUILD_ICP "whether build icp" OFF)
if(BUILD_ICP)
    add_subdirectory(icp)
    set(ICP_LIB icp)
else(BUILD_ICP)
    add_definitions(-DNO_ICP)
endif(BUILD_ICP)


AUX_SOURCE_DIRECTORY("particlefilter" PFSRC)
#add_library(particlefilter STATIC  ${PFSRC})


add_library(localize localize.cpp ${PFSRC} )
TARGET_LINK_LIBRARIES(localize ${AGV_LIBS} ${ICP_LIB})


