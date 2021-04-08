cmake_minimum_required (VERSION 2.6) 
project (driver)
SET(CMAKE_BUILD_TYPE Release)
set( CMAKE_CXX_FLAGS " -O3" )

option (BUILD_PYTHON_PLUGIN "whether use python as driver" OFF)
set(BUILD_PYTHON_PLUGIN ON)
if(BUILD_PYTHON_PLUGIN)
    find_package(PythonLibs REQUIRED)
    include_directories(${PYTHON_INCLUDE_DIRS})
    link_directories(${PYTHON_LINK_DIRS})
    add_definitions(-DPYTHON_PLUGIN)
endif(BUILD_PYTHON_PLUGIN)

include_directories("LaserSrc")

AUX_SOURCE_DIRECTORY("net" NETSRC)
FILE(GLOB LASERSRC "LaserSrc/*.h*" "LaserSrc/*.c*")
FILE(GLOB LASERREMOVE "LaserSrc/*windows*" "LaserSrc/*linux*")
LIST(REMOVE_ITEM LASERSRC ${LASERREMOVE})
add_library(driver motion.cpp Laser.cpp Laser.h ${NETSRC} ${LASERSRC})
if(WIN32)
TARGET_LINK_LIBRARIES(driver    ${PYTHON_LIBRARIES})
else(WIN32)
TARGET_LINK_LIBRARIES(driver  ${PYTHON_LIBRARIES}  rt)
endif(WIN32)

#www.cnblogs.com/lidabo/p/3976989.html