cmake_minimum_required (VERSION 2.6) 
project (slam)
SET(CMAKE_BUILD_TYPE Release)
set( CMAKE_CXX_FLAGS " -O3" )


include_directories(utils)
include_directories(sensor)
include_directories(log)
include_directories(configfile)
include_directories(scanmatcher)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}")


add_library(utils STATIC  utils/stat.cpp  utils/movement.cpp)

add_library(sensor sensor/sensor_base/sensor.cpp sensor/sensor_base/sensorreading.cpp
 sensor/sensor_range/rangesensor.cpp sensor/sensor_range/rangereading.cpp
 sensor/sensor_odometry/odometrysensor.cpp sensor/sensor_odometry/odometryreading.cpp)


add_library( log
log/configuration.cpp log/carmenconfiguration.cpp log/sensorlog.cpp log/sensorstream.cpp )

add_library(configfile configfile/configfile.cpp)

add_library(scanmatcher scanmatcher/smmap.cpp  scanmatcher/scanmatcher.cpp 
		scanmatcher/scanmatcherprocessor.cpp scanmatcher/eig3.cpp)

add_library(gridfastslam gridfastslam/gridslamprocessor_tree.cpp  
		gridfastslam/motionmodel.cpp  gridfastslam/gridslamprocessor.cpp  gridfastslam/gfsreader.cpp)

add_library(slam SHARED slam.cpp ../msg/stdmsg.cc)


target_link_libraries(slam gridfastslam scanmatcher configfile log  sensor  utils
 node ${AGV_LIBS})

add_executable(map2png carmen/carmenmap.cpp carmen/main.cpp )
target_link_libraries(map2png node ${AGV_LIBS})
