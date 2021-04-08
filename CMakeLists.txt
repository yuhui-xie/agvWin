cmake_minimum_required (VERSION 2.6) 
project (nav)
SET(CMAKE_BUILD_TYPE Release)
set( CMAKE_CXX_FLAGS " -O3" )
set( CMAKE_CXX_FLAGS "/Mt" )
SET(EXECUTABLE_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/bin/")
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}/lib )
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}/lib )
link_directories(${CMAKE_ARCHIVE_OUTPUT_DIRECTORY} ${CMAKE_SOURCE_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})

set(PYTHON_EXECUTABLE D:/Python-2.7.5/python.exe) 
set(PYTHON_INCLUDE_DIR D:/Python-2.7.5/include) 
set(PYTHON_LIBRARY D:/Python-2.7.5/libs/python27.lib)


include(AGV.cmake)
add_message(./msg/stdmsg.proto)
add_library(stdmsg ./msg/stdmsg.cc)

include_directories(msg slam ${CMAKE_CURRENT_SOURCE_DIR})
add_subdirectory(global_plan)
add_subdirectory(local_plan)
add_subdirectory(localize)
add_subdirectory(driver)
add_subdirectory(slam)
add_subdirectory(route_planner)

add_library(robot  robot.cpp)
target_link_libraries(robot driver ${AGV_LIBS} stdmsg)

add_executable(localizer localizer.cpp )
TARGET_LINK_LIBRARIES(localizer localize ${AGV_LIBS} robot slam stdmsg)

add_executable(navigator navigator.cpp )
TARGET_LINK_LIBRARIES(navigator route_planner local_plan global_plan ${AGV_LIBS} robot slam stdmsg)

add_executable(explore explorer.cpp )
TARGET_LINK_LIBRARIES(explore  robot slam)

#add_executable(follow follow.cpp )
#TARGET_LINK_LIBRARIES(follow  ${AGV_LIBS} stdmsg)

add_executable(gui_connect gui_connect.cpp )
TARGET_LINK_LIBRARIES(gui_connect  ${AGV_LIBS} stdmsg)

#add_executable(battery_charge battery_charge.cpp )
#TARGET_LINK_LIBRARIES(battery_charge  ${AGV_LIBS} stdmsg)

option (LASER_TEST "whether build the laser test code" OFF)
if(LASER_TEST)
    #add_executable(localplan_test test/localplan_test.cpp )
    #TARGET_LINK_LIBRARIES(localplan_test local_plan global_plan ${AGV_LIBS} stdmsg)

    add_executable(laser_test laser_test.cpp )
    TARGET_LINK_LIBRARIES(laser_test ${AGV_LIBS} stdmsg driver)
endif(LASER_TEST)

option (ZEROMQ_TEST "whether build the laser test code" OFF)
if(ZEROMQ_TEST)
    add_executable(receive_test test/receive_test.cpp )
    TARGET_LINK_LIBRARIES(receive_test ${AGV_LIBS} stdmsg)
    add_executable(send_test test/send_test.cpp )
    TARGET_LINK_LIBRARIES(send_test ${AGV_LIBS} stdmsg)
endif(ZEROMQ_TEST)



macro(copy from to )
    file(COPY "${CMAKE_SOURCE_DIR}/${from}" DESTINATION "${CMAKE_SOURCE_DIR}/${to}")
endmacro()
    
#copy("msg/stdmsg.py"    "python/gui" )
#copy("python/driver"    "bin" )
#copy("python/gui"       "bin" )
#copy("navigator.ini"    "bin" )
