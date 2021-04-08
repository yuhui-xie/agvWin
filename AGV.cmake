if(WIN32)
set(AGV_PATH "${CMAKE_SOURCE_DIR}/AGVtoolkit2013")
else(WIN32)
SET(AGV_PATH "/usr/local")
endif()

include_directories(${AGV_PATH}/include)
link_directories(${AGV_PATH}/lib)


if(WIN32)
SET(AGV_LIBS node libprotobuf libzmq)
else(WIN32)
SET(AGV_LIBS node protobuf zmq rt pthread)
endif()


macro(add_message)  
    set(_args ${ARGN})        
    foreach(_var ${ARGN})  
        execute_process(COMMAND python ${AGV_PATH}/bin/msgc.py ${CMAKE_CURRENT_SOURCE_DIR}/${_var} OUTPUT_VARIABLE result)

        if(result)
            message(FATAL_ERROR ${result})
        endif() 
    endforeach()  
    AUX_SOURCE_DIRECTORY("msg" MSG_SRC)
endmacro()  




