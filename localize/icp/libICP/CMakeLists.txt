# project
cmake_minimum_required (VERSION 2.6)
project (libicp)

set (LIBICP_SRC_DIR src)

#FIND_PATH(BOOST_DIR "boost")

# include directory
include_directories("${LIBICP_SRC_DIR}")
#include_directories("${BOOST_DIR}")

FILE(GLOB LIBICP_SRC_FILES "src/*.cpp")

add_library(icp ${LIBICP_SRC_FILES})

