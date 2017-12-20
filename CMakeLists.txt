cmake_minimum_required(VERSION 2.8.11)
project(v_repExtICP)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MACOSX_RPATH 1)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")

find_package(Boost REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")
find_package(VREP REQUIRED)

include_directories(${VREP_INCLUDE})
include_directories(${CMAKE_BINARY_DIR}/generated)
include_directories(${CMAKE_SOURCE_DIR}/external/libicp/src)
include_directories(${Boost_INCLUDE_DIRS})

vrep_generate_stubs(${CMAKE_BINARY_DIR}/generated XML_FILE ${CMAKE_SOURCE_DIR}/callbacks.xml LUA_FILE ${CMAKE_SOURCE_DIR}/simExtICP.lua)

set(SOURCES
    v_repExtICP.cpp
    external/libicp/src/icp.cpp
    external/libicp/src/icpPointToPlane.cpp
    external/libicp/src/icpPointToPoint.cpp
    external/libicp/src/kdtree.cpp
    external/libicp/src/matrix.cpp
)
add_library(v_repExtICP SHARED ${SOURCES} ${CMAKE_BINARY_DIR}/generated/stubs.cpp "${VREP_EXPORTED_SOURCES}")
target_link_libraries(v_repExtICP ${Boost_LIBRARIES} ${VREP_LIBRARIES})

install(TARGETS v_repExtICP DESTINATION ${VREP_ROOT})

