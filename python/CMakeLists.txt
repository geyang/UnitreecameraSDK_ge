cmake_minimum_required( VERSION 2.8 )
project(unitree_camera_sdk)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bins/)

find_package(OpenCV 4 REQUIRED)
if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIR})
    message(STATUS "OpenCV ${OpenCV_VERSION} FOUND")
    message(STATUS ${OpenCV_LIBS})
    if(${OpenCV_VERSION_MAJOR} EQUAL 4)
        add_definitions(-DOPENCV_VERSION_4)
    elseif(${OpenCV_VERSION_MAJOR} LESS 4)
        add_definitions(-DOPENCV_VERSION_3)
    endif()
else()
    message(FATAL_ERROR "OpenCV Library Not Found")
endif()

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib/amd64/)
link_directories(${PROJECT_SOURCE_DIR}/lib/amd64/)
else()
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib/arm64/)
link_directories(${PROJECT_SOURCE_DIR}/lib/arm64/)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
include_directories(${PROJECT_SOURCE_DIR}/include)

set(SDKLIBS unitree_camera tstc_V4L2_xu_camera udev systemlog ${OpenCV_LIBS})

find_package(pybind11 CONFIG 2.3 REQUIRED)
pybind11_add_module(unitree_camera_sdk bindings.cpp)

# add_executable(unitree_camera_sdk ${SDKLIBS})
target_link_libraries(unitree_camera_sdk ${OpenCV_LIBS})
#set_target_properties(${SDKLIBS} PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

#

#include_directories(${PROJECT_SOURCE_DIR}/include)
# link_directories(../lib/amd64)
# add_library(unitree_camera STATIC ${CMAKE_SOURCE_DIR}/lib/arm64/libunitree_camera.a)
#set_target_properties(libunitreecam PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
target_link_libraries(unitree_camera_sdk ${CMAKE_SOURCE_DIR}/lib/amd64/libunitree_camera.a)


install(TARGETS unitree_camera_sdk DESTINATION lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
