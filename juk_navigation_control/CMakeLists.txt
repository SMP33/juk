#Generated by VisualGDB project wizard.
#Note: VisualGDB will automatically update this file when you add new sources to the project.

cmake_minimum_required(VERSION 2.7)
project(juk_navigation_control)
set(LIBRARIES_FROM_REFERENCES "")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/../bin)


find_package(roscpp REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})

find_package(GeoMath REQUIRED)
include_directories(${GeoMath_INCLUDE_DIRS})

find_package(juk_msg REQUIRED)
include_directories( ${juk_msg_INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)

FILE(GLOB SOURCE_CPP ../include/*.cpp *.cpp)

add_executable(juk_navigation_control
${SOURCE_CPP}
NavigationNode.h)

#target_include_directories ( juk_navigation_control PUBLIC /home/ubuntu/catkin_ws/devel/include/juk_msg)
target_link_libraries(juk_navigation_control /usr/local/lib/libGeoMath_lib.a ${roscpp_LIBRARIES} "${LIBRARIES_FROM_REFERENCES}")

