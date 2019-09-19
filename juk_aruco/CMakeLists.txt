cmake_minimum_required(VERSION 2.7)
project(juk_aruco)

set(LIBRARIES_FROM_REFERENCES "")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/../bin)

find_package(roscpp REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})

find_package(image_transport REQUIRED)
include_directories(${image_transport_INCLUDE_DIRS})

find_package(cv_bridge REQUIRED)
include_directories(${cv_bridge_INCLUDE_DIRS})

find_package(OpenCV 3 REQUIRED )
message(${OpenCV_LIBRARIES})

find_package(GeoMath REQUIRED)
include_directories(${GeoMath_INCLUDE_DIRS})

find_package(juk_msg REQUIRED)
include_directories( ${juk_msg_INCLUDE_DIRS})

add_executable(juk_aruco juk_aruco.cpp)

target_link_libraries(juk_aruco  ${roscpp_LIBRARIES}  ${image_transport_LIBRARIES}  ${cv_bridge_LIBRARIES} /usr/local/lib/libGeoMath_lib.a ${OpenCV_LIBRARIES} "${LIBRARIES_FROM_REFERENCES}")

