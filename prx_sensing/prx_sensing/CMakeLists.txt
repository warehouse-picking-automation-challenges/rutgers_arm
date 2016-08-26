
cmake_minimum_required(VERSION 2.8.3)
project(prx_sensing)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_BUILD_TYPE Debug)
add_definitions(-DCPU_ONLY=1)

# Catkin package dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp 
  roslib 
  std_msgs 
  sensor_msgs 
  geometry_msgs 
  pcl_ros
  image_transport
  message_filters 
  cv_bridge 
  message_generation 
  prx_utilities
  tf
)

find_package(simtrack_nodes)
if (EXISTS "${simtrack_nodes_DIR}/../../../include/simtrack_nodes/UpdateObjectList.h")
  MESSAGE( STATUS "Simtrack Loc: " ${simtrack_nodes_DIR})
  MESSAGE( STATUS "Simtrack GH FOUND")
  add_definitions(-DSIMTRACK_GH_FOUND)
endif()

# System dependencies
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  ObjectPose.msg
)

add_service_files(
  FILES
  UpdateObjectList.srv
  PublishObjectList.srv
  UpdateShelfPosition.srv
  SwitchCameras.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs geometry_msgs sensor_msgs
)

catkin_package(
  INCLUDE_DIRS "prx"
  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS 
    roscpp 
    roslib 
    std_msgs 
    sensor_msgs 
    geometry_msgs 
    pcl_ros 
    message_filters
    image_transport 
    cv_bridge
    prx_utilities 
    tf
  )




# All the included search paths
include_directories(
  ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
)
# include_directories(${CAFFE_INCLUDEDIR})

include_directories(${PROJECT_SOURCE_DIR}/../../prx_utilities)

# PRACSYS includes
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/prx/)
include_directories(${PROJECT_SOURCE_DIR}/prx/ShelfDetect)
include_directories(${PROJECT_SOURCE_DIR}/prx/ObjectDetect)

# Executables and libraries
#link_directories(${CAFFE_LINK_LIBRARAY})

# file(GLOB_RECURSE SRC_SENSING ${PROJECT_SOURCE_DIR}/prx/sense/*.cpp)
add_library(
			${PROJECT_NAME}_lib 
			${PROJECT_SOURCE_DIR}/prx/main.cpp
			${PROJECT_SOURCE_DIR}/prx/ShelfDetect/apc_shelf.cpp
			${PROJECT_SOURCE_DIR}/prx/ObjectDetect/apc_object.cpp
			#${PROJECT_SOURCE_DIR}/prx/ObjectDetect/classifier.cpp
			${PROJECT_SOURCE_DIR}/prx/sensing_utilities.cpp)
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES} ${prx_utilities_LIBRARIES} )

# add_executable(forward_toy_car prx/applications/forward_toy_car.cpp)
# target_link_libraries(forward_toy_car ${PROJECT_NAME}_lib )

# add_executable(data_publisher prx/applications/data_publisher.cpp)
# target_link_libraries(data_publisher ${PROJECT_NAME}_lib )

add_executable(prx_sensing ${PROJECT_SOURCE_DIR}/prx/main.cpp)
target_link_libraries(prx_sensing ${PROJECT_NAME}_lib)

##
## TESTS AND INSTALL
##
install( DIRECTORY "prx"
         DESTINATION "include"
         FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
         )

install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
