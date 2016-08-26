cmake_minimum_required(VERSION 2.8.3)
project(prx_learn)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_definitions(-DCPU_ONLY=1)

# Catkin package dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp roslib std_msgs sensor_msgs image_transport cv_bridge
)

set(CAFFE_INCLUDEDIR external/caffe/include external/caffe/distribute/include)
set(CAFFE_LINK_LIBRARAY external/caffe/build/lib)

# System dependencies
find_package(Boost REQUIRED COMPONENTS system)
# find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS "prx"
  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS roscpp roslib std_msgs sensor_msgs image_transport cv_bridge prx_utilities
  )

# All the included search paths
include_directories(
  ${catkin_INCLUDE_DIRS}
#  ${OpenCV_INCLUDE_DIRS}
)
include_directories(${CAFFE_INCLUDEDIR})

# PRACSYS includes
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/../prx_utilities)





# Executables and libraries
#link_directories(${CAFFE_LINK_LIBRARAY})

add_library( caffe SHARED IMPORTED )
set_target_properties( caffe PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/${CAFFE_LINK_LIBRARAY}/libcaffe.so )

file(GLOB_RECURSE SRC_LEARN ${PROJECT_SOURCE_DIR}/prx/learn/*.cpp)
add_library(${PROJECT_NAME}_lib ${SRC_LEARN})
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES} caffe glog protobuf pthread )

add_executable(forward_toy_car prx/applications/forward_toy_car.cpp)
target_link_libraries(forward_toy_car ${PROJECT_NAME}_lib )

add_executable(data_publisher prx/applications/data_publisher.cpp)
target_link_libraries(data_publisher ${PROJECT_NAME}_lib )


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