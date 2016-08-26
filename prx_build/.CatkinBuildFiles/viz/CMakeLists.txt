#############################
## PRELIMINARIES
#############################

# Set the required version of CMake
cmake_minimum_required(VERSION 2.8.3)

# Define the project name
project(prx_visualization)

#############################
## SET ENVIRONMENT VARIABLES
#############################

# Set the build type.  Options are Coverage, Debug, Release, RelWitheDebInfo, MinSizeRel
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Locate CMake Modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake_modules/")


#############################
## FIND PACKAGES
#############################

# catkin
find_package( catkin REQUIRED 
              COMPONENTS roscpp rospy roslib geometry_msgs class_loader tf prx_utilities )

# CMake
find_package(cmake_modules REQUIRED)

# OSG
find_package( OpenSceneGraph 2.8 REQUIRED
              COMPONENTS osgDB osgGA osgUtil osgViewer osgText)

# PNG
find_library(PNGLIB PRXPNG
         paths ${PROJECT_SOURCE_DIR}/../prx_external/lib
         doc "PNG library")
find_path(PNGINCLUDE lodepng.hpp
      paths ${PROJECT_SOURCE_DIR}/../prx_external/PNGCreator
      doc "PNG include directory")
if(PNGLIB AND PNGINCLUDE)
    message("-- Found PNG library and header files")
else(PNGLIB AND PNGINCLUDE)
    message(WARNING "Could not find PNG library and header files")
endif(PNGLIB AND PNGINCLUDE)


###########################################
## MESSAGES, SERVICES AND ACTION TARGETS
###########################################

###########################################
## REGISTER THIS AS A CATKIN PACKAGE
###########################################

catkin_package(
  INCLUDE_DIRS "prx"
  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS roscpp rospy roslib geometry_msgs message_runtime class_loader tf prx_utilities 
  DEPENDS Boost
  )

##############################################
## INCLUDE DIRECTORIES and ADD DEFINITIONS
##############################################

# Our code...
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/../prx_utilities)

# Catkin packages
include_directories( ${catkin_INCLUDE_DIRS} )
# include_directories(${prx_decision_making_INCLUDE_DIRS})

# Boost
include_directories(${Boost_INCLUDE_DIRS})

# OSG
include_directories(${OPENSCENEGRAPH_INCLUDE_DIRS})

# PNG
if(PNGLIB AND PNGINCLUDE)
    include_directories(${PNGINCLUDE})
endif(PNGLIB AND PNGINCLUDE)

############################################
## ADD LIBRARY and TARGET LINK LIBRARIES
############################################

# define the simulation source files
file(GLOB_RECURSE SRC_VISUALIZATION ${PROJECT_SOURCE_DIR}/prx/visualization/*.cpp)

add_library(${PROJECT_NAME}_lib ${SRC_VISUALIZATION})

# target link libraries
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OPENSCENEGRAPH_LIBRARIES} ${PNGLIB} tinyxml ${prx_utilities_LIBRARIES} )

SET_TARGET_PROPERTIES(${PROJECT_NAME}_lib PROPERTIES ENABLE_EXPORTS TRUE)

# add dependency to the generation of messages
add_dependencies( ${PROJECT_NAME}_lib ${PROJECT_NAME}_generate_messages_cpp)

################
## Executable 
################
add_executable(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/prx/main.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)

############################
## TESTS AND INSTALL
############################
install( DIRECTORY "prx"
         DESTINATION "include"
         FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
         )

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

