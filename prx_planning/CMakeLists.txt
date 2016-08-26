#############################
## PRELIMINARIES
#############################

# Set the required version of CMake
cmake_minimum_required(VERSION 2.8.3)

# Define the project name
project(prx_planning)

#############################
## SET ENVIRONMENT VARIABLES
#############################

# Set the build type.  Options are Coverage, Debug, Release, RelWitheDebInfo, MinSizeRel
set(CMAKE_BUILD_TYPE debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Locate CMake Modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

#############################
## FIND PACKAGES
#############################

# catkin
find_package( catkin REQUIRED 
              COMPONENTS roscpp rospy roslib message_generation geometry_msgs std_msgs sensor_msgs class_loader tf actionlib actionlib_msgs prx_utilities prx_simulation)

#catkin_python_setup()
find_package(cmake_modules REQUIRED)

# Octave
find_package(OCTAVE)

# Locating Boost 
find_package(Boost REQUIRED)

# OCTOMAP
find_package( octomap )

#Yaml-Cpp
find_package( YamlCpp )

# ANN
find_library(ANNLIB ANN
         paths ${PROJECT_SOURCE_DIR}/../prx_external/lib
         doc "ANN library")
find_path(ANNINCLUDE ANN.h
      paths ${PROJECT_SOURCE_DIR}/../prx_external/ANN
      doc "ANN include directory")
if(ANNLIB AND ANNINCLUDE)
    message("-- Found ANN library and header files")
else(ANNLIB AND ANNINCLUDE)
    message(WARNING "Could not find ANN library and header files")
endif(ANNLIB AND ANNINCLUDE)

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

# CLIPPER
find_library(CLIPPERLIB PRXCLIPPER
         paths ${PROJECT_SOURCE_DIR}/../prx_external/lib
         doc "Clipper library")
find_path(CLIPPERINCLUDE clipper.hpp
      paths ${PROJECT_SOURCE_DIR}/../prx_external/clipper
      doc "Clipper include directory")
if(CLIPPERLIB AND CLIPPERINCLUDE)
    message("-- Found CLIPPER library and header files")
else(CLIPPERLIB AND CLIPPERINCLUDE)
    message(WARNING "Could not find CLIPPER library and header files")
endif(CLIPPERLIB AND CLIPPERINCLUDE)


#
# Locating PQP
#
find_library(PQPLIB PQP
         paths /usr/local/lib /opt/local/lib ../prx_external/lib
         doc "PQP library")
find_path(PQPINCLUDE PQP.h
      paths /usr/local/include/pqp /opt/local/include/PQP ../prx_external/parallel_pqp/PQP
      doc "PQP include directory")
if(PQPLIB AND PQPINCLUDE)
    message("-- Found PQP library and header files")
else(PQPLIB AND PQPINCLUDE)
    message(WARNING "Could not find PQP library and header files")
endif(PQPLIB AND PQPINCLUDE)

##
## MESSAGES, SERVICES AND ACTION TARGETS
##
file(GLOB_RECURSE SRV_PLAN RELATIVE ${PROJECT_SOURCE_DIR}/srv *.srv)

#add_service_files(FILES ${SRV_PLAN})

add_action_files(
  DIRECTORY action
  FILES apc_query.action parallel_prop.action
)

generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs std_msgs sensor_msgs prx_simulation)

##
## REGISTER THIS AS A CATKIN PACKAGE
##
catkin_package(
  INCLUDE_DIRS "prx"
  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS roscpp rospy roslib message_generation geometry_msgs std_msgs message_runtime sensor_msgs class_loader tf actionlib actionlib_msgs prx_utilities prx_simulation
  DEPENDS Boost OCTOMAP
 )

##
## INCLUDE DIRECTORIES and ADD DEFINITIONS
##

# Our code...
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/../prx_utilities)
include_directories(${PROJECT_SOURCE_DIR}/../prx_simulation)


# Catkin packages
include_directories( ${catkin_INCLUDE_DIRS} )

# Octave
if(OCTAVE_FOUND)
    include_directories(${OCTAVE_INCLUDE_DIR})
    add_definitions(-DOCTAVE_FOUND)
endif(OCTAVE_FOUND)

# Boost
include_directories(${Boost_INCLUDE_DIRS})

# Octomap
include_directories(${OCTOMAP_INCLUDE_DIRS})

# YamlCpp
include_directories(${YAMLCPP_INCLUDE_DIR})

# ANN
if(ANNLIB AND ANNINCLUDE)
    include_directories(${ANNINCLUDE})
endif(ANNLIB AND ANNINCLUDE)

# PNG
if(PNGLIB AND PNGINCLUDE)
    include_directories(${PNGINCLUDE})
endif(PNGLIB AND PNGINCLUDE)


# CLIPPER
if(CLIPPERLIB AND CLIPPERINCLUDE)
    include_directories(${CLIPPERINCLUDE})
endif(CLIPPERLIB AND CLIPPERINCLUDE)

# PQP
if(PQPLIB AND PQPINCLUDE)
    include_directories(${PQPINCLUDE})
endif(PQPLIB AND PQPINCLUDE)

##
## ADD LIBRARY and TARGET LINK LIBRARIES
##

# define the planning source files
file(GLOB_RECURSE SRC_PLANNING ${PROJECT_SOURCE_DIR}/prx/planning/*.cpp)


### PACKAGES
get_filename_component(PARENT_DIR "${CMAKE_SOURCE_DIR}" PATH)
include_directories(${PROJECT_SOURCE_DIR}/../prx_packages/apc)
file(GLOB_RECURSE SRC_apc ${PROJECT_SOURCE_DIR}/../prx_packages/apc/planning/*.cpp)
set(SRC_PLANNING ${SRC_apc} ${SRC_PLANNING})

get_filename_component(PARENT_DIR "${CMAKE_SOURCE_DIR}" PATH)
include_directories(${PROJECT_SOURCE_DIR}/../prx_packages/manipulation)
file(GLOB_RECURSE SRC_manipulation ${PROJECT_SOURCE_DIR}/../prx_packages/manipulation/planning/*.cpp)
set(SRC_PLANNING ${SRC_manipulation} ${SRC_PLANNING})

### END PACKAGES

add_library(${PROJECT_NAME}_lib ${SRC_PLANNING})

# target link libraries
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${YAMLCPP_LIBRARY} ${ANNLIB} ${PNGLIB} ${PQPLIB} ${CLIPPERLIB} ${prx_utilities_LIBRARIES}  ${prx_simulation_LIBRARIES})


if(OCTOMAP_FOUND)
    target_link_libraries(${PROJECT_NAME}_lib ${OCTOMAP_LIBRARIES})
endif(OCTOMAP_FOUND)


if(OCTAVE_FOUND)
    target_link_libraries(${PROJECT_NAME}_lib ${OCTAVE_LIBRARY} ${OCTAVE_INT_LIBRARY})
endif(OCTAVE_FOUND)


if(DEFINED ENV{NTRT_PATH})
include_directories($ENV{NTRT_PATH}/src)
file(GLOB_RECURSE NTRT_LIB $ENV{NTRT_PATH}/build/*.so)
target_link_libraries(${PROJECT_NAME}_lib ${NTRT_LIB})
endif()


#target_link_libraries(${PROJECT_NAME}_lib  console_bridge)

SET_TARGET_PROPERTIES(${PROJECT_NAME}_lib PROPERTIES ENABLE_EXPORTS TRUE)

# add dependency to the generation of messages
add_dependencies( ${PROJECT_NAME}_lib ${PROJECT_NAME}_generate_messages_cpp )

################
## Executables
################
add_executable(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/prx/main.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)

add_executable(planning_experiment ${PROJECT_SOURCE_DIR}/prx/experiment.cpp)
target_link_libraries(planning_experiment ${PROJECT_NAME}_lib)

##
## TESTS AND INSTALL
##
install( DIRECTORY "prx"
         DESTINATION "include"
         FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
         )

install(TARGETS ${PROJECT_NAME}_lib ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
