#############################
## PRELIMINARIES
#############################

# Set the required version of CMake
cmake_minimum_required(VERSION 2.8.3)

# Define the project name
project(prx_simulation)

#############################
## SET ENVIRONMENT VARIABLES
#############################

# Set the build type.  Options are Coverage, Debug, Release, RelWitheDebInfo, MinSizeRel
set(CMAKE_BUILD_TYPE debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Locate CMake Modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake_modules/")

#############################
## FIND PACKAGES
#############################

add_definitions(-DUSE_TRAC_IK)

# catkin
find_package( catkin REQUIRED 
              COMPONENTS roscpp rospy roslib message_generation geometry_msgs std_msgs class_loader tf prx_utilities kdl_parser actionlib_msgs actionlib trac_ik_lib)
find_package(cmake_modules REQUIRED)
# Eigen
find_package(Eigen)

# Octave
find_package(OCTAVE)

# Locating Boost ?????????????????? what components do we use from Boost ????????????????
find_package(Boost REQUIRED)

# OCTOMAP
find_package( OCTOMAP )

# FCL
find_library(FCLLIB fcl
         paths /usr/local/lib /usr/lib /opt/ros/indigo/lib/
         doc "FCL library")
find_path(FCLINCLUDE fcl/collision.h
      paths /usr/include /usr/local/include /opt/ros/indigo/include
      doc "fcl include directory")
if(FCLLIB AND FCLINCLUDE)
    message("-- Found FCL library and header files")
else(FCLLIB AND FCLINCLUDE)
    message(WARNING "Could not find FCL library and header files")
endif(FCLLIB AND FCLINCLUDE)

# LAPACK
find_package( LAPACK )

# BULLET
find_package( Bullet )

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


# PPQP
find_library(PPQPLIB PPQP
       paths ${PROJECT_SOURCE_DIR}/../prx_external/lib
       doc "PPQP library")
find_path(PPQPINCLUDE cuda_defs.h
    paths ${PROJECT_SOURCE_DIR}/../prx_external/parallel_pqp
    doc "PPQP include directory")
if(PPQPLIB AND PPQPINCLUDE)
    message("-- Found PPQP library and header files")
  add_definitions(-DCUDA_COLLISION)
else(PPQPLIB AND PPQPINCLUDE)
    message(WARNING "Could not find PPQP library and header files: ${PPQPLIB} ... ${PPQPINCLUDE}")
endif(PPQPLIB AND PPQPINCLUDE)

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

#
# Trying to find the prx_learn package
#
find_package(prx_learn)

#
# Trying to find the prx_sensing package
#
find_package(prx_sensing)


##
## MESSAGES, SERVICES AND ACTION TARGETS
##

file(GLOB_RECURSE MSG_SIM RELATIVE ${PROJECT_SOURCE_DIR}/msg *.msg)
add_message_files(FILES ${MSG_SIM})

file(GLOB_RECURSE SRV_SIM RELATIVE ${PROJECT_SOURCE_DIR}/srv *.srv)
add_service_files(FILES ${SRV_SIM})

add_action_files(
  DIRECTORY action
  FILES send_trajectory.action
)

generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs std_msgs )


##
## REGISTER THIS AS A CATKIN PACKAGE
##
catkin_package(
  INCLUDE_DIRS "prx"
  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS roscpp rospy roslib message_generation geometry_msgs std_msgs message_runtime class_loader tf prx_utilities kdl_parser actionlib_msgs actionlib trac_ik_lib
  DEPENDS OCTAVE Boost OCTOMAP LAPACK Bullet
  )

##
## INCLUDE DIRECTORIES and ADD DEFINITIONS
##

# Our code...
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/../prx_utilities)


# Catkin packages
include_directories( ${catkin_INCLUDE_DIRS} )
include_directories(${prx_decision_making_INCLUDE_DIRS})

# XML
#add_definitions(-DTIXML_USE_STL)

# Eigen
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

# Octave
if(OCTAVE_FOUND)
    include_directories(${OCTAVE_INCLUDE_DIR})
    add_definitions(-DOCTAVE_FOUND)
endif(OCTAVE_FOUND)

# Boost
include_directories(${Boost_INCLUDE_DIRS})

# Octomap
include_directories(${OCTOMAP_INCLUDE_DIRS})

# fcl
if(FCLLIB AND FCLINCLUDE)
    include_directories(${FCLINCLUDE})
endif(FCLLIB AND FCLINCLUDE)

# LAPACK
include_directories(${LAPACK_INCLUDE_DIRS})

# BULLET
if(BULLET_FOUND)
  include_directories(${BULLET_INCLUDE_DIR})
  add_definitions(-DBULLET_FOUND)
endif(BULLET_FOUND)

# ANN
if(ANNLIB AND ANNINCLUDE)
    include_directories(${ANNINCLUDE})
endif(ANNLIB AND ANNINCLUDE)

# PPQP
if(PPQPLIB AND PPQPINCLUDE)
    include_directories(${PPQPINCLUDE})
endif(PPQPLIB AND PPQPINCLUDE)

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


if(prx_learn_FOUND)
    add_definitions(-DPRX_LEARN_FOUND)
    include_directories(${prx_learn_INCLUDE_DIRS})
    include_directories(${PROJECT_SOURCE_DIR}/../prx_learn)
endif(prx_learn_FOUND)


if(prx_sensing_FOUND)
    include_directories(${prx_sensing_INCLUDE_DIRS})
    include_directories(${PROJECT_SOURCE_DIR}/../prx_sensing)
endif(prx_sensing_FOUND)


##
## ADD LIBRARY and TARGET LINK LIBRARIES
##

# define the simulation source files
file(GLOB_RECURSE SRC_SIMULATION ${PROJECT_SOURCE_DIR}/prx/simulation/*.cpp)

### PACKAGES
get_filename_component(PARENT_DIR "${CMAKE_SOURCE_DIR}" PATH)
include_directories(${PROJECT_SOURCE_DIR}/../prx_packages/apc)
file(GLOB_RECURSE SRC_apc ${PROJECT_SOURCE_DIR}/../prx_packages/apc/simulation/*.cpp)
set(SRC_SIMULATION ${SRC_apc} ${SRC_SIMULATION})

get_filename_component(PARENT_DIR "${CMAKE_SOURCE_DIR}" PATH)
include_directories(${PROJECT_SOURCE_DIR}/../prx_packages/manipulation)
file(GLOB_RECURSE SRC_manipulation ${PROJECT_SOURCE_DIR}/../prx_packages/manipulation/simulation/*.cpp)
set(SRC_SIMULATION ${SRC_manipulation} ${SRC_SIMULATION})

### END PACKAGES

add_library(${PROJECT_NAME}_lib ${SRC_SIMULATION})

# target link libraries
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LAPACK_LIBRARIES} ${PQPLIB} ${ANNLIB} ${PNGLIB} ${CLIPPERLIB} ${prx_utilities_LIBRARIES} )

if(FCLLIB AND FCLINCLUDE)
    target_link_libraries(${PROJECT_NAME}_lib ${FCLLIB})
endif(FCLLIB AND FCLINCLUDE)

# PPQP
if(PPQPLIB AND PPQPINCLUDE)
    target_link_libraries(${PROJECT_NAME}_lib ${PPQPLIB})
endif(PPQPLIB AND PPQPINCLUDE)


if(BULLET_FOUND)
    target_link_libraries(${PROJECT_NAME}_lib ${BULLET_LIBRARIES})
endif(BULLET_FOUND)

if(OCTAVE_FOUND)
    target_link_libraries(${PROJECT_NAME}_lib ${OCTAVE_LIBRARY} ${OCTAVE_INT_LIBRARY})
endif(OCTAVE_FOUND)

if(prx_learn_FOUND)
    target_link_libraries(${PROJECT_NAME}_lib ${prx_learn_LIBRARIES} )
    message("Learn Library: ${prx_learn_LIBRARIES}")
endif(prx_learn_FOUND)

if(prx_sensing_FOUND)
    add_definitions(-DPRX_SENSING_FOUND)
    target_link_libraries(${PROJECT_NAME}_lib ${prx_sensing_LIBRARIES} )
    message("Sensing Library: ${prx_sensing_LIBRARIES}")
endif(prx_sensing_FOUND)


if(DEFINED ENV{NTRT_PATH})
include_directories($ENV{NTRT_PATH}/src)
file(GLOB_RECURSE NTRT_LIB $ENV{NTRT_PATH}/build/*.so)
message(STATUS "${NTRT_LIB}")
target_link_libraries(${PROJECT_NAME}_lib ${NTRT_LIB})
endif()


SET_TARGET_PROPERTIES(${PROJECT_NAME}_lib PROPERTIES ENABLE_EXPORTS TRUE)

# add dependency to the generation of messages
add_dependencies( ${PROJECT_NAME}_lib ${PROJECT_NAME}_generate_messages_cpp)

################
## Executable 
################
add_executable(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/prx/main.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)

########################
## Executable for PABT
########################
add_executable(PABT ${PROJECT_SOURCE_DIR}/prx/pabt_main.cpp)
target_link_libraries(PABT ${PROJECT_NAME}_lib)


##
## TESTS AND INSTALL
##
install( DIRECTORY "prx"
         DESTINATION "include"
         FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
         )

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
