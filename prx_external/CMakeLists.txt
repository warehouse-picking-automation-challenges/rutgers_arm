cmake_minimum_required(VERSION 2.8)

#project(KEBROS_DEPENDENCIES CXX C)

set(LIBRARY_OUTPUT_PATH    ${CMAKE_SOURCE_DIR}/lib)


set(BUILD_SHARED_LIBS TRUE)


#
# Cuda-based PQP collision checker
#
OPTION(CUDA_COLLISION "Cuda Collision Checker Library" OFF)
if(CUDA_COLLISION)

	set( PPQP_DIR "${CMAKE_SOURCE_DIR}/parallel_pqp" )

	include_directories( ${PPQP_DIR} )

	find_package(CUDA 6.5 REQUIRED)
	set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};--ptxas-options=-v -Xlinker --verbose -O3  -std=c++11)
	set(CUDA_PROPAGATE_HOST_FLAGS ON)


	file(GLOB_RECURSE SRC_CUDA_COLL ${CMAKE_SOURCE_DIR}/parallel_pqp/*.cpp ${CMAKE_SOURCE_DIR}/parallel_pqp/*.cu)

	cuda_add_library(PPQP ${SRC_CUDA_COLL} OPTIONS ${CUDA_OPTIONS})
	target_link_libraries(PPQP ${CUDA_LIBRARIES})


endif()


add_definitions(-Wall -fPIC -O3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

if(APPLE)
    add_definitions(-DAPPLE)

    include(CMakeFindFrameworks)
    CMAKE_FIND_FRAMEWORKS(vecLib)
    if(vecLib_FRAMEWORKS)
      set(
        prx_vecLib_FRAMEWORK ${vecLib_FRAMEWORKS}
        CACHE PATH "Path to vecLib framework"
      )
      message("Found veclib")
      mark_as_advanced(prx_vecLib_FRAMEWORK)
    endif(vecLib_FRAMEWORKS)

endif(APPLE)


#
# ANN
#

set( ANN_DIR "${CMAKE_SOURCE_DIR}/ANN" )

include_directories( ${ANN_DIR} )

add_library( ANN
		${ANN_DIR}/ANN.cc
		${ANN_DIR}/bd_pr_search.cc
		${ANN_DIR}/bd_search.cc
		${ANN_DIR}/bd_tree.cc
		${ANN_DIR}/brute.cc
		${ANN_DIR}/kd_pr_search.cc
		${ANN_DIR}/kd_search.cc
		${ANN_DIR}/kd_split.cc
		${ANN_DIR}/kd_tree.cc
		${ANN_DIR}/kd_util.cc
		${ANN_DIR}/perf.cc  )

#
# PNG
#

set( PNG_DIR "${CMAKE_SOURCE_DIR}/PNGCreator" )
include_directories( ${PNG_DIR} )
file(GLOB_RECURSE SRC_PNG ${PNG_DIR}/*.cpp)
add_library( PRXPNG ${SRC_PNG})

#
# clipper
#

set( CLIPPER_DIR "${CMAKE_SOURCE_DIR}/clipper" )
include_directories( ${CLIPPER_DIR} )
file(GLOB_RECURSE SRC_CLIPPER ${CLIPPER_DIR}/*.cpp)
add_library( PRXCLIPPER ${SRC_CLIPPER})

if(APPLE)
#
# PQP
#
#	message(STATUS "NOT CUDA")
#	set( PQP_DIR "${CMAKE_SOURCE_DIR}/PQP" )
#
#	include_directories( ${PQP_DIR} )
#
#	add_library( PQP
#			${PQP_DIR}/Build.cpp
#			${PQP_DIR}/BV.cpp
#			${PQP_DIR}/PQP.cpp
#			${PQP_DIR}/TriDist.cpp)


	set( PQP_DIR "${CMAKE_SOURCE_DIR}/parallel_pqp/PQP" )

	include_directories( ${PQP_DIR} )

	add_library( PQP
			${PQP_DIR}/Build.cpp
			${PQP_DIR}/BV.cpp
			${PQP_DIR}/PQP.cpp
			${PQP_DIR}/TriDist.cpp)

	file(COPY ${PQP_DIR}/MatVec.h DESTINATION /usr/local/include )
	file(COPY ${PQP_DIR}/BVTQ.h DESTINATION /usr/local/include )
	file(COPY ${PQP_DIR}/PQP.h DESTINATION /usr/local/include )
	file(COPY ${PQP_DIR}/PQP_Compile.h DESTINATION /usr/local/include )
	file(COPY ${PQP_DIR}/RectDist.h DESTINATION /usr/local/include )
	file(COPY ${PQP_DIR}/TriDist.h DESTINATION /usr/local/include )
	file(COPY ${PQP_DIR}/Tri.h DESTINATION /usr/local/include )
	file(COPY ${PQP_DIR}/Build.h DESTINATION /usr/local/include )
	file(COPY ${PQP_DIR}/BV.h DESTINATION /usr/local/include )
	file(COPY ${PQP_DIR}/GetTime.h DESTINATION /usr/local/include )
	file(COPY ${PQP_DIR}/OBB_Disjoint.h DESTINATION /usr/local/include )
	file(COPY ${PQP_DIR}/PQP_Internal.h DESTINATION /usr/local/include )
else(APPLE)


	set( PQP_DIR "${CMAKE_SOURCE_DIR}/parallel_pqp/PQP" )

	include_directories( ${PQP_DIR} )

	add_library( PQP
			${PQP_DIR}/Build.cpp
			${PQP_DIR}/BV.cpp
			${PQP_DIR}/PQP.cpp
			${PQP_DIR}/TriDist.cpp)

#	set( PQP_DIR "${CMAKE_SOURCE_DIR}/PQP" )
#
#	include_directories( ${PQP_DIR} )
#
#	add_library( PQP
#			${PQP_DIR}/Build.cpp
#			${PQP_DIR}/BV.cpp
#			${PQP_DIR}/PQP.cpp
#			${PQP_DIR}/TriDist.cpp)
endif(APPLE)
