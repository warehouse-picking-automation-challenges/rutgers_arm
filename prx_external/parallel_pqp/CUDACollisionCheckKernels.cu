#include <stdio.h>
#include <algorithm>
//#include <cuda.h>
#include <curand_kernel.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <curand.h>

#include "cuda_errorcheck.h"
#include "cuda_defs.h"
#include "collision/cuda_robobs.h"
#include "PlanningParams.h"
#include "cuda_statePrimitives.h"
#include "cuda_stateIntersect.h"
#include "cuda_timing.h"
#include "CUDACollisionCheckKernels.h"

// device memory random generator states
////////////////////////////////////////////////////////////////////////////////

// solution info

// device memory constants
////////////////////////////////////////////////////////////////////////////////
__device__ __constant__ float c_boundsMin[3];
__device__ __constant__ float c_boundsMax[3];
__device__ __constant__ float c_boundsDiff[3];
__device__ __constant__ float c_radius[MAX_LEVEL];

// device functions
////////////////////////////////////////////////////////////////////////////////


// device kernels
////////////////////////////////////////////////////////////////////////////////
//check collision of robot with different states ,and object
  template <class CollideFunctor>
__global__ void checkCollisionKernel(int num_of_robot_states, SerializedStates* robot_states,	int num_of_obstacle_states, SerializedStates* obstacle_states, GMesh* robot, GMesh* obstacle, CollideFunctor collideF,	int* d_results)
{
  int state_id = blockIdx.x * blockDim.x + threadIdx.x;
  if (state_id >= num_of_robot_states) {
//    printf ("state_id %d is out of boundary!\n", state_id);
    return;
  }

  int result = COLLISION_FREE;
  int obstacle_state_id = (num_of_obstacle_states == 1)? 0: state_id;
    
  result = collideF(robot->models, robot->vertexPointers, robot->triIdxPointers, &(robot_states[state_id].x[0]), obstacle->models, obstacle->vertexPointers, obstacle->triIdxPointers, &(obstacle_states[obstacle_state_id].x[0]));

  d_results[state_id] = (result == COLLISION_FREE) ? 0: 1;

#ifdef DEBUG_STEP_VALUES
  if (d_results[state_id]) 
    printf("checkCollisionKernel2:: Found collision!\n");
  else
    printf("checkCollisionkernel2:: No collision!\n");
#endif
}

  template <class CollideFunctor>
__global__ void checkCollisionFastKernel(int num_of_robot_states, SerializedStates* robot_states, int num_of_obstacle_states, SerializedStates* obstacle_states, GMesh* robot, GMesh* obstacle, CollideFunctor collideF, volatile int* d_results)
{
  int state_id = blockIdx.x * blockDim.x + threadIdx.x;
  if (state_id >= num_of_robot_states) {
//    printf ("state_id %d is out of boundary!\n", state_id);
    return;
  }

  int result = COLLISION_FREE;
  int obstacle_state_id = (num_of_obstacle_states == 1)? 0: state_id;
  if (*d_results == 1)
    return;
    
  result = collideF(robot->models, robot->vertexPointers, robot->triIdxPointers, &(robot_states[state_id].x[0]), obstacle->models, obstacle->vertexPointers, obstacle->triIdxPointers, &(obstacle_states[obstacle_state_id].x[0]));

  if (result != COLLISION_FREE && *d_results == 0) {
    d_results[0] = 1;
  }

#ifdef DEBUG_STEP_VALUES
  if (d_results[0]) 
    printf("checkCollisionKernel2:: Found collision!\n");
  else
    printf("checkCollisionkernel2:: No collision!\n");
#endif
}

namespace collision
{

  namespace cuda
  {

    void allocMemforMesh(GMesh* &d_object) {
      // meshes
      GPUMALLOC((void**)&d_object, sizeof(GMesh));
    }

    void cpyMeshToGPU(const GMesh* h_object, GMesh*& d_object) {
      TOGPU(d_object, h_object, sizeof(GMesh));
    }

    void allocMemforStates(const std::vector<SerializedStates>& h_states, SerializedStates* &d_states, int num_of_states) {
      GPUMALLOC(&d_states, sizeof(SerializedStates) * num_of_states);
    }


    void allocMemforStatesAndResults(SerializedStates* &d_robot_states, SerializedStates* &d_obstacle_states, int* &d_results,  int max_num_of_states)
    {
      //state and results gpu memory allocation
      GPUMALLOC((void**)&d_obstacle_states, sizeof(SerializedStates) * max_num_of_states);
      GPUMALLOC((void**)&d_robot_states, sizeof(SerializedStates) * max_num_of_states);
      GPUMALLOC((void**)&d_results, sizeof(int) * max_num_of_states);
    }



    void freeStatesAndResults(SerializedStates* d_robot_states, SerializedStates* d_obstacle_states, int* d_results)
    {
      CUDA_CHECK_ERROR();
      // states
      GPUFREE(d_robot_states);
      GPUFREE(d_obstacle_states);

      // results
      GPUFREE(d_results);
      CUDA_CHECK_ERROR();
    }

    void cpyStatesToGPU(const std::vector<SerializedStates>& h_states, SerializedStates* &d_states, int num_of_states)
    {
      // obstacle state and robot state
      TOGPU(d_states, &(h_states[0]), sizeof(SerializedStates) * num_of_states);
    }

    void setResults(int* d_results, int val, int num_of_states) {
      GPUMEMSET(d_results, val, sizeof(int) * num_of_states);
    }

    void downloadResults(int* h_results, int* d_results, int num_of_robot_states)
    {
      CUDA_CHECK_ERROR();
      CUDA_TIMING_BEGIN();

      FROMGPU(h_results, d_results, sizeof(int) * num_of_robot_states);

      CUDA_TIMING_END("downloadResults");
      CUDA_CHECK_ERROR();

    }

    void initializeMemory(GMesh* &d_robot, GMesh* &d_obstacle, SerializedStates* &d_robot_states, int* &d_results, int num_of_robot_states, SerializedStates* &d_obstacle_states, int num_of_obstacle_states)
    {
      //obstable state
      GPUMALLOC(&d_obstacle_states, sizeof(SerializedStates) * num_of_obstacle_states);
      CUDA_CHECK_ERROR();

      GPUMALLOC(&d_robot_states, sizeof(SerializedStates) * num_of_robot_states);
      CUDA_CHECK_ERROR();
      GPUMALLOC(&d_results, sizeof(int) * num_of_robot_states);
      CUDA_CHECK_ERROR();

      // meshes
      GPUMALLOC(&d_robot, sizeof(GMesh));
      GPUMALLOC(&d_obstacle, sizeof(GMesh));

      CUDA_CHECK_ERROR();
    }

    void uploadData(const GMesh* h_robot, const GMesh* h_obstacle, const std::vector<SerializedStates>& h_robot_states, const std::vector<SerializedStates>& h_obstacle_states, GMesh*& d_robot, GMesh*& d_obstacle, SerializedStates*& d_robot_states, SerializedStates*& d_obstacle_states)
    {
      // meshes
      TOGPU(d_robot, h_robot, sizeof(GMesh));
      TOGPU(d_obstacle, h_obstacle, sizeof(GMesh));
      // obstacle state and robot state
      TOGPU(d_obstacle_states, &(h_obstacle_states[0]), sizeof(SerializedStates) * h_obstacle_states.size()); 
      TOGPU(d_robot_states, &(h_robot_states[0]), sizeof(SerializedStates) * h_robot_states.size());

    }


    void freeMemory(GMesh* d_robot, GMesh* d_obstacle, SerializedStates* d_robot_states, SerializedStates* d_obstacle_states, int* d_results)
    {
      CUDA_CHECK_ERROR();
      // meshes
      GPUFREE(d_robot);
      GPUFREE(d_obstacle);

      // states
      GPUFREE(d_robot_states);
      GPUFREE(d_obstacle_states);

      // results
      GPUFREE(d_results);
      CUDA_CHECK_ERROR();
    }

    void checkCollision(int num_of_robot_states, SerializedStates* d_robot_states, int num_of_obstacle_states, SerializedStates* d_obstacle_states, int* d_results, GMesh* d_robot, GMesh* d_obstacle)
    {
      int gridX = (num_of_robot_states + COLLISION_THREADS) / COLLISION_THREADS;
      dim3 grids = dim3(gridX, 1, 1);
      int threadX = (COLLISION_THREADS > num_of_robot_states)? num_of_robot_states: COLLISION_THREADS;
      dim3 threads = dim3(threadX, 1, 1); // (64, 1, 1)

      CUDA_TIMING_BEGIN();

      // compute N (# of checking points)
      BVHTwoStatesCollideFunctor<OBBNode, OBB, ushort2, IN_COLLISION, unsigned short> collideF;
      checkCollisionKernel<BVHTwoStatesCollideFunctor<OBBNode, OBB, ushort2, IN_COLLISION, unsigned short> > <<< grids, threads >>>(
          num_of_robot_states, d_robot_states, num_of_obstacle_states, d_obstacle_states, 
          d_robot, d_obstacle, collideF,
          d_results);

      CUDA_TIMING_END("checkCollisionKernel");

      CUDA_CHECK_ERROR();
    }

    void checkCollisionFast(int num_of_robot_states, SerializedStates* d_robot_states, int num_of_obstacle_states, SerializedStates* d_obstacle_states, int* d_results, GMesh* d_robot, GMesh* d_obstacle)
    {
      int gridX = (num_of_robot_states + COLLISION_THREADS) / COLLISION_THREADS;
      dim3 grids = dim3(gridX, 1, 1);
      int threadX = (COLLISION_THREADS > num_of_robot_states)? num_of_robot_states: COLLISION_THREADS;
      dim3 threads = dim3(threadX, 1, 1); // (64, 1, 1)

      CUDA_TIMING_BEGIN();

      // compute N (# of checking points)
      BVHTwoStatesFastCollideFunctor<OBBNode, OBB, ushort2, IN_COLLISION, unsigned short> collideF;
      checkCollisionKernel<BVHTwoStatesFastCollideFunctor<OBBNode, OBB, ushort2, IN_COLLISION, unsigned short> > <<< grids, threads >>>(
          num_of_robot_states, d_robot_states, num_of_obstacle_states, d_obstacle_states, 
          d_robot, d_obstacle, collideF,
          d_results);

      CUDA_TIMING_END("checkCollisionKernel");

      CUDA_CHECK_ERROR();
    }



    void reset()
    {
    }

  }

}





