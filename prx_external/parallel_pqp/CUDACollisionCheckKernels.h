#ifndef CUDA_COLLISION_CHECK_KERNELS_H
#define CUDA_COLLISION_CHECK_KERNELS_H

#include <vector>
#include "cuda_rrt_types.h"

class GMesh;

namespace collision
{
  namespace cuda
  {
    // initialize
    void allocMemforMesh(GMesh* &d_object);
    void cpyMeshToGPU(const GMesh* h_object, GMesh*& d_object);

    void allocMemforStates(const std::vector<SerializedStates>& h_states, SerializedStates* &d_states, int num_of_states);
    void allocMemforStatesAndResults(SerializedStates* &d_robot_states, SerializedStates* &d_obstacle_states, int* &d_results,  int max_num_of_states);

    void initializeMemory(GMesh* &d_robot, GMesh* &d_obstacle, SerializedStates* &d_robot_states, int* &d_results, int num_of_robot_states, SerializedStates* &d_obstacle_states, int num_of_obstacle_states);
    void cpyStatesToGPU(const std::vector<SerializedStates>& h_states, SerializedStates* &d_states, int num_of_states);
    // destroy
    void freeStatesAndResults(SerializedStates* d_robot_states, SerializedStates* d_obstacle_states, int* d_results);
    void freeMemory(GMesh* d_robot, GMesh* d_obstacle, SerializedStates* d_robot_states, SerializedStates* d_obstacle_states, int* d_results);
    // upload data from CPU TO GPU
    void uploadData(const GMesh* h_robot, const GMesh* h_obstacle, const std::vector<SerializedStates>& h_robot_states, const std::vector<SerializedStates>& h_obstacle_states, GMesh*& d_robot, GMesh*& d_obstacle, SerializedStates*& d_robot_states, SerializedStates*& d_obstacle_states);
    //void uploadData(const GMesh* h_robot, const GMesh* h_obstacle, const std::vector<SerializedStates>& h_robot_states, const std::vector<SerializedStates>& h_obstacle_states, GMesh* d_robot, GMesh* d_obstacle, SerializedStates* d_robot_states, SerializedStates* d_obstacle_states);
    
    void setResults(int* d_results, int val, int num_of_states);

    // download results from GPU
    void downloadResults(int* h_results, int* d_results, int num_of_robot_states);

    void reset();
    void checkCollision(int num_of_robot_states, SerializedStates* d_robot_states, int num_of_obstacle_states, SerializedStates* d_obstacle_states, int* d_results, GMesh* d_robot, GMesh* d_obstacle);
    void checkCollisionFast(int num_of_robot_states, SerializedStates* d_robot_states, int num_of_obstacle_states, SerializedStates* d_obstacle_states, int* d_results, GMesh* d_robot, GMesh* d_obstacle);

  }
}

#endif
