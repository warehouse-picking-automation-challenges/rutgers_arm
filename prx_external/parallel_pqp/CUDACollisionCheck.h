#ifndef CUDA_COLLISION_CHECK_H
#define CUDA_COLLISION_CHECK_H

//#include "ompl/geometric/planners/PlannerIncludes.h"
#include "PQP/PQP.h"
#include "cuda_rrt_types.h"
#include <iostream>
#include <vector>
#include <unordered_map>

using namespace std;

class GMesh;



namespace collision
{

  class CUDACollisionCheck
  {
    public:
      CUDACollisionCheck();
      CUDACollisionCheck(  PQP_Model* robot_model,   PQP_Model* obstacle_model);
      CUDACollisionCheck(  std::vector<std::vector<float> > &robot_triangles,   std::vector<std::vector<float> > &obstacle_triangles);
      CUDACollisionCheck(  PQP_Model* robot_model,   PQP_Model* obstacle_model,   std::vector<std::vector<float> > &robot_triangles,   std::vector<std::vector<float> > &obstacle_triangles);

      CUDACollisionCheck(  PQP_Model* robot_model,   PQP_Model* obstacle_model, std::vector<float> &robot_states, std::vector<float> &obstacle_states);
      CUDACollisionCheck(  std::vector<std::vector<float> > &robot_triangles,   std::vector<std::vector<float> > &obstacle_triangles, std::vector<float> &robot_states, std::vector<float> &obstacle_states);
      CUDACollisionCheck(  PQP_Model* robot_model,   PQP_Model* obstacle_model,   std::vector<std::vector<float> > &robot_triangles,   std::vector<std::vector<float> > &obstacle_triangles, std::vector<float> &robot_states, std::vector<float> &obstacle_states);

      ~CUDACollisionCheck();

      std::vector<bool> checkCollision();
      bool checkCollisionFast();
      PQP_Model* createPQPModel(  vector<vector<float> >& triangles);
      void setupRobotAndObstable(  std::vector<std::vector<float> > &obstacleTriangles,   std::vector<std::vector<float> > &robotTriangles, GMesh** obstacles, GMesh** robot);
      bool setupRobotAndObstable(  PQP_Model* p_robot_model,   PQP_Model* p_obstacle_model, GMesh** obstacles, GMesh** robot);
      void setupRobotAndObstable(  PQP_Model* p_robot_model,   PQP_Model* p_obstacle_model,   std::vector<std::vector<float> > &obstacle_triangles,   std::vector<std::vector<float> > &robot_triangles, GMesh** obstacles, GMesh** robot);
      void clear();



      void initStates(std::vector<float> &robot_states, std::vector<float> &obstacle_states);
      void loadTrianglesFromPQPModel (const  PQP_Model* model, std::vector<std::vector<float> > &triangles);


      bool initMeshes(PQP_Model * p_robot_model, PQP_Model* p_obstacle_model);
      bool initMesh(PQP_Model* p_model, GMesh* &d_mesh);
      GMesh* findMeshCacheOnGPU(const PQP_Model* model) ;
      bool setupMeshOnCPU(PQP_Model* p_model, GMesh* &pMesh);
      bool setupMeshOnGPU(const GMesh* h_mesh, GMesh* &d_mesh);
      

      void setupMemoryforStatesAndResults();
      void setupStatesOnGPU(std::vector<SerializedStates>& h_robot_states,   std::vector<SerializedStates>& h_obstacle_states);

    protected:
      void freeStatesAndResults();

      void reset();
      void uploadData(  GMesh* h_robot,   GMesh* h_obstacle,   std::vector<SerializedStates>& h_robot_states,   std::vector<SerializedStates>& h_obstacle_states);

      GMesh* h_obstacle_; // pointer 
      GMesh* h_robot_;
      GMesh* d_robot_;
      GMesh* d_obstacle_;
      SerializedStates*  d_robot_states_;
      SerializedStates*  d_obstacle_states_;
      int num_of_robot_states_;
      int num_of_obstacle_states_;
      int* d_results_;
      int* h_results_;
      int max_result_size;
      std::unordered_map<const PQP_Model*, GMesh*> GPU_mesh_hashtable;

      int states_allocated;
  };

}

#endif
