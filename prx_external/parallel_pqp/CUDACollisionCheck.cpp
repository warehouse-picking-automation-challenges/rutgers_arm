#include "CUDACollisionCheck.h"
#include "CUDACollisionCheckKernels.h"
//#include "gPlannerCollisionWrapper.h"
#include "cuda_defs.h"
#include "collision/cuda_robobs.h"
#include "PlanningParams.h"
#include <cuda_runtime_api.h>
#include "cuda_errorcheck.h"
#include <algorithm>
#include <assert.h>

//#define DEBUG_COLLISION_CHECK


namespace collision
{

  CUDACollisionCheck::CUDACollisionCheck():states_allocated(0){};
  CUDACollisionCheck::CUDACollisionCheck(  PQP_Model* robot_model,   PQP_Model* obstacle_model){
    // allocate memory on GPU for obstacle and robot, and the data structure GMesh stores the pointer to corresponding memory on GPU.
    setupRobotAndObstable( obstacle_model, robot_model, &h_obstacle_, &h_robot_); 
  };

  CUDACollisionCheck::CUDACollisionCheck(  std::vector<std::vector<float> > &robot_triangles,   std::vector<std::vector<float> > &obstacle_triangles)
  {
    // allocate memory on GPU for obstacle and robot, and the data structure GMesh stores the pointer to corresponding memory on GPU.
    setupRobotAndObstable( obstacle_triangles, robot_triangles, &h_obstacle_, &h_robot_); 
  };

  CUDACollisionCheck::CUDACollisionCheck(  PQP_Model* robot_model,   PQP_Model* obstacle_model,   std::vector<std::vector<float> > &robot_triangles,   std::vector<std::vector<float> > &obstacle_triangles) 
  {
    setupRobotAndObstable( obstacle_model, robot_model, obstacle_triangles, robot_triangles, &h_obstacle_, &h_robot_);
  }

  CUDACollisionCheck::CUDACollisionCheck(  PQP_Model* robot_model,   PQP_Model* obstacle_model, std::vector<float> &robot_states, std::vector<float> &obstacle_states){
    setupRobotAndObstable( obstacle_model, robot_model, &h_obstacle_, &h_robot_); 
    initStates(robot_states, obstacle_states);
  };

  CUDACollisionCheck::CUDACollisionCheck(  std::vector<std::vector<float> > &robot_triangles,   std::vector<std::vector<float> > &obstacle_triangles, std::vector<float> &robot_states, std::vector<float> &obstacle_states) 
  {
    setupRobotAndObstable( obstacle_triangles, robot_triangles, &h_obstacle_, &h_robot_); 
    initStates(robot_states, obstacle_states);
  }

  CUDACollisionCheck::CUDACollisionCheck(  PQP_Model* robot_model,   PQP_Model* obstacle_model,   std::vector<std::vector<float> > &robot_triangles,   std::vector<std::vector<float> > &obstacle_triangles, std::vector<float> &robot_states, std::vector<float> &obstacle_states) {
    setupRobotAndObstable( obstacle_model, robot_model, obstacle_triangles, robot_triangles, &h_obstacle_, &h_robot_);
    initStates(robot_states, obstacle_states);
  }
  
  CUDACollisionCheck::~CUDACollisionCheck()
  {
    freeStatesAndResults();
    for (auto& p:GPU_mesh_hashtable) {
      delete p.second;
    }
    GPU_mesh_hashtable.clear();
  }

  bool CUDACollisionCheck::initMeshes(PQP_Model * p_robot_model, PQP_Model* p_obstacle_model) {
    #ifdef DEBUG_COLLISION_CHECK
    std::cout << __FILE__ << " at line: " << __LINE__ << " " << __func__ << std::endl;
    #endif
    if (!p_robot_model || !p_obstacle_model) return false;
    assert(p_robot_model != NULL && p_obstacle_model != NULL);

    return initMesh(p_robot_model, d_robot_) && initMesh(p_obstacle_model, d_obstacle_);
  }

  bool CUDACollisionCheck::initMesh(PQP_Model* p_model, GMesh* &d_mesh) {
    #ifdef DEBUG_COLLISION_CHECK
    std::cout << __FILE__ << " at line: " << __LINE__ << " " << __func__ << std::endl;
    #endif
    if (p_model == nullptr) return false;
    d_mesh = findMeshCacheOnGPU(p_model);
    if (d_mesh == nullptr) {
//      std::cout << __FILE__ << " at line: " << __LINE__ << " " << __func__ << " create model !" << std::endl;
      GMesh* h_mesh;
      setupMeshOnCPU(p_model, h_mesh);
      setupMeshOnGPU(h_mesh, d_mesh);
      delete h_mesh;
      GPU_mesh_hashtable.insert({p_model, d_mesh});
    }
    return true;
  }

  bool CUDACollisionCheck::setupMeshOnCPU(PQP_Model* p_model, GMesh* &h_mesh) {
    #ifdef DEBUG_COLLISION_CHECK
    std::cout << __FILE__ << " at line: " << __LINE__ << " " << __func__ << std::endl;
    #endif
    std::vector<std::vector<float> > model_triangles;
    loadTrianglesFromPQPModel(p_model, model_triangles);
    h_mesh = new GMesh(p_model, model_triangles);
    assert(h_mesh != nullptr);
    return (h_mesh != nullptr);
  }

  bool CUDACollisionCheck::setupMeshOnGPU(const GMesh* h_mesh, GMesh* &d_mesh) {
    #ifdef DEBUG_COLLISION_CHECK
    std::cout << __FILE__ << " at line: " << __LINE__ << " " << __func__ << std::endl;
    #endif
    collision::cuda::allocMemforMesh(d_mesh);
    collision::cuda::cpyMeshToGPU(h_mesh, d_mesh);
    assert(d_mesh != nullptr);
    return (d_mesh != nullptr);
  }

  GMesh* CUDACollisionCheck::findMeshCacheOnGPU(const PQP_Model* model) {
    #ifdef DEBUG_COLLISION_CHECK
    std::cout << __FILE__ << " at line: " << __LINE__ << " " << __func__ << std::endl;
    #endif
    // return nullptr;
    auto  d_model_iterator = GPU_mesh_hashtable.find(model);
    if (d_model_iterator == GPU_mesh_hashtable.end()) {
      return nullptr;
    } else {
      return d_model_iterator->second;
    }
    //   std::vector<std::vector<float> > triangles(model->num_tris);
    //   loadTrianglesFromPQPModel(model, triangles);
    //   GMesh* h_model = new GMesh(model, triangles);
    //   GMesh* d_model = nullptr;
    //   //collision::cuda::initMeshes(d_model);
    //   //collision::cuda::uploadMesh(h_model, d_model);
    //   delete h_model;
    //   if (d_model != nullptr)
    //     GPU_mesh_hashtable.insert({model, d_model});
    //   return d_model;
    // } else {
    //   return d_model_iterator->second;
    // }
  }

  void CUDACollisionCheck::loadTrianglesFromPQPModel (const  PQP_Model* model, std::vector<std::vector<float> > &triangles) {
    // std::cout<<"Tris: "<<model->num_tris<<std::endl;
    triangles.resize(model->num_tris);
    for (int i = 0; i < model->num_tris; ++i) {
      triangles[i].resize(9);
      for (int j = 0; j < 9; ++j) {
        if (j < 3) {
          triangles[i][j] = model->tris[i].p1[j];
        } else if (j < 6) {
          triangles[i][j] = model->tris[i].p2[j%3];
        } else {
          triangles[i][j] = model->tris[i].p3[j%3];
        }
      }
    }
  }

  bool CUDACollisionCheck::setupRobotAndObstable(  PQP_Model* p_robot_model,   PQP_Model* p_obstacle_model, GMesh** obstacles, GMesh** robot)
  {
    std::vector<std::vector<float> > obstacle_triangles;
    loadTrianglesFromPQPModel(p_obstacle_model, obstacle_triangles);
    (*obstacles) = new GMesh(p_obstacle_model, obstacle_triangles);

    std::vector<std::vector<float> > robot_triangles;
    loadTrianglesFromPQPModel(p_robot_model, robot_triangles);
    (*robot) = new GMesh(p_robot_model, robot_triangles);

    return true;
  }

  void CUDACollisionCheck::initStates(std::vector<float> &robot_states, std::vector<float> &obstacle_states)
  {
    
    num_of_robot_states_ = robot_states.size() / 7;

    if (num_of_robot_states_ == 0 || robot_states.size() % 7) {
      printf("input error: robot states should be multiple of 7! \n");
      assert(num_of_robot_states_ == 0 || robot_states.size() % 7);
      return;
    }
    std::vector<SerializedStates> h_robot_states(num_of_robot_states_);
    for (int i = 0; i < num_of_robot_states_; i++) {
      for (int j = 0; j < 7; j++) {
        h_robot_states[i].x[j] = robot_states[i * 7 + j];
      }
      h_robot_states[i].x[7] = 0;
    }

    num_of_obstacle_states_ = obstacle_states.size() / 7; 
    if (num_of_obstacle_states_ == 0 || obstacle_states.size() % 7) {
      printf("input error: obstacle states should be multiple of 7! \n");
      assert(num_of_obstacle_states_ == 0 || obstacle_states.size() % 7);
      return;
    }

    if (num_of_obstacle_states_ > 1 && num_of_robot_states_ != num_of_obstacle_states_) {
      printf("input error: robot and obstacle should have same amount of states!!\n");
      assert(num_of_robot_states_ != num_of_obstacle_states_);
      return;
    }
    std::vector<SerializedStates> h_obstacle_states(num_of_obstacle_states_);

    for (int i = 0; i < num_of_obstacle_states_; ++i) {
      for (int j = 0; j < 7; j++) {
        h_obstacle_states[i].x[j] = obstacle_states[j];
      }
      h_obstacle_states[i].x[7] = 0;
    }   

    // memory allocation
    setupMemoryforStatesAndResults();

    // upload
    setupStatesOnGPU(h_robot_states, h_obstacle_states);
  }

  void CUDACollisionCheck::setupMemoryforStatesAndResults()
  {
    if (num_of_robot_states_ > states_allocated) {
      if (states_allocated != 0) {
        freeStatesAndResults();
        while (states_allocated < num_of_robot_states_) {
          states_allocated *= 2;
        }
      } else {
        states_allocated = ((num_of_robot_states_ + 8)/8) * 8;// just for alignment
      }
      h_results_ = (int*)malloc(sizeof(int) * states_allocated);
      collision::cuda::allocMemforStatesAndResults(d_robot_states_, d_obstacle_states_, d_results_, states_allocated);
    }
  }

  void CUDACollisionCheck::setupStatesOnGPU(std::vector<SerializedStates>& h_robot_states,   std::vector<SerializedStates>& h_obstacle_states) {
    collision::cuda::cpyStatesToGPU(h_obstacle_states, d_obstacle_states_, num_of_obstacle_states_);
    collision::cuda::cpyStatesToGPU(h_robot_states, d_robot_states_, num_of_robot_states_);
  }

  void CUDACollisionCheck::freeStatesAndResults()
  {
    free(h_results_);
    collision::cuda::freeStatesAndResults(d_robot_states_, d_obstacle_states_, d_results_);
  }

	void CUDACollisionCheck::setupRobotAndObstable(  std::vector<std::vector<float> > &obstacleTriangles,   std::vector<std::vector<float> > &robotTriangles, GMesh** obstacles, GMesh** robot)
	{
		PQP_Model* obstacleModel = createPQPModel(obstacleTriangles);
		(*obstacles) = new GMesh(obstacleModel, obstacleTriangles);
		delete obstacleModel;

		PQP_Model* robotModel = createPQPModel(robotTriangles);
		(*robot) = new GMesh(robotModel, robotTriangles);
		delete robotModel;
	}
	
  PQP_Model* CUDACollisionCheck::createPQPModel(  vector<vector<float> >& triangles)
	{
		PQP_Model* pTree = new PQP_Model;
		pTree->BeginModel(triangles.size());

		for(unsigned int i = 0; i < triangles.size(); ++i)
		{
			pTree->AddTri(&(triangles[i][0]), &(triangles[i][3]), &(triangles[i][6]), i);
		}
		pTree->EndModel(false);

		return pTree;
	}






	void CUDACollisionCheck::setupRobotAndObstable(  PQP_Model* p_robot_model,   PQP_Model* p_obstacle_model,   std::vector<std::vector<float> > &obstacle_triangles,   std::vector<std::vector<float> > &robot_triangles, GMesh** obstacles, GMesh** robot)
	{
		(*obstacles) = new GMesh(p_obstacle_model, obstacle_triangles);
		(*robot) = new GMesh(p_robot_model, robot_triangles);
	}

  std::vector<bool> CUDACollisionCheck::checkCollision()
  {
   // ompl::time::point timeStart = time::now();

    // collision check
    collision::cuda::checkCollision(num_of_robot_states_, d_robot_states_, num_of_obstacle_states_, d_obstacle_states_, d_results_, d_robot_, d_obstacle_);


    //double planningTime = time::seconds(time::now() - timeStart);
  //  printf("Collision checking time is  %f sec\n", planningTime);
    collision::cuda::downloadResults(h_results_, d_results_, num_of_robot_states_);

    std::vector<bool> results(num_of_robot_states_);
    for (int i = 0; i < num_of_robot_states_; ++i) {
      if(*(h_results_ + i)) {
        results[i] = true;
#ifdef DEBUG_COLLISION_CHECK
        printf("state %d found collision!\n", i);
#endif
      } else {
        results[i] = false;
#ifdef DEBUG_COLLISION_CHECK
        printf("state %d has no collision!\n", i);
#endif
      }
    }
    return results;
  }

  bool CUDACollisionCheck::checkCollisionFast()
  {
   // ompl::time::point timeStart = time::now();
    collision::cuda::setResults(d_results_, 0, 1);

    // collision check
    collision::cuda::checkCollisionFast(num_of_robot_states_, d_robot_states_, num_of_obstacle_states_, d_obstacle_states_, d_results_, d_robot_, d_obstacle_);


    //double planningTime = time::seconds(time::now() - timeStart);
  //  printf("Collision checking time is  %f sec\n", planningTime);
    collision::cuda::downloadResults(h_results_, d_results_, 1);
    for (int i = 0; i < 1; ++i) {
      if(*(h_results_ + i)) {
        return true;
#ifdef DEBUG_COLLISION_CHECK
        printf("state %d found collision!\n", i);
#endif
      } else {
#ifdef DEBUG_COLLISION_CHECK
        printf("state %d has no collision!\n", i);
#endif
      }
    }
    return false;
  }

  void CUDACollisionCheck::clear(void)
  {
      reset();
  }

  void CUDACollisionCheck::reset()
  {
    // TODO:

    collision::cuda::reset();

  }

  void CUDACollisionCheck::uploadData(  GMesh* h_robot,   GMesh* h_obstacle,   std::vector<SerializedStates>& h_robot_states,   std::vector<SerializedStates>& h_obstacle_states)
  {
    collision::cuda::uploadData(h_robot, h_obstacle, h_robot_states, h_obstacle_states, d_robot_, d_obstacle_, d_robot_states_, d_obstacle_states_);
  }
}
