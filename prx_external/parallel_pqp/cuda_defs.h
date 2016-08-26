#ifndef __CUDA_DEFS_H_
#define __CUDA_DEFS_H_

#define QUEUE_NTASKS 128
#define LOG_QUEUE_NTASKS 7
#define QUEUE_SIZE_PER_TASK 600
#define QUEUE_SIZE_PER_TASK_GLOBAL 50000
#define QUEUE_IDLETASKS_FOR_ABORT QUEUE_NTASKS*2/3
#define TRAVERSAL_THREADS 64
#define BALANCE_THREADS 512 //256
#define QUEUE_SIZE_PER_TASK_INIT TRAVERSAL_THREADS*3
#define QUEUE_NTASKS_PER_CIRCLE 50000
#define QUEUE_REFILL_THRESHOLD QUEUE_NTASKS_PER_CIRCLE*1.5

#define COLLISIONFREE_RES 1
#define COLLISION_RES 0
#define COLLISIONFREE_TAG (make_ushort2(-1, -1))

#define COLLISION_THREADS 64
#define COLLISION_STACK_SIZE 16
#define GENERAL_THREADS 256

#define BVH_PACKET_BLOCK_WIDTH 32
#define BVH_PACKET_BLOCK_HEIGHT 9
#define BVH_PACKET_STACK_SIZE 97
#define BVH_PACKET_OVERLAP_SUM_SIZE 32

#define CCD_MAX_INTERPOLATION 16
#define LOG_CCD_MAX_INTERPOLATION 4

#define WARP_SIZE 32
#define HALF_WARP_SIZE 16
#define TWO_WARP_SIZE 64
#define WARP_SIZE_LOG2 5

#define LBVH_GRID_LEVELS 10
#define AABB_BLOCK_SIZE 256
#define SMALLSPLIT_THRESHOLD WARP_SIZE
#define AABB_BOUNDINGBOX_THREADS 256

#define SCAN_NUM_BANKS 16
#define SCAN_LOG_NUM_BANKS 4

#define SEPARATE_MODE 0
#define ALLINONE_MODE 1
#define COLLISION_MODE ALLINONE_MODE

// how many triangles to have at the leaves of the tree (can be less,
// but not more)
#define SCENE_SPLIT_LEAFTRIS 0

#define NSPLIT_CANDIDATES (2*WARP_SIZE)
#define NSPLIT_AXES 1
#define SCAN_THREADS (NSPLIT_CANDIDATES * NSPLIT_AXES)
#define SPLIT_THREADS NSPLIT_CANDIDATES

// Threads used for main work queue compaction kernel
#define SPLITLIST_COMPACTION_THREADS 32

#define CONDOREMU_SYNC(cond) if (cond) __syncthreads()


#define SCAN_CONFLICT_FREE_OFFSET(index) ((index) >> SCAN_LOG_NUM_BANKS)

#define GPUMALLOC(D_POINTER, SIZE) CUDA_SAFE_CALL(cudaMalloc(D_POINTER, SIZE))
#define CPUMALLOC(H_POINTER, SIZE) CUDA_SAFE_CALL(cudaMallocHost(H_POINTER, SIZE))

#define CPUFREE(H_POINTER) \
	do { \
		if(H_POINTER) CUDA_SAFE_CALL(cudaFreeHost(H_POINTER)); \
		H_POINTER = NULL; \
	} while(0)

#define GPUFREE(D_POINTER) \
	do { \
		if(D_POINTER) CUDA_SAFE_CALL(cudaFree(D_POINTER)); \
		D_POINTER = NULL; \
	} while(0)

#define TOGPU(D_POINTER, H_POINTER, SIZE) CUDA_SAFE_CALL(cudaMemcpy(D_POINTER, H_POINTER, SIZE, cudaMemcpyHostToDevice))
#define FROMGPU(H_POINTER, D_POINTER, SIZE) CUDA_SAFE_CALL(cudaMemcpy(H_POINTER, D_POINTER, SIZE, cudaMemcpyDeviceToHost))
#define GPUTOGPU(D_TO, D_FROM, SIZE) CUDA_SAFE_CALL(cudaMemcpy(D_TO, D_FROM, SIZE, cudaMemcpyDeviceToDevice))

//#define TOGPU_CONSTANT(D_POINTER, H_POINTER, SIZE) CUDA_SAFE_CALL(cudaMemcpyToSymbol(D_POINTER, H_POINTER, SIZE, cudaMemcpyHostToDevice))
//#define GPUTOGPU_CONSTANT(D_TO, D_FROM, SIZE) CUDA_SAFE_CALL(cudaMemcpyToSymbol(D_TO, D_FORM, SIZE, cudaMemcpyDeviceToDevice))
#define TOGPU_CONSTANT(D_POINTER, H_POINTER, SIZE) CUDA_SAFE_CALL(cudaMemcpyToSymbol(D_POINTER, H_POINTER, SIZE, 0, cudaMemcpyHostToDevice))

#define GPUMEMSET(D_POINTER, INTVALUE, SIZE) CUDA_SAFE_CALL(cudaMemset(D_POINTER, INTVALUE, SIZE))

#ifdef __DEVICE_EMULATION__
#define EMULATION_ONLY_SYNC() __syncthreads()
#define CONDOREMU_SYNC(cond) __syncthreads()
#else
#define EMULATION_ONLY_SYNC()
#define CONDOREMU_SYNC(cond) if (cond) __syncthreads()
#endif

#define CUDA_UPPERROUND4(size) (ceilf(size/4.0f) * 4)

// chpark
enum RRT_TREE_ID
{
	NO_TREE = 0,
	START_TREE = 1,
	GOAL_TREE = 2,
	BOTH_TREE = 3,
};
const int STATE_SIZE = sizeof(float) * 7;
const int STATE_STRIDE = 8;
const int STATE_SIZE_STRIDE = sizeof(float) * STATE_STRIDE;

const int MAX_NODES = 100000;
const int MAX_SAMPLES = 200000;

enum COLLISION_RESULT
{
	COLLISION_FREE = 0,
	IN_COLLISION = 1,
	IN_COLLISION_WITH_NO_ADVANCE = 2,
};

#define EPSILON 1e-7f
#define FLOAT_MAX 1e+37f

//#define DEBUG_NEW_NODE
//#define DEBUG_NEW_SAMPLE

#endif
