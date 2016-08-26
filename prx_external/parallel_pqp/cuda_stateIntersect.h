#ifndef CUDA_STATE_INTERSECT_H
#define CUDA_STATE_INTERSECT_H

#include "collision/cuda_obb.h"
#include "cuda_statePrimitives.h"
#include "collision/cuda_intersect_nodes.h"
#include "collision/cuda_intersect_tritri.h"

//#define DEBUG_STEP_VALUES

__device__ void transformVector(float3* vertexIn, const float* state, float3* vertexOut, bool rotationOnly)
{
	float rotMat[9];

	const float* quaternion = state + 3;
	const float* translation = state;

	// convert quaternion to matrix
	rotMat[0] = 1 - 2 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2]);
	rotMat[1] = 2 * (quaternion[0] * quaternion[1] - quaternion[2] * quaternion[3]);
	rotMat[2] = 2 * (quaternion[0] * quaternion[2] + quaternion[1] * quaternion[3]);

	rotMat[3] = 2 * (quaternion[0] * quaternion[1] + quaternion[2] * quaternion[3]);
	rotMat[4] = 1 - 2 * (quaternion[0] * quaternion[0] + quaternion[2] * quaternion[2]);
	rotMat[5] = 2 * (quaternion[1] * quaternion[2] - quaternion[0] * quaternion[3]);

	rotMat[6] = 2 * (quaternion[0] * quaternion[2] - quaternion[1] * quaternion[3]);
	rotMat[7] = 2 * (quaternion[1] * quaternion[2] + quaternion[0] * quaternion[3]);
	rotMat[8] = 1 - 2 * (quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1]);

	float3 rotated;
	rotated.x = rotMat[0] * vertexIn->x + rotMat[1] * vertexIn->y + rotMat[2] * vertexIn->z;
	rotated.y = rotMat[3] * vertexIn->x + rotMat[4] * vertexIn->y + rotMat[5] * vertexIn->z;
	rotated.z = rotMat[6] * vertexIn->x + rotMat[7] * vertexIn->y + rotMat[8] * vertexIn->z;

	float3 translated = rotated;

	if (!rotationOnly)
	{
		translated.x += translation[0];
		translated.y += translation[1];
		translated.z += translation[2];
	}

	*vertexOut = translated;
}

__device__ void transformByState(OBB& bv, const float* state)
{
	transformVector(&bv.center, state, &bv.center, false);
	transformVector(&bv.axis1, state, &bv.axis1, true);
	transformVector(&bv.axis2, state, &bv.axis2, true);
	transformVector(&bv.axis3, state, &bv.axis3, true);
}

template <class TreeNode, class BV, class stackElem, int StackFullDefault, typename stackElemComponent>
class BVHTwoStatesFastCollideFunctor
{
public:
	__device__ int operator()(const TreeNode* tree_object1, const GPUVertex* vertex_object1, const uint3* tri_object1, const float* robot_state,
		const TreeNode* tree_object2, const GPUVertex* vertex_object2, const uint3* tri_object2, const float* object_state) const
	{
		__shared__ stackElem traversalStacks[COLLISION_THREADS * COLLISION_STACK_SIZE];
//		__shared__ bool foundCollision = false;

		int sharedDataIdx = threadIdx.x;

		int stackPtr = sharedDataIdx;
		stackElem work_pair;
		work_pair.x = 1;
		work_pair.y = 1;

		int result = COLLISION_FREE;
		int counter = 0;

		int threadActive = 1;
		while(1)
		{
			int blockActive = __syncthreads_or(threadActive);
			if(blockActive == 0) {
//				result == IN_COLLISION;
				break;
			}

			if (!threadActive)
				continue;
      // threadActive == 0, means found collision. Then blockActive ==  0
//			int blockActive = __syncthreads_or(threadActive);
//			if(blockActive == 0)
//				break;

//      if (threadActive == 0) {
//        break;
//      }

			++counter;

			TreeNode node1 = tree_object1[work_pair.x];
			TreeNode node2 = tree_object2[work_pair.y];

      // transform coordinate of node1.bbox to coordinate of environment
      transformByState(node1.bbox, robot_state);
      transformByState(node2.bbox, object_state);

			result = intersect<BV>(node1.bbox, node2.bbox);
#ifdef DEBUG_STEP_VALUES
      printf("node 1 center is (%f, %f, %f)\n", node1.bbox.center.x, node1.bbox.center.y, node1.bbox.center.z);
      printf("node 2 center is (%f, %f, %f)\n", node2.bbox.center.x, node2.bbox.center.y, node2.bbox.center.z);

      printf("result in this iteration is : %d.\n", result);
#endif
			if (result) // collision
			{
				// 3 cases:
				//  - both are inner nodes: create 4 new intersections with children
				//  - one is leaf, one is inner: intersect leaf with children of other
				//  - both are leafs: write triangle-triangle pair to list
				if(node1.isLeaf() && node2.isLeaf())    // both leaf
				{
					int idx = node1.getTriID();
					int idx2 = node2.getTriID();

					float3 lVertices[3];
					float3 rVertices[3];

					lVertices[0] = vertex_object1[idx * 3].v;
					lVertices[1] = vertex_object1[idx * 3 + 1].v;
					lVertices[2] = vertex_object1[idx * 3 + 2].v;
					rVertices[0] = vertex_object2[idx2 * 3].v;
					rVertices[1] = vertex_object2[idx2 * 3 + 1].v;
					rVertices[2] = vertex_object2[idx2 * 3 + 2].v;

					float3 *l1, *l2, *l3, *r1, *r2, *r3;
					// transformed robot vertices
					l1 = &lVertices[0];
					l2 = &lVertices[1];
					l3 = &lVertices[2];
					transformVector(l1, robot_state, l1, false);
					transformVector(l2, robot_state, l2, false);
					transformVector(l3, robot_state, l3, false);

					r1 = &rVertices[0];
					r2 = &rVertices[1];
					r3 = &rVertices[2];

					result = triangleIntersection(*l1, *l2, *l3, *r1, *r2, *r3);
					if (result == IN_COLLISION)
					{
#ifdef DEBUG_STEP_VALUES
            printf ("BVHTwoStatesCollideFunctor:: found collision!!!!\n");
#endif
						threadActive = 0;
						continue;
					}
				}
				else if(node2.isLeaf() || (!node1.isLeaf() && (node1.bbox.getSize() > node2.bbox.getSize())))
				{
          // set work_pair.x as leftchild of node1, 
          // and store node1's rightchild into stack
          // DFS
					work_pair.x = node1.getLeftChild();

					if(stackPtr >= COLLISION_THREADS * COLLISION_STACK_SIZE)
					{
            printf("Stack overflow!!");
						result = StackFullDefault;
						threadActive = 0;
						continue;
					}
					traversalStacks[stackPtr].x = work_pair.x + 1;
					traversalStacks[stackPtr].y = work_pair.y;
					stackPtr += COLLISION_THREADS;

					continue;
				}
				else
				{
					work_pair.y = node2.getLeftChild();

					if(stackPtr >= COLLISION_THREADS * COLLISION_STACK_SIZE)
					{
            printf("Stack overflow!!");
						result = StackFullDefault;
						threadActive = 0;
						continue;
					}
					traversalStacks[stackPtr].x = work_pair.x;
					traversalStacks[stackPtr].y = work_pair.y + 1;
					stackPtr += COLLISION_THREADS;

					continue;
				}
			}

			stackPtr -= COLLISION_THREADS;

			// stack empty? then no intersection
			if(stackPtr < (int)sharedDataIdx)
			{
#ifdef DEBUG_STEP_VALUES
        printf ("BVHTwoStatesCollideFunctor:: no collision!!!!\n");
#endif
				threadActive = 0;
				continue;
			}
			work_pair.x = traversalStacks[stackPtr].x;
			work_pair.y = traversalStacks[stackPtr].y;
		}

#ifdef DEBUG_STEP_VALUES
		printf("BVH traverse : %d\n", counter);

    printf("result = %d, IN_COLLISION = %d \n", result, IN_COLLISION);
#endif
		return result;
	}
};


template <class TreeNode, class BV, class stackElem, int StackFullDefault, typename stackElemComponent>
class BVHTwoStatesCollideFunctor
{
public:
	__device__ int operator()(const TreeNode* tree_object1, const GPUVertex* vertex_object1, const uint3* tri_object1, const float* robot_state,
		const TreeNode* tree_object2, const GPUVertex* vertex_object2, const uint3* tri_object2, const float* object_state) const
	{
		__shared__ stackElem traversalStacks[COLLISION_THREADS * COLLISION_STACK_SIZE];

		int sharedDataIdx = threadIdx.x;

		int stackPtr = sharedDataIdx;
		stackElem work_pair;
		work_pair.x = 1;
		work_pair.y = 1;

		int result = COLLISION_FREE;
		int counter = 0;

		int threadActive = 1;
		while(1)
		{
//			int blockActive = __syncthreads_or(threadActive);
//			if(blockActive == 0)
//				break;

//			if (!threadActive)
//				continue;
      // threadActive == 0, means found collision. Then blockActive ==  0
//			int blockActive = __syncthreads_or(threadActive);
//			if(blockActive == 0)
//				break;

      if (threadActive == 0) {
        break;
      }

			++counter;

			TreeNode node1 = tree_object1[work_pair.x];
			TreeNode node2 = tree_object2[work_pair.y];

      // transform coordinate of node1.bbox to coordinate of environment
      transformByState(node1.bbox, robot_state);
      transformByState(node2.bbox, object_state);

			result = intersect<BV>(node1.bbox, node2.bbox);
#ifdef DEBUG_STEP_VALUES
      printf("node 1 center is (%f, %f, %f)\n", node1.bbox.center.x, node1.bbox.center.y, node1.bbox.center.z);
      printf("node 2 center is (%f, %f, %f)\n", node2.bbox.center.x, node2.bbox.center.y, node2.bbox.center.z);

      printf("result in this iteration is : %d.\n", result);
#endif
			if (result) // collision
			{
				// 3 cases:
				//  - both are inner nodes: create 4 new intersections with children
				//  - one is leaf, one is inner: intersect leaf with children of other
				//  - both are leafs: write triangle-triangle pair to list
				if(node1.isLeaf() && node2.isLeaf())    // both leaf
				{
					int idx = node1.getTriID();
					int idx2 = node2.getTriID();

					float3 lVertices[3];
					float3 rVertices[3];

					lVertices[0] = vertex_object1[idx * 3].v;
					lVertices[1] = vertex_object1[idx * 3 + 1].v;
					lVertices[2] = vertex_object1[idx * 3 + 2].v;
					rVertices[0] = vertex_object2[idx2 * 3].v;
					rVertices[1] = vertex_object2[idx2 * 3 + 1].v;
					rVertices[2] = vertex_object2[idx2 * 3 + 2].v;

					float3 *l1, *l2, *l3, *r1, *r2, *r3;
					// transformed robot vertices
					l1 = &lVertices[0];
					l2 = &lVertices[1];
					l3 = &lVertices[2];
					transformVector(l1, robot_state, l1, false);
					transformVector(l2, robot_state, l2, false);
					transformVector(l3, robot_state, l3, false);

					r1 = &rVertices[0];
					r2 = &rVertices[1];
					r3 = &rVertices[2];

					result = triangleIntersection(*l1, *l2, *l3, *r1, *r2, *r3);
					if (result == IN_COLLISION)
					{
#ifdef DEBUG_STEP_VALUES
            printf ("BVHTwoStatesCollideFunctor:: found collision!!!!\n");
#endif
						threadActive = 0;
						continue;
					}
				}
				else if(node2.isLeaf() || (!node1.isLeaf() && (node1.bbox.getSize() > node2.bbox.getSize())))
				{
          // set work_pair.x as leftchild of node1, 
          // and store node1's rightchild into stack
          // DFS
					work_pair.x = node1.getLeftChild();

					if(stackPtr >= COLLISION_THREADS * COLLISION_STACK_SIZE)
					{
            printf("Stack overflow!!");
						result = StackFullDefault;
						threadActive = 0;
						continue;
					}
					traversalStacks[stackPtr].x = work_pair.x + 1;
					traversalStacks[stackPtr].y = work_pair.y;
					stackPtr += COLLISION_THREADS;

					continue;
				}
				else
				{
					work_pair.y = node2.getLeftChild();

					if(stackPtr >= COLLISION_THREADS * COLLISION_STACK_SIZE)
					{
            printf("Stack overflow!!");
						result = StackFullDefault;
						threadActive = 0;
						continue;
					}
					traversalStacks[stackPtr].x = work_pair.x;
					traversalStacks[stackPtr].y = work_pair.y + 1;
					stackPtr += COLLISION_THREADS;

					continue;
				}
			}

			stackPtr -= COLLISION_THREADS;

			// stack empty? then no intersection
			if(stackPtr < (int)sharedDataIdx)
			{
#ifdef DEBUG_STEP_VALUES
        printf ("BVHTwoStatesCollideFunctor:: no collision!!!!\n");
#endif
				threadActive = 0;
				continue;
			}
			work_pair.x = traversalStacks[stackPtr].x;
			work_pair.y = traversalStacks[stackPtr].y;
		}

#ifdef DEBUG_STEP_VALUES
		printf("BVH traverse : %d\n", counter);

    printf("result = %d, IN_COLLISION = %d \n", result, IN_COLLISION);
#endif
		return result;
	}
};

template <class TreeNode, class BV, class stackElem, int StackFullDefault, typename stackElemComponent>
class BVHTwoStatesCollideNoStackFunctor
{
public:
	__device__ int operator()(const TreeNode* tree_object1, const GPUVertex* vertex_object1, const uint3* tri_object1, const float* robot_state,
		const TreeNode* tree_object2, const GPUVertex* vertex_object2, const uint3* tri_object2) const
	{
		stackElem work_pair;
		work_pair.x = 1;
		work_pair.y = 1;

		int result = COLLISION_FREE;
		int counter = 0;

		int threadActive = 1;
		while(1)
		{

			int blockActive = __syncthreads_or(threadActive);
			if(blockActive == 0)
				break;

			if (!threadActive)
				continue;

			++counter;

			TreeNode node1 = tree_object1[work_pair.x];
			TreeNode node2 = tree_object2[work_pair.y];

			transformByState(node1.bbox, robot_state);

			result = intersect<BV>(node1.bbox, node2.bbox);
			if (result)
			{
				// 3 cases:
				//  - both are inner nodes: create 4 new intersections with children
				//  - one is leaf, one is inner: intersect leaf with children of other
				//  - both are leafs: write triangle-triangle pair to list
				if(node1.isLeaf() && node2.isLeaf())    // both leaf
				{
					int idx = node1.getTriID();
					int idx2 = node2.getTriID();

					float3 lVertices[3];
					float3 rVertices[3];

					lVertices[0] = vertex_object1[idx * 3].v;
					lVertices[1] = vertex_object1[idx * 3 + 1].v;
					lVertices[2] = vertex_object1[idx * 3 + 2].v;
					rVertices[0] = vertex_object2[idx2 * 3].v;
					rVertices[1] = vertex_object2[idx2 * 3 + 1].v;
					rVertices[2] = vertex_object2[idx2 * 3 + 2].v;

					float3 *l1, *l2, *l3, *r1, *r2, *r3;
					// transformed robot vertices
					l1 = &lVertices[0];
					l2 = &lVertices[1];
					l3 = &lVertices[2];
					transformVector(l1, robot_state, l1, false);
					transformVector(l2, robot_state, l2, false);
					transformVector(l3, robot_state, l3, false);

					r1 = &rVertices[0];
					r2 = &rVertices[1];
					r3 = &rVertices[2];

					result = triangleIntersection(*l1, *l2, *l3, *r1, *r2, *r3);
					if (result == IN_COLLISION)
					{
						threadActive = 0;
						continue;
					}
				}
				else if(node2.isLeaf())
				{
					work_pair.x = node1.getLeftChild();

					continue;
				}
				else
				{
					work_pair.y = node2.getLeftChild();

					continue;
				}
			}

			stackElemComponent nextNode = getNextOBBNodeID(work_pair.x);
			if (nextNode == 0)
			{
				nextNode = getNextOBBNodeID(work_pair.y);
				if (nextNode == 0)
				{
					threadActive = 0;
					continue;
				}
				else
				{
					work_pair.x = 1;
					work_pair.y = nextNode;
				}
			}
			else 
				work_pair.x = nextNode;
		}
#ifdef DEBUG_STEP_VALUES
		printf("BVH traverse : %d\n", counter);
#endif
		return result;
	}
};

template <class TreeNode, class BV, class stackElem, int StackFullDefault, typename stackElemComponent>
class BVHStateCollideFunctor
{
public:
	__device__ int operator()(const TreeNode* tree_object1, const GPUVertex* vertex_object1, const uint3* tri_object1, const float* robot_state,
		const TreeNode* tree_object2, const GPUVertex* vertex_object2, const uint3* tri_object2) const
	{
		__shared__ stackElem traversalStacks[COLLISION_THREADS * COLLISION_STACK_SIZE];

		int sharedDataIdx = threadIdx.x;

		int stackPtr = sharedDataIdx;
		stackElem work_pair;
		work_pair.x = 1;
		work_pair.y = 1;

		int result = COLLISION_FREE;
		int counter = 0;

		int threadActive = 1;
		while(1)
		{

			int blockActive = __syncthreads_or(threadActive);
			if(blockActive == 0)
				break;

			if (!threadActive)
				continue;

			++counter;

			TreeNode node1 = tree_object1[work_pair.x];
			TreeNode node2 = tree_object2[work_pair.y];

      // transform coordinate of node1.bbox to coordinate of environment
			transformByState(node1.bbox, robot_state);

			result = intersect<BV>(node1.bbox, node2.bbox);
			if (result) // collision
			{
				// 3 cases:
				//  - both are inner nodes: create 4 new intersections with children
				//  - one is leaf, one is inner: intersect leaf with children of other
				//  - both are leafs: write triangle-triangle pair to list
				if(node1.isLeaf() && node2.isLeaf())    // both leaf
				{
					int idx = node1.getTriID();
					int idx2 = node2.getTriID();

					float3 lVertices[3];
					float3 rVertices[3];

					lVertices[0] = vertex_object1[idx * 3].v;
					lVertices[1] = vertex_object1[idx * 3 + 1].v;
					lVertices[2] = vertex_object1[idx * 3 + 2].v;
					rVertices[0] = vertex_object2[idx2 * 3].v;
					rVertices[1] = vertex_object2[idx2 * 3 + 1].v;
					rVertices[2] = vertex_object2[idx2 * 3 + 2].v;

					float3 *l1, *l2, *l3, *r1, *r2, *r3;
					// transformed robot vertices
					l1 = &lVertices[0];
					l2 = &lVertices[1];
					l3 = &lVertices[2];
					transformVector(l1, robot_state, l1, false);
					transformVector(l2, robot_state, l2, false);
					transformVector(l3, robot_state, l3, false);

					r1 = &rVertices[0];
					r2 = &rVertices[1];
					r3 = &rVertices[2];

					result = triangleIntersection(*l1, *l2, *l3, *r1, *r2, *r3);
					if (result == IN_COLLISION)
					{
						threadActive = 0;
						continue;
					}
				}
				else if(node2.isLeaf() || (!node1.isLeaf() && (node1.bbox.getSize() > node2.bbox.getSize())))
				{
          // set work_pair.x as leftchild of node1, 
          // and store node1's rightchild into stack
          // DFS
					work_pair.x = node1.getLeftChild();

					if(stackPtr >= COLLISION_THREADS * COLLISION_STACK_SIZE)
					{
						result = StackFullDefault;
						threadActive = 0;
						continue;
					}
					traversalStacks[stackPtr].x = work_pair.x + 1;
					traversalStacks[stackPtr].y = work_pair.y;
					stackPtr += COLLISION_THREADS;

					continue;
				}
				else
				{
					work_pair.y = node2.getLeftChild();

					if(stackPtr >= COLLISION_THREADS * COLLISION_STACK_SIZE)
					{
						result = StackFullDefault;
						threadActive = 0;
						continue;
					}
					traversalStacks[stackPtr].x = work_pair.x;
					traversalStacks[stackPtr].y = work_pair.y + 1;
					stackPtr += COLLISION_THREADS;

					continue;
				}
			}

			stackPtr -= COLLISION_THREADS;

			// stack empty? then no intersection
			if(stackPtr < (int)sharedDataIdx)
			{
				threadActive = 0;
				continue;
			}
			work_pair.x = traversalStacks[stackPtr].x;
			work_pair.y = traversalStacks[stackPtr].y;
		}

		//printf("BVH traverse : %d\n", counter);

		return result;
	}
};

template <class TreeNode, class BV, class stackElem, int StackFullDefault, typename stackElemComponent>
class BVHStateCollideNoStackFunctor
{
public:
	__device__ int operator()(const TreeNode* tree_object1, const GPUVertex* vertex_object1, const uint3* tri_object1, const float* robot_state,
		const TreeNode* tree_object2, const GPUVertex* vertex_object2, const uint3* tri_object2) const
	{
		stackElem work_pair;
		work_pair.x = 1;
		work_pair.y = 1;

		int result = COLLISION_FREE;
		int counter = 0;

		int threadActive = 1;
		while(1)
		{

			int blockActive = __syncthreads_or(threadActive);
			if(blockActive == 0)
				break;

			if (!threadActive)
				continue;

			++counter;

			TreeNode node1 = tree_object1[work_pair.x];
			TreeNode node2 = tree_object2[work_pair.y];

			transformByState(node1.bbox, robot_state);

			result = intersect<BV>(node1.bbox, node2.bbox);
			if (result)
			{
				// 3 cases:
				//  - both are inner nodes: create 4 new intersections with children
				//  - one is leaf, one is inner: intersect leaf with children of other
				//  - both are leafs: write triangle-triangle pair to list
				if(node1.isLeaf() && node2.isLeaf())    // both leaf
				{
					int idx = node1.getTriID();
					int idx2 = node2.getTriID();

					float3 lVertices[3];
					float3 rVertices[3];

					lVertices[0] = vertex_object1[idx * 3].v;
					lVertices[1] = vertex_object1[idx * 3 + 1].v;
					lVertices[2] = vertex_object1[idx * 3 + 2].v;
					rVertices[0] = vertex_object2[idx2 * 3].v;
					rVertices[1] = vertex_object2[idx2 * 3 + 1].v;
					rVertices[2] = vertex_object2[idx2 * 3 + 2].v;

					float3 *l1, *l2, *l3, *r1, *r2, *r3;
					// transformed robot vertices
					l1 = &lVertices[0];
					l2 = &lVertices[1];
					l3 = &lVertices[2];
					transformVector(l1, robot_state, l1, false);
					transformVector(l2, robot_state, l2, false);
					transformVector(l3, robot_state, l3, false);

					r1 = &rVertices[0];
					r2 = &rVertices[1];
					r3 = &rVertices[2];

					result = triangleIntersection(*l1, *l2, *l3, *r1, *r2, *r3);
					if (result == IN_COLLISION)
					{
						threadActive = 0;
						continue;
					}
				}
				else if(node2.isLeaf())
				{
					work_pair.x = node1.getLeftChild();

					continue;
				}
				else
				{
					work_pair.y = node2.getLeftChild();

					continue;
				}
			}

			stackElemComponent nextNode = getNextOBBNodeID(work_pair.x);
			if (nextNode == 0)
			{
				nextNode = getNextOBBNodeID(work_pair.y);
				if (nextNode == 0)
				{
					threadActive = 0;
					continue;
				}
				else
				{
					work_pair.x = 1;
					work_pair.y = nextNode;
				}
			}
			else 
				work_pair.x = nextNode;
		}

		//printf("BVH traverse : %d\n", counter);

		return result;
	}
};

template <class TreeNode, class BV, class stackElem, int StackFullDefault, typename stackElemComponent>
class BVHNoStateCollideFunctor
{
public:
	__device__ int operator()(const TreeNode* tree_object1, const GPUVertex* vertex_object1, const uint3* tri_object1, 
		const TreeNode* tree_object2, const GPUVertex* vertex_object2, const uint3* tri_object2) const
	{
		__shared__ stackElem traversalStacks[COLLISION_THREADS * COLLISION_STACK_SIZE];

		int sharedDataIdx = threadIdx.x;

		int stackPtr = sharedDataIdx;
		stackElem work_pair;
		work_pair.x = 1;
		work_pair.y = 1;

		int result = COLLISION_FREE;
		int counter = 0;

		int threadActive = 1;
		while(1)
		{

			int blockActive = __syncthreads_or(threadActive);
			if(blockActive == 0)
				break;

			if (!threadActive)
				continue;

			++counter;

			TreeNode node1 = tree_object1[work_pair.x];
			TreeNode node2 = tree_object2[work_pair.y];

//			transformByState(node1.bbox, robot_state);

			result = intersect<BV>(node1.bbox, node2.bbox);
			if (result) // collision
			{
				// 3 cases:
				//  - both are inner nodes: create 4 new intersections with children
				//  - one is leaf, one is inner: intersect leaf with children of other
				//  - both are leafs: write triangle-triangle pair to list
				if(node1.isLeaf() && node2.isLeaf())    // both leaf
				{
					int idx = node1.getTriID();
					int idx2 = node2.getTriID();

					float3 lVertices[3];
					float3 rVertices[3];

					lVertices[0] = vertex_object1[idx * 3].v;
					lVertices[1] = vertex_object1[idx * 3 + 1].v;
					lVertices[2] = vertex_object1[idx * 3 + 2].v;
					rVertices[0] = vertex_object2[idx2 * 3].v;
					rVertices[1] = vertex_object2[idx2 * 3 + 1].v;
					rVertices[2] = vertex_object2[idx2 * 3 + 2].v;

					float3 *l1, *l2, *l3, *r1, *r2, *r3;
					// transformed robot vertices
					l1 = &lVertices[0];
					l2 = &lVertices[1];
					l3 = &lVertices[2];
	//				transformVector(l1, robot_state, l1, false);
	//				transformVector(l2, robot_state, l2, false);
	//				transformVector(l3, robot_state, l3, false);

					r1 = &rVertices[0];
					r2 = &rVertices[1];
					r3 = &rVertices[2];

					result = triangleIntersection(*l1, *l2, *l3, *r1, *r2, *r3);
					if (result == IN_COLLISION)
					{
						threadActive = 0;
						continue;
					}
				}
				else if(node2.isLeaf() || (!node1.isLeaf() && (node1.bbox.getSize() > node2.bbox.getSize())))
				{
					work_pair.x = node1.getLeftChild();

					if(stackPtr >= COLLISION_THREADS * COLLISION_STACK_SIZE)
					{
						result = StackFullDefault;
						threadActive = 0;
						continue;
					}
					traversalStacks[stackPtr].x = work_pair.x + 1;
					traversalStacks[stackPtr].y = work_pair.y;
					stackPtr += COLLISION_THREADS;

					continue;
				}
				else
				{
					work_pair.y = node2.getLeftChild();

					if(stackPtr >= COLLISION_THREADS * COLLISION_STACK_SIZE)
					{
						result = StackFullDefault;
						threadActive = 0;
						continue;
					}
					traversalStacks[stackPtr].x = work_pair.x;
					traversalStacks[stackPtr].y = work_pair.y + 1;
					stackPtr += COLLISION_THREADS;

					continue;
				}
			}

			stackPtr -= COLLISION_THREADS;

			// stack empty? then no intersection
			if(stackPtr < (int)sharedDataIdx)
			{
				threadActive = 0;
				continue;
			}
			work_pair.x = traversalStacks[stackPtr].x;
			work_pair.y = traversalStacks[stackPtr].y;
		}

		//printf("BVH traverse : %d\n", counter);

		return result;
	}
};

template <class TreeNode, class BV, class stackElem, int StackFullDefault, typename stackElemComponent>
class BVHNoStateCollideNoStackFunctor
{
public:
	__device__ int operator()(const TreeNode* tree_object1, const GPUVertex* vertex_object1, const uint3* tri_object1,
		const TreeNode* tree_object2, const GPUVertex* vertex_object2, const uint3* tri_object2) const
	{
		stackElem work_pair;
		work_pair.x = 1;
		work_pair.y = 1;

		int result = COLLISION_FREE;
		int counter = 0;

		int threadActive = 1;
		while(1)
		{

			int blockActive = __syncthreads_or(threadActive);
			if(blockActive == 0)
				break;

			if (!threadActive)
				continue;

			++counter;

			TreeNode node1 = tree_object1[work_pair.x];
			TreeNode node2 = tree_object2[work_pair.y];

		//	transformByState(node1.bbox, robot_state);

			result = intersect<BV>(node1.bbox, node2.bbox);
			if (result)
			{
				// 3 cases:
				//  - both are inner nodes: create 4 new intersections with children
				//  - one is leaf, one is inner: intersect leaf with children of other
				//  - both are leafs: write triangle-triangle pair to list
				if(node1.isLeaf() && node2.isLeaf())    // both leaf
				{
					int idx = node1.getTriID();
					int idx2 = node2.getTriID();

					float3 lVertices[3];
					float3 rVertices[3];

					lVertices[0] = vertex_object1[idx * 3].v;
					lVertices[1] = vertex_object1[idx * 3 + 1].v;
					lVertices[2] = vertex_object1[idx * 3 + 2].v;
					rVertices[0] = vertex_object2[idx2 * 3].v;
					rVertices[1] = vertex_object2[idx2 * 3 + 1].v;
					rVertices[2] = vertex_object2[idx2 * 3 + 2].v;

					float3 *l1, *l2, *l3, *r1, *r2, *r3;
					// transformed robot vertices
					l1 = &lVertices[0];
					l2 = &lVertices[1];
					l3 = &lVertices[2];
//					transformVector(l1, robot_state, l1, false);
//					transformVector(l2, robot_state, l2, false);
//					transformVector(l3, robot_state, l3, false);

					r1 = &rVertices[0];
					r2 = &rVertices[1];
					r3 = &rVertices[2];

					result = triangleIntersection(*l1, *l2, *l3, *r1, *r2, *r3);
					if (result == IN_COLLISION)
					{
						threadActive = 0;
						continue;
					}
				}
				else if(node2.isLeaf())
				{
					work_pair.x = node1.getLeftChild();

					continue;
				}
				else
				{
					work_pair.y = node2.getLeftChild();

					continue;
				}
			}

			stackElemComponent nextNode = getNextOBBNodeID(work_pair.x);
			if (nextNode == 0)
			{
				nextNode = getNextOBBNodeID(work_pair.y);
				if (nextNode == 0)
				{
					threadActive = 0;
					continue;
				}
				else
				{
					work_pair.x = 1;
					work_pair.y = nextNode;
				}
			}
			else 
				work_pair.x = nextNode;
		}

		//printf("BVH traverse : %d\n", counter);

		return result;
	}
};
#endif
