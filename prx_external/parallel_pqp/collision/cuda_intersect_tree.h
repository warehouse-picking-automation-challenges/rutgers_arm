#ifndef __CUDA_INTERSECT_TREE_H_
#define __CUDA_INTERSECT_TREE_H_

#include "defs.h"
#include "cuda.h"
#include "cuda_intersect_nodes.h"
#include "cuda_transform.h"
#include "cuda_intersect_tritri.h"
#include "cuda_prefix.h"

__device__ void reduceSum_(int* items, const int tidx)
{
	items[tidx] += items[tidx^1];
	items[tidx] += items[tidx^2];
	items[tidx] += items[tidx^4];
	items[tidx] += items[tidx^8];
	items[tidx] += items[tidx^16];
}

__device__ void reduceSum_(volatile int* items, const int tidx)
{
	reduceSum_((int*)items, tidx);
}


template <class TreeNode, class BV, class stackElem, int StackFullDefault>
class BVHCollidePacket2Functor
{
public:
	__device__ int operator()(const TreeNode* tree_object1, const GPUVertex* vertex_object1, const uint3* tri_object1, GTransform& transform_object1,
	                          const TreeNode* tree_object2, const GPUVertex* vertex_object2, const uint3* tri_object2, GTransform& transform_object2) const
	{
		__shared__ volatile stackElem traversalStacks[BVH_PACKET_STACK_SIZE * BVH_PACKET_BLOCK_HEIGHT];
		__shared__ volatile int stackPtrs[BVH_PACKET_BLOCK_HEIGHT];
		
		volatile stackElem* stack = traversalStacks + BVH_PACKET_STACK_SIZE * threadIdx.y;
		volatile int& stackPtr = stackPtrs[threadIdx.y];
		
		
		int tidx = threadIdx.x;
		//int widx = threadIdx.y;
		stackElem work_pair;
		bool terminated = false;
		stackPtr = -1;
		work_pair.x = 0;
		work_pair.y = 0;
		
		{
			TreeNode node1 = tree_object1[work_pair.x];
			TreeNode node2 = tree_object2[work_pair.y];
			transform_object1.apply(node1.bbox);
			transform_object2.apply(node2.bbox);
			terminated = !intersect<BV>(node1.bbox, node2.bbox);
			if(__all(terminated)) return 0;
		}
		
		
		while(1)
		{
			TreeNode node1 = tree_object1[work_pair.x];
			TreeNode node2 = tree_object2[work_pair.y];
			
			
			if(!node1.isLeaf() && !node2.isLeaf())
			{
				int left1 = node1.getLeftChild() + work_pair.x;
				int right1 = left1 + 1;
				int left2 = node2.getLeftChild() + work_pair.y;
				int right2 = left2 + 1;
				
				TreeNode node1c, node2c;
				bool overlapRes0, overlapRes1, overlapRes2, overlapRes3;
				
				node1c = tree_object1[left1];
				node2c = tree_object2[left2];
				transform_object1.apply(node1c.bbox);
				transform_object2.apply(node2c.bbox);
				overlapRes0 = (!terminated && intersect<BV>(node1c.bbox, node2c.bbox));
				
				node1c = tree_object1[left1];
				node2c = tree_object2[right2];
				transform_object1.apply(node1c.bbox);
				transform_object2.apply(node2c.bbox);
				overlapRes1 = (!terminated && intersect<BV>(node1c.bbox, node2c.bbox));
				
				node1c = tree_object1[right1];
				node2c = tree_object2[left2];
				transform_object1.apply(node1c.bbox);
				transform_object2.apply(node2c.bbox);
				overlapRes2 = (!terminated && intersect<BV>(node1c.bbox, node2c.bbox));
				
				node1c = tree_object1[right1];
				node2c = tree_object2[right2];
				transform_object1.apply(node1c.bbox);
				transform_object2.apply(node2c.bbox);
				overlapRes3 = (!terminated && intersect<BV>(node1c.bbox, node2c.bbox));
				
				bool anyc0 = __any(overlapRes0);
				bool anyc1 = __any(overlapRes1);
				bool anyc2 = __any(overlapRes2);
				bool anyc3 = __any(overlapRes3);
				
				if(!anyc0 && !anyc1 && !anyc2 && !anyc3)
				{
					if(stackPtr < 0)
						break;
					else
					{
						work_pair.x = stack[stackPtr].x;
						work_pair.y = stack[stackPtr].y;
						if(tidx == 0)
							stackPtr--;
					}
				}
				else
				{
					bool work_pair_setted = false;
					
					if(anyc0)
					{
						work_pair_setted = true;
						work_pair.x = left1;
						work_pair.y = left2;
					}
					
					if(anyc1)
					{
						if(work_pair_setted)
						{
							if(tidx == 0)
							{
								++stackPtr;
								if(stackPtr >= BVH_PACKET_STACK_SIZE) break;
								stack[stackPtr].x = left1;
								stack[stackPtr].y = right2;
							}
						}
						else
						{
							work_pair_setted = true;
							work_pair.x = left1;
							work_pair.y = right2;
						}
					}
					
					if(anyc2)
					{
						if(work_pair_setted)
						{
							if(tidx == 0)
							{
								++stackPtr;
								if(stackPtr >= BVH_PACKET_STACK_SIZE) break;
								stack[stackPtr].x = right1;
								stack[stackPtr].y = left2;
							}
						}
						else
						{
							work_pair_setted = true;
							work_pair.x = right1;
							work_pair.y = left2;
						}
					}
					
					if(anyc3)
					{
						if(work_pair_setted)
						{
							if(tidx == 0)
							{
								++stackPtr;
								if(stackPtr >= BVH_PACKET_STACK_SIZE) break;
								stack[stackPtr].x = right1;
								stack[stackPtr].y = right2;
							}
						}
						else
						{
							work_pair_setted = true;
							work_pair.x = right1;
							work_pair.y = right2;
						}
					}
				}
			}
			else if(node1.isLeaf() && node2.isLeaf())    // both leaf
			{
				uint3 idx = tri_object1[node1.getTriID()];
				uint3 idx2 = tri_object2[node2.getTriID()];
				
				// transformed robot vertices
				float3 l1 = transform_object1.applyv(vertex_object1[idx.x].v);
				float3 l2 = transform_object1.applyv(vertex_object1[idx.y].v);
				float3 l3 = transform_object1.applyv(vertex_object1[idx.z].v);
				
				float3 r1 = transform_object2.applyv(vertex_object2[idx2.x].v);
				float3 r2 = transform_object2.applyv(vertex_object2[idx2.y].v);
				float3 r3 = transform_object2.applyv(vertex_object2[idx2.z].v);
				
				// intersect triangles:
				terminated = (terminated || triangleIntersection(l1, l2, l3, r1, r2, r3));
				
				if(__all(terminated)) break;
				
				if(stackPtr < 0)
					break;
				else
				{
					work_pair.x = stack[stackPtr].x;
					work_pair.y = stack[stackPtr].y;
					if(tidx == 0)
						stackPtr--;
				}
			}
			else if(node1.isLeaf())   	// left node is leaf
			{
				int left2 = node2.getLeftChild() + work_pair.y;
				int right2 = left2 + 1;
				
				TreeNode node1c, node2c;
				bool overlapRes0, overlapRes1;
				
				node1c = tree_object1[work_pair.x];
				node2c = tree_object2[left2];
				transform_object1.apply(node1c.bbox);
				transform_object2.apply(node2c.bbox);
				overlapRes0 = (!terminated && intersect<BV>(node1c.bbox, node2c.bbox));
				
				node1c = tree_object1[work_pair.x];
				node2c = tree_object2[right2];
				transform_object1.apply(node1c.bbox);
				transform_object2.apply(node2c.bbox);
				overlapRes1 = (!terminated && intersect<BV>(node1c.bbox, node2c.bbox));
				
				bool anyc0 = __any(overlapRes0);
				bool anyc1 = __any(overlapRes1);
				
				if(anyc0 && anyc1)
				{
					work_pair.y = left2;
					if(tidx == 0)
					{
						++stackPtr;
						if(stackPtr >= BVH_PACKET_STACK_SIZE) break;
						stack[stackPtr].x = work_pair.x;
						stack[stackPtr].y = right2;
					}
				}
				else if(anyc0)
				{
					work_pair.y = left2;
				}
				else if(anyc1)
				{
					work_pair.y = right2;
				}
				else
				{
					if(stackPtr < 0)
						break;
					else
					{
						work_pair.x = stack[stackPtr].x;
						work_pair.y = stack[stackPtr].y;
						if(tidx == 0)
							stackPtr--;
					}
				}
			}
			else
			{
				int left1 = node1.getLeftChild() + work_pair.x;
				int right1 = left1 + 1;
				
				TreeNode node1c, node2c;
				bool overlapRes0, overlapRes1;
				
				node1c = tree_object1[left1];
				node2c = tree_object2[work_pair.y];
				transform_object1.apply(node1c.bbox);
				transform_object2.apply(node2c.bbox);
				overlapRes0 = (!terminated && intersect<BV>(node1c.bbox, node2c.bbox));
				
				node1c = tree_object1[right1];
				node2c = tree_object2[work_pair.y];
				transform_object1.apply(node1c.bbox);
				transform_object2.apply(node2c.bbox);
				overlapRes1 = (!terminated && intersect<BV>(node1c.bbox, node2c.bbox));
				
				bool anyc0 = __any(overlapRes0);
				bool anyc1 = __any(overlapRes1);
				
				if(anyc0 && anyc1)
				{
					work_pair.x = left1;
					if(tidx == 0)
					{
						++stackPtr;
						if(stackPtr >= BVH_PACKET_STACK_SIZE) break;
						stack[stackPtr].x = right1;
						stack[stackPtr].y = work_pair.y;
					}
				}
				else if(anyc0)
				{
					work_pair.x = left1;
				}
				else if(anyc1)
				{
					work_pair.x = right1;
				}
				else
				{
					if(stackPtr < 0)
						break;
					else
					{
						work_pair.x = stack[stackPtr].x;
						work_pair.y = stack[stackPtr].y;
						if(tidx == 0)
							stackPtr--;
					}
				}
			}
		}
		
		return (terminated) ? 1 : 0;
	}
};




template <class TreeNode, class BV, class stackElem, int StackFullDefault>
class BVHCollidePacketFunctor_old
{
public:
	__device__ int operator()(const TreeNode* tree_object1, const GPUVertex* vertex_object1, const uint3* tri_object1, GTransform& transform_object1,
	                          const TreeNode* tree_object2, const GPUVertex* vertex_object2, const uint3* tri_object2, GTransform& transform_object2) const
	{
		__shared__ volatile stackElem traversalStacks[BVH_PACKET_STACK_SIZE * BVH_PACKET_BLOCK_HEIGHT];
		__shared__ volatile int stackPtrs[BVH_PACKET_BLOCK_HEIGHT];
		__shared__ volatile int sumBuffers[BVH_PACKET_OVERLAP_SUM_SIZE * BVH_PACKET_BLOCK_HEIGHT];
		
		volatile stackElem* stack = traversalStacks + BVH_PACKET_STACK_SIZE * threadIdx.y;
		volatile int* sumBuffer = sumBuffers + BVH_PACKET_OVERLAP_SUM_SIZE * threadIdx.y;
		volatile int& stackPtr = stackPtrs[threadIdx.y];
		
		
		int tidx = threadIdx.x;
		//int widx = threadIdx.y;
		stackElem work_pair;
		bool terminated = false;
		stackPtr = -1;
		work_pair.x = 0;
		work_pair.y = 0;
		
		if(tidx == 0)
		{
			for(int i = 0; i < BVH_PACKET_OVERLAP_SUM_SIZE; ++i)
				sumBuffer[i] = 0;
		}
		
		__syncthreads();
		
		{
			TreeNode node1 = tree_object1[work_pair.x];
			TreeNode node2 = tree_object2[work_pair.y];
			transform_object1.apply(node1.bbox);
			transform_object2.apply(node2.bbox);
			terminated = !intersect<BV>(node1.bbox, node2.bbox);
			if(__all(terminated)) return 0;
		}
		
		
		while(1)
		{
			TreeNode node1 = tree_object1[work_pair.x];
			TreeNode node2 = tree_object2[work_pair.y];
			
			if(!node1.isLeaf() && !node2.isLeaf())
			{
				int left1 = node1.getLeftChild() + work_pair.x;
				int right1 = left1 + 1;
				int left2 = node2.getLeftChild() + work_pair.y;
				int right2 = left2 + 1;
				
				int sum0 = 0, sum1 = 0, sum2 = 0, sum3 = 0;
				int maxSum, traverseDir;
				
				TreeNode node1c, node2c;
				bool overlapRes;
				
				node1c = tree_object1[left1];
				node2c = tree_object2[left2];
				transform_object1.apply(node1c.bbox);
				transform_object2.apply(node2c.bbox);
				overlapRes = !terminated && intersect<BV>(node1c.bbox, node2c.bbox);
				if(__any(overlapRes))
				{
					sumBuffer[tidx] = overlapRes;
					reduceSum_(sumBuffer, tidx);
					sum0 = sumBuffer[0];
				}
				maxSum = sum0;
				traverseDir = 0;
				
				
				node1c = tree_object1[left1];
				node2c = tree_object2[right2];
				transform_object1.apply(node1c.bbox);
				transform_object2.apply(node2c.bbox);
				overlapRes = !terminated && intersect<BV>(node1c.bbox, node2c.bbox);
				if(__any(overlapRes))
				{
					sumBuffer[tidx] = overlapRes ? 1 : 0;
					reduceSum_(sumBuffer, tidx);
					sum1 = sumBuffer[0];
				}
				if(maxSum < sum1)
				{
					maxSum = sum1;
					traverseDir = 1;
				}
				
				node1c = tree_object1[right1];
				node2c = tree_object2[left2];
				transform_object1.apply(node1c.bbox);
				transform_object2.apply(node2c.bbox);
				overlapRes = !terminated && intersect<BV>(node1c.bbox, node2c.bbox);
				if(__any(overlapRes))
				{
					sumBuffer[tidx] = overlapRes ? 1 : 0;
					reduceSum_(sumBuffer, tidx);
					sum2 = sumBuffer[0];
				}
				if(maxSum < sum2)
				{
					maxSum = sum2;
					traverseDir = 2;
				}
				
				node1c = tree_object1[right1];
				node2c = tree_object2[right2];
				transform_object1.apply(node1c.bbox);
				transform_object2.apply(node2c.bbox);
				overlapRes = !terminated && intersect<BV>(node1c.bbox, node2c.bbox);
				if(__any(overlapRes))
				{
					sumBuffer[tidx] = overlapRes ? 1 : 0;
					reduceSum_(sumBuffer, tidx);
					sum3 = sumBuffer[0];
				}
				if(maxSum < sum3)
				{
					maxSum = sum3;
					traverseDir = 3;
				}
				
				
				if(maxSum == 0)
				{
					if(stackPtr < 0)
						break;
					else
					{
						work_pair.x = stack[stackPtr].x;
						work_pair.y = stack[stackPtr].y;
						if(tidx == 0)
							stackPtr--;
					}
				}
				else
				{
					if(traverseDir == 0)
					{
						work_pair.x = left1;
						work_pair.y = left2;
					}
					else if(sum0 > 0)
					{
						if(tidx == 0)
						{
							++stackPtr;
							if(stackPtr >= BVH_PACKET_STACK_SIZE) break;
							stack[stackPtr].x = left1;
							stack[stackPtr].y = left2;
						}
					}
					
					if(traverseDir == 1)
					{
						work_pair.x = left1;
						work_pair.y = right2;
					}
					else if(sum1 > 0)
					{
						if(tidx == 0)
						{
							++stackPtr;
							if(stackPtr >= BVH_PACKET_STACK_SIZE) break;
							stack[stackPtr].x = left1;
							stack[stackPtr].y = right2;
						}
					}
					
					if(traverseDir == 2)
					{
						work_pair.x = right1;
						work_pair.y = left2;
					}
					else if(sum2 > 0)
					{
						if(tidx == 0)
						{
							++stackPtr;
							if(stackPtr >= BVH_PACKET_STACK_SIZE) break;
							stack[stackPtr].x = right1;
							stack[stackPtr].y = left2;
						}
					}
					
					if(traverseDir == 3)
					{
						work_pair.x = right1;
						work_pair.y = right2;
					}
					else if(sum3 > 0)
					{
						if(tidx == 0)
						{
							++stackPtr;
							if(stackPtr >= BVH_PACKET_STACK_SIZE) break;
							stack[stackPtr].x = right1;
							stack[stackPtr].y = right2;
						}
					}
				}
			}
			else if(node1.isLeaf() && node2.isLeaf())    // both leaf
			{
				uint3 idx = tri_object1[node1.getTriID()];
				uint3 idx2 = tri_object2[node2.getTriID()];
				
				// transformed robot vertices
				float3 l1 = transform_object1.applyv(vertex_object1[idx.x].v);
				float3 l2 = transform_object1.applyv(vertex_object1[idx.y].v);
				float3 l3 = transform_object1.applyv(vertex_object1[idx.z].v);
				
				float3 r1 = transform_object2.applyv(vertex_object2[idx2.x].v);
				float3 r2 = transform_object2.applyv(vertex_object2[idx2.y].v);
				float3 r3 = transform_object2.applyv(vertex_object2[idx2.z].v);
				
				// intersect triangles:
				terminated = (terminated || triangleIntersection(l1, l2, l3, r1, r2, r3));
				if(__all(terminated)) break;
				
				if(stackPtr < 0)
					break;
				else
				{
					work_pair.x = stack[stackPtr].x;
					work_pair.y = stack[stackPtr].y;
					if(tidx == 0)
						stackPtr--;
				}
			}
			else if(node1.isLeaf())   	// left node is leaf
			{
				int left2 = node2.getLeftChild() + work_pair.y;
				int right2 = left2 + 1;
				
				TreeNode node1c, node2c;
				bool overlapRes0, overlapRes1;
				
				node1c = tree_object1[work_pair.x];
				node2c = tree_object2[left2];
				transform_object1.apply(node1c.bbox);
				transform_object2.apply(node2c.bbox);
				overlapRes0 = !terminated && intersect<BV>(node1c.bbox, node2c.bbox);
				
				node1c = tree_object1[work_pair.x];
				node2c = tree_object2[right2];
				transform_object1.apply(node1c.bbox);
				transform_object2.apply(node2c.bbox);
				overlapRes1 = !terminated && intersect<BV>(node1c.bbox, node2c.bbox);
				
				bool anyc0 = __any(overlapRes0);
				bool anyc1 = __any(overlapRes1);
				
				if(anyc0 && anyc1)
				{
					sumBuffer[tidx] = 0;
					if(overlapRes0 && !overlapRes1) sumBuffer[tidx] = 1;
					if(!overlapRes0 && overlapRes1) sumBuffer[tidx] = -1;
					reduceSum_(sumBuffer, tidx);
					
					if(sumBuffer[0] >= 0)
					{
						work_pair.y = left2;
						if(tidx == 0)
						{
							++stackPtr;
							if(stackPtr >= BVH_PACKET_STACK_SIZE) break;
							stack[stackPtr].x = work_pair.x;
							stack[stackPtr].y = right2;
						}
					}
					else
					{
						work_pair.y = right2;
						if(tidx == 0)
						{
							++stackPtr;
							if(stackPtr >= BVH_PACKET_STACK_SIZE) break;
							stack[stackPtr].x = work_pair.x;
							stack[stackPtr].y = left2;
						}
					}
				}
				else if(anyc0)
				{
					work_pair.y = left2;
				}
				else if(anyc1)
				{
					work_pair.y = right2;
				}
				else
				{
					if(stackPtr < 0)
						break;
					else
					{
						work_pair.x = stack[stackPtr].x;
						work_pair.y = stack[stackPtr].y;
						if(tidx == 0)
							stackPtr--;
					}
				}
			}
			else
			{
				int left1 = node1.getLeftChild() + work_pair.x;
				int right1 = left1 + 1;
				
				TreeNode node1c, node2c;
				bool overlapRes0, overlapRes1;
				
				node1c = tree_object1[left1];
				node2c = tree_object2[work_pair.y];
				transform_object1.apply(node1c.bbox);
				transform_object2.apply(node2c.bbox);
				overlapRes0 = !terminated && intersect<BV>(node1c.bbox, node2c.bbox);
				
				node1c = tree_object1[right1];
				node2c = tree_object2[work_pair.y];
				transform_object1.apply(node1c.bbox);
				transform_object2.apply(node2c.bbox);
				overlapRes1 = !terminated && intersect<BV>(node1c.bbox, node2c.bbox);
				
				bool anyc0 = __any(overlapRes0);
				bool anyc1 = __any(overlapRes1);
				
				if(anyc0 && anyc1)
				{
					sumBuffer[tidx] = 0;
					if(overlapRes0 && !overlapRes1) sumBuffer[tidx] = 1;
					if(!overlapRes0 && overlapRes1) sumBuffer[tidx] = -1;
					reduceSum_(sumBuffer, tidx);
					
					if(sumBuffer[0] >= 0)
					{
						work_pair.x = left1;
						if(tidx == 0)
						{
							++stackPtr;
							if(stackPtr >= BVH_PACKET_STACK_SIZE) break;
							stack[stackPtr].x = right1;
							stack[stackPtr].y = work_pair.y;
						}
					}
					else
					{
						work_pair.x = right1;
						if(tidx == 0)
						{
							++stackPtr;
							if(stackPtr >= BVH_PACKET_STACK_SIZE) break;
							stack[stackPtr].x = left1;
							stack[stackPtr].y = work_pair.y;
						}
					}
				}
				else if(anyc0)
				{
					work_pair.x = left1;
				}
				else if(anyc1)
				{
					work_pair.x = right1;
				}
				else
				{
					if(stackPtr < 0)
						break;
					else
					{
						work_pair.x = stack[stackPtr].x;
						work_pair.y = stack[stackPtr].y;
						if(tidx == 0)
							stackPtr--;
					}
				}
			}
		}
		
		return (terminated) ? 1 : 0;
	}
};

template <class TreeNode, class BV, class stackElem, int StackFullDefault>
class BVHCollidePacketFunctor
{
public:
	__device__ int operator()(const TreeNode* tree_object1, const GPUVertex* vertex_object1, const uint3* tri_object1, GTransform& transform_object1,
	                          const TreeNode* tree_object2, const GPUVertex* vertex_object2, const uint3* tri_object2, GTransform& transform_object2) const
	{
		__shared__ volatile stackElem traversalStacks[BVH_PACKET_STACK_SIZE * BVH_PACKET_BLOCK_HEIGHT];
		__shared__ volatile int stackPtrs[BVH_PACKET_BLOCK_HEIGHT];
		__shared__ volatile int sumBuffers[BVH_PACKET_OVERLAP_SUM_SIZE * BVH_PACKET_BLOCK_HEIGHT];
		
		volatile stackElem* stack = traversalStacks + BVH_PACKET_STACK_SIZE * threadIdx.y;
		volatile int* sumBuffer = sumBuffers + BVH_PACKET_OVERLAP_SUM_SIZE * threadIdx.y;
		volatile int& stackPtr = stackPtrs[threadIdx.y];
		
		
		int tidx = threadIdx.x;
		//int widx = threadIdx.y;
		stackElem work_pair;
		bool terminated = false;
		stackPtr = -1;
		work_pair.x = 0;
		work_pair.y = 0;
		
		if(tidx == 0)
		{
			for(int i = 0; i < BVH_PACKET_OVERLAP_SUM_SIZE; ++i)
				sumBuffer[i] = 0;
		}
		
		__syncthreads();
		
		{
			TreeNode node1 = tree_object1[work_pair.x];
			TreeNode node2 = tree_object2[work_pair.y];
			transform_object1.apply(node1.bbox);
			transform_object2.apply(node2.bbox);
			terminated = !intersect<BV>(node1.bbox, node2.bbox);
			if(__all(terminated)) return 0;
		}
		
		
		while(1)
		{
			TreeNode node1 = tree_object1[work_pair.x];
			TreeNode node2 = tree_object2[work_pair.y];
			
			
			if(node1.isLeaf() && node2.isLeaf())    // both leaf
			{
				uint3 idx = tri_object1[node1.getTriID()];
				uint3 idx2 = tri_object2[node2.getTriID()];
				
				// transformed robot vertices
				float3 l1 = transform_object1.applyv(vertex_object1[idx.x].v);
				float3 l2 = transform_object1.applyv(vertex_object1[idx.y].v);
				float3 l3 = transform_object1.applyv(vertex_object1[idx.z].v);
				
				float3 r1 = transform_object2.applyv(vertex_object2[idx2.x].v);
				float3 r2 = transform_object2.applyv(vertex_object2[idx2.y].v);
				float3 r3 = transform_object2.applyv(vertex_object2[idx2.z].v);
				
				// intersect triangles:
				terminated = (terminated || triangleIntersection(l1, l2, l3, r1, r2, r3));
				if(__all(terminated)) break;
				
				if(stackPtr < 0)
					break;
				else
				{
					work_pair.x = stack[stackPtr].x;
					work_pair.y = stack[stackPtr].y;
					if(tidx == 0)
						stackPtr--;
				}
			}
			else if(node2.isLeaf() || (!node1.isLeaf() && node1.bbox.getSize() > node2.bbox.getSize()))
			{
				int left1 = node1.getLeftChild() + work_pair.x;
				int right1 = left1 + 1;
				
				TreeNode node1c, node2c;
				bool overlapRes0, overlapRes1;
				
				node1c = tree_object1[left1];
				node2c = tree_object2[work_pair.y];
				transform_object1.apply(node1c.bbox);
				transform_object2.apply(node2c.bbox);
				overlapRes0 = !terminated && intersect<BV>(node1c.bbox, node2c.bbox);
				
				node1c = tree_object1[right1];
				node2c = tree_object2[work_pair.y];
				transform_object1.apply(node1c.bbox);
				transform_object2.apply(node2c.bbox);
				overlapRes1 = !terminated && intersect<BV>(node1c.bbox, node2c.bbox);
				
				bool anyc0 = __any(overlapRes0);
				bool anyc1 = __any(overlapRes1);
				
				if(anyc0 && anyc1)
				{
					sumBuffer[tidx] = 0;
					if(overlapRes0 && !overlapRes1) sumBuffer[tidx] = 1;
					if(!overlapRes0 && overlapRes1) sumBuffer[tidx] = -1;
					reduceSum_(sumBuffer, tidx);
					
					if(sumBuffer[0] >= 0)
					{
						work_pair.x = left1;
						if(tidx == 0)
						{
							++stackPtr;
							if(stackPtr >= BVH_PACKET_STACK_SIZE) break;
							stack[stackPtr].x = right1;
							stack[stackPtr].y = work_pair.y;
						}
					}
					else
					{
						work_pair.x = right1;
						if(tidx == 0)
						{
							++stackPtr;
							if(stackPtr >= BVH_PACKET_STACK_SIZE) break;
							stack[stackPtr].x = left1;
							stack[stackPtr].y = work_pair.y;
						}
					}
				}
				else if(anyc0)
				{
					work_pair.x = left1;
				}
				else if(anyc1)
				{
					work_pair.x = right1;
				}
				else
				{
					if(stackPtr < 0)
						break;
					else
					{
						work_pair.x = stack[stackPtr].x;
						work_pair.y = stack[stackPtr].y;
						if(tidx == 0)
							stackPtr--;
					}
				}
			}
			else
			{
				int left2 = node2.getLeftChild() + work_pair.y;
				int right2 = left2 + 1;
				
				TreeNode node1c, node2c;
				bool overlapRes0, overlapRes1;
				
				node1c = tree_object1[work_pair.x];
				node2c = tree_object2[left2];
				transform_object1.apply(node1c.bbox);
				transform_object2.apply(node2c.bbox);
				overlapRes0 = !terminated && intersect<BV>(node1c.bbox, node2c.bbox);
				
				node1c = tree_object1[work_pair.x];
				node2c = tree_object2[right2];
				transform_object1.apply(node1c.bbox);
				transform_object2.apply(node2c.bbox);
				overlapRes1 = !terminated && intersect<BV>(node1c.bbox, node2c.bbox);
				
				bool anyc0 = __any(overlapRes0);
				bool anyc1 = __any(overlapRes1);
				
				if(anyc0 && anyc1)
				{
					sumBuffer[tidx] = 0;
					if(overlapRes0 && !overlapRes1) sumBuffer[tidx] = 1;
					if(!overlapRes0 && overlapRes1) sumBuffer[tidx] = -1;
					reduceSum_(sumBuffer, tidx);
					
					if(sumBuffer[0] >= 0)
					{
						work_pair.y = left2;
						if(tidx == 0)
						{
							++stackPtr;
							if(stackPtr >= BVH_PACKET_STACK_SIZE) break;
							stack[stackPtr].x = work_pair.x;
							stack[stackPtr].y = right2;
						}
					}
					else
					{
						work_pair.y = right2;
						if(tidx == 0)
						{
							++stackPtr;
							if(stackPtr >= BVH_PACKET_STACK_SIZE) break;
							stack[stackPtr].x = work_pair.x;
							stack[stackPtr].y = left2;
						}
					}
				}
				else if(anyc0)
				{
					work_pair.y = left2;
				}
				else if(anyc1)
				{
					work_pair.y = right2;
				}
				else
				{
					if(stackPtr < 0)
						break;
					else
					{
						work_pair.x = stack[stackPtr].x;
						work_pair.y = stack[stackPtr].y;
						if(tidx == 0)
							stackPtr--;
					}
				}
			}
			
		}
		
		return (terminated) ? 1 : 0;
	}
};

template <class TreeNode, class BV, class stackElem, int StackFullDefault>
class BVHCollideFunctor_old
{
public:
	__device__ int operator()(const TreeNode* tree_object1, const GPUVertex* vertex_object1, const uint3* tri_object1, GTransform& transform_object1,
	                          const TreeNode* tree_object2, const GPUVertex* vertex_object2, const uint3* tri_object2, GTransform& transform_object2) const
	{
		__shared__ stackElem traversalStacks[COLLISION_THREADS * COLLISION_STACK_SIZE];
		
		int stackPtr = threadIdx.x;
		stackElem work_pair;
		work_pair.x = 0;
		work_pair.y = 0;
		
		while(1)
		{
			TreeNode node1 = tree_object1[work_pair.x];
			TreeNode node2 = tree_object2[work_pair.y];
			
			transform_object1.apply(node1.bbox);
			transform_object2.apply(node2.bbox);
			
			if(intersect<BV>(node1.bbox, node2.bbox))
			{
				// 3 cases:
				//  - both are inner nodes: create 4 new intersections with children
				//  - one is leaf, one is inner: intersect leaf with children of other
				//  - both are leafs: write triangle-triangle pair to list
				if(!node1.isLeaf() && !node2.isLeaf())    // both inner
				{
					int left1 = node1.getLeftChild() + work_pair.x;
					int right1 = left1 + 1;
					int left2 = node2.getLeftChild() + work_pair.y;
					int right2 = left2 + 1;
					
					if((stackPtr + 3 * COLLISION_THREADS) >= COLLISION_THREADS * COLLISION_STACK_SIZE)
					{
						return StackFullDefault;
					}
					
					// test all 4 combination of children
					work_pair.x = left1;
					work_pair.y = left2;
					traversalStacks[stackPtr].x = left1;
					traversalStacks[stackPtr].y = right2;
					stackPtr += COLLISION_THREADS;
					traversalStacks[stackPtr].x = right1;
					traversalStacks[stackPtr].y = left2;
					stackPtr += COLLISION_THREADS;
					traversalStacks[stackPtr].x = right1;
					traversalStacks[stackPtr].y = right2;
					stackPtr += COLLISION_THREADS;
					
					continue;
				}
				else if(node1.isLeaf() && node2.isLeaf())    // both leaf
				{
					uint3 idx = tri_object1[node1.getTriID()];
					uint3 idx2 = tri_object2[node2.getTriID()];
					
					// transformed robot vertices
					float3 l1 = transform_object1.applyv(vertex_object1[idx.x].v);
					float3 l2 = transform_object1.applyv(vertex_object1[idx.y].v);
					float3 l3 = transform_object1.applyv(vertex_object1[idx.z].v);
					
					float3 r1 = transform_object2.applyv(vertex_object2[idx2.x].v);
					float3 r2 = transform_object2.applyv(vertex_object2[idx2.y].v);
					float3 r3 = transform_object2.applyv(vertex_object2[idx2.z].v);
					
					// intersect triangles:
					if(triangleIntersection(l1, l2, l3, r1, r2, r3))
					{
						// intersection found: write collision result and abort
						return 1;
					}
				}
				else if(node1.isLeaf())   	// left node is leaf
				{
					work_pair.y = node2.getLeftChild() + work_pair.y;
					
					if(stackPtr >= COLLISION_THREADS * COLLISION_STACK_SIZE)
					{
						return StackFullDefault;
					}
					traversalStacks[stackPtr].x = work_pair.x;
					traversalStacks[stackPtr].y = work_pair.y + 1;
					stackPtr += COLLISION_THREADS;
					
					continue;
				}
				else   // right node is leaf
				{
					work_pair.x = node1.getLeftChild() + work_pair.x;
					
					if(stackPtr >= COLLISION_THREADS * COLLISION_STACK_SIZE)
					{
						return StackFullDefault;
					}
					
					traversalStacks[stackPtr].x = work_pair.x + 1;
					traversalStacks[stackPtr].y = work_pair.y;
					stackPtr += COLLISION_THREADS;
					
					continue;
				}
			}
			
			stackPtr -= COLLISION_THREADS;
			
			// stack empty? then no intersection
			if(stackPtr < (int)threadIdx.x)
				break;
			work_pair.x = traversalStacks[stackPtr].x;
			work_pair.y = traversalStacks[stackPtr].y;
		}
		
		return 0;
	}
};


template <class TreeNode, class BV, class stackElem, int StackFullDefault>
class BVHCollideFunctor
{
public:
	__device__ int operator()(const TreeNode* tree_object1, const GPUVertex* vertex_object1, const uint3* tri_object1, GTransform& transform_object1,
	                          const TreeNode* tree_object2, const GPUVertex* vertex_object2, const uint3* tri_object2, GTransform& transform_object2) const
	{
		__shared__ stackElem traversalStacks[COLLISION_THREADS * COLLISION_STACK_SIZE];
		
		int stackPtr = threadIdx.x;
		stackElem work_pair;
		work_pair.x = 0;
		work_pair.y = 0;
		
		while(1)
		{
			TreeNode node1 = tree_object1[work_pair.x];
			TreeNode node2 = tree_object2[work_pair.y];
			
			transform_object1.apply(node1.bbox);
			transform_object2.apply(node2.bbox);
			
			if(intersect<BV>(node1.bbox, node2.bbox))
			{
				// 3 cases:
				//  - both are inner nodes: create 4 new intersections with children
				//  - one is leaf, one is inner: intersect leaf with children of other
				//  - both are leafs: write triangle-triangle pair to list
				if(node1.isLeaf() && node2.isLeaf())    // both leaf
				{
					uint3 idx = tri_object1[node1.getTriID()];
					uint3 idx2 = tri_object2[node2.getTriID()];
					
					// transformed robot vertices
					float3 l1 = transform_object1.applyv(vertex_object1[idx.x].v);
					float3 l2 = transform_object1.applyv(vertex_object1[idx.y].v);
					float3 l3 = transform_object1.applyv(vertex_object1[idx.z].v);
					
					float3 r1 = transform_object2.applyv(vertex_object2[idx2.x].v);
					float3 r2 = transform_object2.applyv(vertex_object2[idx2.y].v);
					float3 r3 = transform_object2.applyv(vertex_object2[idx2.z].v);
					
					// intersect triangles:
					if(triangleIntersection(l1, l2, l3, r1, r2, r3))
					{
						// intersection found: write collision result and abort
						return 1;
					}
				}
				else if(node2.isLeaf() || (!node1.isLeaf() && (node1.bbox.getSize() > node2.bbox.getSize())))
				{
					work_pair.x = node1.getLeftChild() + work_pair.x;
					
					if(stackPtr >= COLLISION_THREADS * COLLISION_STACK_SIZE)
					{
						return StackFullDefault;
					}
					
					traversalStacks[stackPtr].x = work_pair.x + 1;
					traversalStacks[stackPtr].y = work_pair.y;
					stackPtr += COLLISION_THREADS;
					
					continue;
				}
				else
				{
					work_pair.y = node2.getLeftChild() + work_pair.y;
					
					if(stackPtr >= COLLISION_THREADS * COLLISION_STACK_SIZE)
					{
						return StackFullDefault;
					}
					traversalStacks[stackPtr].x = work_pair.x;
					traversalStacks[stackPtr].y = work_pair.y + 1;
					stackPtr += COLLISION_THREADS;
					
					continue;
				}
			}
			
			stackPtr -= COLLISION_THREADS;
			
			// stack empty? then no intersection
			if(stackPtr < (int)threadIdx.x)
				break;
			work_pair.x = traversalStacks[stackPtr].x;
			work_pair.y = traversalStacks[stackPtr].y;
		}
		
		return 0;
	}
};



// balance phase 1 (old, not include pump):
// (1) check whether need to check balance
// (2) compute the total task left
// (3) If need balance
//     compute the new task number for each queue in next iteration
template <int nThreads, int nQueues>
__global__ void balanceWorkList_phase1(unsigned int *workQueueCountIn, unsigned int* workQueueCountOut, unsigned int *totalEntries, int* ifBalanceNeeded)
{
	__shared__ int shared_queuesizes[nQueues];
	__shared__ int shared_ifneedbalance;
	const int idx = threadIdx.x;
	
	int nSplitsLeft = nQueues, inputOffset = idx;
	
	if(idx == 0)
	{
		shared_ifneedbalance = 0;
	}
	__syncthreads();
	
	while(idx < nSplitsLeft)
	{
		int nQueueElements = workQueueCountIn[inputOffset];
		shared_queuesizes[inputOffset] = nQueueElements;
		if(nQueueElements < QUEUE_SIZE_PER_TASK_INIT && shared_ifneedbalance == 0) atomicExch(&shared_ifneedbalance, 1);
		else if(nQueueElements >=  QUEUE_SIZE_PER_TASK_GLOBAL - TRAVERSAL_THREADS * 4 - QUEUE_SIZE_PER_TASK && shared_ifneedbalance == 0) atomicExch(&shared_ifneedbalance, 1);
		inputOffset += nThreads;
		nSplitsLeft -= nThreads;
	}
	__syncthreads();
	
	REDUCE(shared_queuesizes, idx, nQueues, +);
	
	if(shared_ifneedbalance)
	{
		nSplitsLeft = shared_queuesizes[0];
		inputOffset = idx;
		int elementPerQueue, nWithPlus1;
		elementPerQueue = nSplitsLeft / nQueues;
		nWithPlus1 = nSplitsLeft - elementPerQueue * nQueues;
		
		nSplitsLeft = nQueues;
		while(idx < nSplitsLeft)
		{
			if(inputOffset < nWithPlus1)
				workQueueCountOut[inputOffset] = elementPerQueue + 1;
			else
				workQueueCountOut[inputOffset] = elementPerQueue;
			inputOffset += nThreads;
			nSplitsLeft -= nThreads;
		}
		__syncthreads();
	}
	
	if(idx == 0)
	{
		*totalEntries = shared_queuesizes[0];
		*ifBalanceNeeded = shared_ifneedbalance;
	}
	__syncthreads();
}

// balance phase 1:
// (1) check whether need to check balance
// (2) compute the total task left
// (3) If need balance
//     (i) check whether need to pump new queries. If 'YES', then left space by pumping
//     (ii) compute the new task number for each queue in next iteration (including pump)
template <int nThreads, int nQueues>
__global__ void balanceWorkList_phase1b(unsigned int *workQueueCountIn, unsigned int* workQueueCountOut, unsigned int *totalEntries, int* ifBalanceNeeded, unsigned int nSamplesToAdd, unsigned int thresholdForAdd)
{
	__shared__ int shared_queuesizes[nQueues];
	__shared__ int shared_ifneedbalance;
	const int idx = threadIdx.x;
	
	int nSplitsLeft = nQueues, inputOffset = idx;
	
	if(idx == 0)
	{
		shared_ifneedbalance = 0;
	}
	__syncthreads();
	
	while(idx < nSplitsLeft)
	{
		int nQueueElements = workQueueCountIn[inputOffset];
		shared_queuesizes[inputOffset] = nQueueElements;
		if(nQueueElements < QUEUE_SIZE_PER_TASK_INIT && shared_ifneedbalance == 0) atomicExch(&shared_ifneedbalance, 1);
		else if(nQueueElements >=  QUEUE_SIZE_PER_TASK_GLOBAL - TRAVERSAL_THREADS * 4 - QUEUE_SIZE_PER_TASK && shared_ifneedbalance == 0) atomicExch(&shared_ifneedbalance, 1);
		inputOffset += nThreads;
		nSplitsLeft -= nThreads;
	}
	__syncthreads();
	
	REDUCE(shared_queuesizes, idx, nQueues, +);
	
	if(shared_ifneedbalance)
	{
		nSplitsLeft = shared_queuesizes[0];
		
		if(nSplitsLeft < thresholdForAdd)
		{
			inputOffset = idx;
			int elementPerQueue, nWithPlus1;
			elementPerQueue = nSplitsLeft / nQueues;
			nWithPlus1 = nSplitsLeft - elementPerQueue * nQueues;
			int toAddElementPerQueue = nSamplesToAdd / nQueues;
			int nToAddWithPlus1 = nSamplesToAdd - toAddElementPerQueue * nQueues;
			
			nSplitsLeft = nQueues;
			while(idx < nSplitsLeft)
			{
				if(inputOffset < nWithPlus1)
					workQueueCountOut[inputOffset] = elementPerQueue + 1;
				else
					workQueueCountOut[inputOffset] = elementPerQueue;
				inputOffset += nThreads;
				nSplitsLeft -= nThreads;
			}
			
			nSplitsLeft = nQueues;
			inputOffset = idx;
			while(idx < nSplitsLeft)
			{
				if(inputOffset < nToAddWithPlus1)
					workQueueCountOut[inputOffset] += toAddElementPerQueue + 1;
				else
					workQueueCountOut[inputOffset] += toAddElementPerQueue;
				inputOffset += nThreads;
				nSplitsLeft -= nThreads;
			}
			__syncthreads();
		}
		else
		{
			inputOffset = idx;
			int elementPerQueue, nWithPlus1;
			elementPerQueue = nSplitsLeft / nQueues;
			nWithPlus1 = nSplitsLeft - elementPerQueue * nQueues;
			
			nSplitsLeft = nQueues;
			while(idx < nSplitsLeft)
			{
				if(inputOffset < nWithPlus1)
					workQueueCountOut[inputOffset] = elementPerQueue + 1;
				else
					workQueueCountOut[inputOffset] = elementPerQueue;
				inputOffset += nThreads;
				nSplitsLeft -= nThreads;
			}
			__syncthreads();
		}
	}
	
	if(idx == 0)
	{
		*totalEntries = shared_queuesizes[0];
		*ifBalanceNeeded = shared_ifneedbalance;
	}
	__syncthreads();
}

// balance phase 2
// rebalance work queue (copy from in to out), using all threads
// (no pumping)
template <int nQueues, class WorkElement>
__global__ void balanceWorkList_phase2(WorkElement* workQueueIn, WorkElement* workQueueOut, unsigned int* workQueueCountIn, const unsigned int maxQueueSize, unsigned int* totalEntries, int* bIfBalanceNeeded)
{
	int threadId = (blockIdx.y * gridDim.x + blockIdx.x) * blockDim.x + threadIdx.x;
	int nElements = *totalEntries;
	if(threadId >= nElements)
		return;
		
	int queueIn = 0, queueOut = 0, idIn, idOut;
	int curId = threadId;
	for(int i = 0; i < nQueues; ++i)
	{
		if(curId >= workQueueCountIn[i])
			curId -= workQueueCountIn[i];
		else
		{
			idIn = curId;
			queueIn = i;
			break;
		}
	}
	
	int elementPerQueue, nWithPlus1;
	elementPerQueue = nElements / nQueues;
	nWithPlus1 = nElements - elementPerQueue * nQueues;
	
	if(threadId < nWithPlus1 *(elementPerQueue + 1))
	{
		queueOut = threadId / (float)(elementPerQueue + 1);
		idOut = threadId - queueOut * (elementPerQueue + 1);
	}
	else
	{
		queueOut = nWithPlus1 + (threadId - nWithPlus1 * (elementPerQueue + 1)) / (float)elementPerQueue;
		idOut = threadId - nWithPlus1 - queueOut * elementPerQueue;
	}
	
	workQueueOut[queueOut * maxQueueSize + idOut] = workQueueIn[queueIn * maxQueueSize + idIn];
}


// balance phase 2
// rebalance work queue (copy from in to out), using all threads
// with pump
template <int nQueues, class WorkElement>
__global__ void balanceWorkList_phase2b(WorkElement* workQueueIn, WorkElement* workQueueOut, unsigned int* workQueueCountIn, const unsigned int maxQueueSize,
                                        unsigned int nSamplesToAdd, unsigned int* totalEntries, int* bIfBalanceNeeded)
{
	int threadId = (blockIdx.y * gridDim.x + blockIdx.x) * blockDim.x + threadIdx.x;
	int nElements = *totalEntries;
	if(threadId >= nElements)
		return;
		
	int nToAddPerQueue = nSamplesToAdd / nQueues;
	int nToAddWithPlus1 = nSamplesToAdd - nToAddPerQueue * nQueues;
	
	int queueIn = 0, queueOut = 0, idIn, idOut;
	int curId = threadId;
	for(int i = 0; i < nQueues; ++i)
	{
		if(curId >= workQueueCountIn[i])
			curId -= workQueueCountIn[i];
		else
		{
			idIn = curId;
			queueIn = i;
			break;
		}
	}
	
	int elementPerQueue, nWithPlus1;
	elementPerQueue = nElements / nQueues;
	nWithPlus1 = nElements - elementPerQueue * nQueues;
	
	if(threadId < nWithPlus1 *(elementPerQueue + 1))
	{
		queueOut = threadId / (float)(elementPerQueue + 1);
		idOut = threadId - queueOut * (elementPerQueue + 1);
	}
	else
	{
		queueOut = nWithPlus1 + (threadId - nWithPlus1 * (elementPerQueue + 1)) / (float)elementPerQueue;
		idOut = threadId - nWithPlus1 - queueOut * elementPerQueue;
	}
	
	if(queueOut < nToAddWithPlus1)
		workQueueOut[queueOut * maxQueueSize + nToAddPerQueue + 1 + idOut] = workQueueIn[queueIn * maxQueueSize + idIn];
	else
		workQueueOut[queueOut * maxQueueSize + nToAddPerQueue + idOut] = workQueueIn[queueIn * maxQueueSize + idIn];
}

// oldest balancing (gproximity's)
// 1. no pump
// 2. copy from inqueue to outqueue with only a few threads (large overhead)
template <int nThreads, int nQueues, class WorkElement>
__global__ void balanceWorkList(WorkElement *workQueueIn, WorkElement* workQueueOut, unsigned int* workQueueCount, const unsigned int maxQueueSize, unsigned int *totalEntries, int* balanceSignal)
{
	__shared__ int shared_sum[nThreads];
	__shared__ int shared_queuesizes[nQueues];
	__shared__ int shared_ifbalance;
	const int idx = threadIdx.x;
	
	// sum on workQueueCount to find total number of entries
	int nSplitsLeft = nQueues, inputOffset = idx;
	shared_sum[idx] = 0;
	
	if(idx == 0)
	{
		shared_ifbalance = 0;
	}
	__syncthreads();
	
	while(idx < nSplitsLeft)
	{
		int nQueueElements = workQueueCount[inputOffset];
		shared_queuesizes[inputOffset] = nQueueElements;
		if((nQueueElements < QUEUE_SIZE_PER_TASK_INIT) && (shared_ifbalance == 0)) atomicExch(&shared_ifbalance, 1);
		else if((nQueueElements >=  QUEUE_SIZE_PER_TASK_GLOBAL - TRAVERSAL_THREADS * 4 - QUEUE_SIZE_PER_TASK) && (shared_ifbalance == 0)) atomicExch(&shared_ifbalance, 1);
		shared_sum[idx] += nQueueElements;
		inputOffset += nThreads;
		nSplitsLeft -= nThreads;
	}
	__syncthreads();
	
	REDUCE(shared_sum, idx, nThreads, +);
	
	nSplitsLeft = shared_sum[0];
	
	if(idx == 0)
	{
		*totalEntries = nSplitsLeft;
		*balanceSignal = shared_ifbalance;
	}
	__syncthreads();
	
	if(shared_ifbalance > 0)
	{
		int nSplitsPerQueue, nWithPlus1;
		nSplitsPerQueue = nSplitsLeft / nQueues;
		nWithPlus1 = nSplitsLeft - nSplitsPerQueue * nQueues;
		
		inputOffset = 0;
		int outputOffset = 0, inputQueue = -1, inputQueueCount = 0;
		
		for(int q = 0; q < nQueues; ++q)
		{
			int nSplitsLocal;
			if(q < nWithPlus1)
				nSplitsLocal = nSplitsPerQueue + 1;
			else
				nSplitsLocal = nSplitsPerQueue;
				
			outputOffset = maxQueueSize * q + idx;
			
			if(idx == 0)
				workQueueCount[q] = nSplitsLocal;
				
			while(nSplitsLocal > 0)
			{
				if(inputQueueCount <= 0)
				{
					inputQueue++;
					inputOffset = idx;
					inputQueueCount = shared_queuesizes[inputQueue];
				}
				else
				{
					int splitsToWrite = min(nSplitsLocal, inputQueueCount);
					splitsToWrite = min(splitsToWrite, nThreads);
					
					if(idx < splitsToWrite)
						workQueueOut[outputOffset] = workQueueIn[inputQueue * maxQueueSize + inputOffset];
					nSplitsLocal -= splitsToWrite;
					inputOffset += splitsToWrite;
					outputOffset += splitsToWrite;
					inputQueueCount -= splitsToWrite;
					nSplitsLeft -= splitsToWrite;
				}
				__syncthreads();
			}
		}
	}
}


template <int nTotalProcessors>
static __device__ __inline__ void callAbort(unsigned int *workQueueCounter, const int threadID)
{
	if(threadID == 0)
		atomicInc(workQueueCounter, nTotalProcessors);
}


// nWorkQueueInitItems should be smaller than workQueueCapacity, e.g. 1/2 workQueueCapacity
// CollisionTag: set the tag for collision. (for milestone 0, for ccd -1)
template <class TreeNode, class BV, int workQueueCapacity, int nWorkQueueInitItems, int nThreads, int CollisionTag>
__global__ void traverseTree_old(const TreeNode* tree_object1, const GPUVertex* vertex_object1, const uint3* tri_object1, GTransform* transform_object1, unsigned int* transformIdx_object1, unsigned int transformCacheSize,
                                 const TreeNode* tree_object2, const GPUVertex* vertex_object2, const uint3* tri_object2, GTransform* transform_object2,
                                 int3* workQueues, unsigned int* workQueueCounts, unsigned int* idleCount, const unsigned int globalWorkQueueCapacity,
                                 unsigned int* collisionResults)
{
	__shared__ int workQueueIdx;
	__shared__ int globalWorkQueueIdx;
	__shared__ int3 localWorkQueue[workQueueCapacity];
	__shared__ unsigned int wantToAbort;
	
	const int blockOffset = gridDim.x * blockIdx.y + blockIdx.x;
	const int threadOffset = threadIdx.x;
	
	// read local counter
	if(threadOffset == 0)
	{
		globalWorkQueueIdx = workQueueCounts[blockOffset];
		workQueueIdx = min(nWorkQueueInitItems, globalWorkQueueIdx);
		globalWorkQueueIdx -= workQueueIdx;
	}
	
	__syncthreads();
	
	if(workQueueIdx == 0)
	{
		callAbort<workQueueCapacity>(idleCount, threadOffset);
		return;
	}
	
	
	{
		int3* globalQueue = &workQueues[blockOffset * globalWorkQueueCapacity + globalWorkQueueIdx];
		int queueOffset = threadOffset;
		while(queueOffset < workQueueIdx * 3)
		{
			((int*)localWorkQueue)[queueOffset] = ((int*)globalQueue)[queueOffset];
			queueOffset += nThreads;
		}
	}
	
	__syncthreads();
	
	while(workQueueIdx > 0)
	{
		int3 work_item;
		int nActive = min(nThreads, workQueueIdx);
		
		work_item.x = -1;
		if(threadOffset < workQueueIdx)
		{
			work_item = localWorkQueue[workQueueIdx - nActive + threadOffset];
			if(collisionResults[work_item.z] == CollisionTag) // already collision
				work_item.x = -1;
		}
		__syncthreads();
		
		if(threadOffset == 0)
		{
			workQueueIdx -= nActive;
		}
		__syncthreads();
		
		if(work_item.x >= 0)
		{
			TreeNode node1 = tree_object1[work_item.x];
			TreeNode node2 = tree_object2[work_item.y];
			GTransform trans1 = transform_object1[(int)fmodf(work_item.z, transformCacheSize)];
			GTransform trans2 = transform_object2[0];
			
			trans1.apply(node1.bbox);
			trans2.apply(node2.bbox);
			
			if(intersect<BV>(node1.bbox, node2.bbox))
			{
				if(!node1.isLeaf() && !node2.isLeaf())    // both inner
				{
					int left1 = node1.getLeftChild() + work_item.x;
					int right1 = left1 + 1;
					int left2 = node2.getLeftChild() + work_item.y;
					int right2 = left2 + 1;
					
					int localPtr = atomicAdd(&workQueueIdx, 4);
					
					if(localPtr < workQueueCapacity)
					{
						localWorkQueue[localPtr].x = left1;
						localWorkQueue[localPtr].y = left2;
						localWorkQueue[localPtr].z = work_item.z;
						localPtr++;
						
						if(localPtr < workQueueCapacity)
						{
							localWorkQueue[localPtr].x = left1;
							localWorkQueue[localPtr].y = right2;
							localWorkQueue[localPtr].z = work_item.z;
							localPtr++;
							
							if(localPtr < workQueueCapacity)
							{
								localWorkQueue[localPtr].x = right1;
								localWorkQueue[localPtr].y = left2;
								localWorkQueue[localPtr].z = work_item.z;
								localPtr++;
								
								if(localPtr < workQueueCapacity)
								{
									localWorkQueue[localPtr].x = right1;
									localWorkQueue[localPtr].y = right2;
									localWorkQueue[localPtr].z = work_item.z;
								}
								else
								{
									int globalPtr = atomicAdd(&globalWorkQueueIdx, 1);
									int3* globalQueue = &workQueues[blockOffset * globalWorkQueueCapacity + globalPtr];
									
									globalQueue[0].x = right1;
									globalQueue[0].y = right2;
									globalQueue[0].z = work_item.z;
								}
							}
							else
							{
								int globalPtr = atomicAdd(&globalWorkQueueIdx, 2);
								int3* globalQueue = &workQueues[blockOffset * globalWorkQueueCapacity + globalPtr];
								
								globalQueue[0].x = right1;
								globalQueue[0].y = left2;
								globalQueue[0].z = work_item.z;
								
								globalQueue[1].x = right1;
								globalQueue[1].y = right2;
								globalQueue[1].z = work_item.z;
							}
						}
						else
						{
							int globalPtr = atomicAdd(&globalWorkQueueIdx, 3);
							int3* globalQueue = &workQueues[blockOffset * globalWorkQueueCapacity + globalPtr];
							
							globalQueue[0].x = left1;
							globalQueue[0].y = right2;
							globalQueue[0].z = work_item.z;
							
							globalQueue[1].x = right1;
							globalQueue[1].y = left2;
							globalQueue[1].z = work_item.z;
							
							globalQueue[2].x = right1;
							globalQueue[2].y = right2;
							globalQueue[2].z = work_item.z;
						}
					}
					else
					{
						int globalPtr = atomicAdd(&globalWorkQueueIdx, 4);
						int3* globalQueue = &workQueues[blockOffset * globalWorkQueueCapacity + globalPtr];
						
						globalQueue[0].x = left1;
						globalQueue[0].y = left2;
						globalQueue[0].z = work_item.z;
						
						globalQueue[1].x = left1;
						globalQueue[1].y = right2;
						globalQueue[1].z = work_item.z;
						
						globalQueue[2].x = right1;
						globalQueue[2].y = left2;
						globalQueue[2].z = work_item.z;
						
						globalQueue[3].x = right1;
						globalQueue[3].y = right2;
						globalQueue[3].z = work_item.z;
					}
				}
				else if(node1.isLeaf() && node2.isLeaf())    // both leaf
				{
					uint3 idx = tri_object1[node1.getTriID()];
					uint3 idx2 = tri_object2[node2.getTriID()];
					
					// transformed robot vertices
					float3 l1 = trans1.applyv(vertex_object1[idx.x].v);
					float3 l2 = trans1.applyv(vertex_object1[idx.y].v);
					float3 l3 = trans1.applyv(vertex_object1[idx.z].v);
					
					float3 r1 = trans2.applyv(vertex_object2[idx2.x].v);
					float3 r2 = trans2.applyv(vertex_object2[idx2.y].v);
					float3 r3 = trans2.applyv(vertex_object2[idx2.z].v);
					
					// intersect triangles:
					if(triangleIntersection(l1, l2, l3, r1, r2, r3))
					{
						// intersection found: write collision result and abort
						unsigned int* result = &collisionResults[work_item.z];
						atomicExch(result, CollisionTag);
					}
				}
				else if(node1.isLeaf())   	// left node is leaf
				{
					int left2 = node2.getLeftChild() + work_item.y;
					int right2 = left2 + 1;
					
					int localPtr = atomicAdd(&workQueueIdx, 2);
					
					if(localPtr < workQueueCapacity)
					{
						localWorkQueue[localPtr].x = work_item.x;
						localWorkQueue[localPtr].y = left2;
						localWorkQueue[localPtr].z = work_item.z;
						localPtr++;
						
						if(localPtr < workQueueCapacity)
						{
							localWorkQueue[localPtr].x = work_item.x;
							localWorkQueue[localPtr].y = right2;
							localWorkQueue[localPtr].z = work_item.z;
						}
						else
						{
							int globalPtr = atomicAdd(&globalWorkQueueIdx, 1);
							int3* globalQueue = &workQueues[blockOffset * globalWorkQueueCapacity + globalPtr];
							
							globalQueue[0].x = work_item.x;
							globalQueue[0].y = right2;
							globalQueue[0].z = work_item.z;
						}
					}
					else
					{
						int globalPtr = atomicAdd(&globalWorkQueueIdx, 2);
						int3* globalQueue = &workQueues[blockOffset * globalWorkQueueCapacity + globalPtr];
						
						globalQueue[0].x = work_item.x;
						globalQueue[0].y = left2;
						globalQueue[0].z = work_item.z;
						
						globalQueue[1].x = work_item.x;
						globalQueue[1].y = right2;
						globalQueue[1].z = work_item.z;
					}
				}
				else   // right node is leaf
				{
					int left1 = node1.getLeftChild() + work_item.x;
					int right1 = left1 + 1;
					
					int localPtr = atomicAdd(&workQueueIdx, 2);
					
					if(localPtr < workQueueCapacity)
					{
						localWorkQueue[localPtr].x = left1;
						localWorkQueue[localPtr].y = work_item.y;
						localWorkQueue[localPtr].z = work_item.z;
						localPtr++;
						
						if(localPtr < workQueueCapacity)
						{
							localWorkQueue[localPtr].x = right1;
							localWorkQueue[localPtr].y = work_item.y;
							localWorkQueue[localPtr].z = work_item.z;
						}
						else
						{
							int globalPtr = atomicAdd(&globalWorkQueueIdx, 1);
							int3* globalQueue = &workQueues[blockOffset * globalWorkQueueCapacity + globalPtr];
							
							globalQueue[0].x = right1;
							globalQueue[0].y = work_item.y;
							globalQueue[0].z = work_item.z;
						}
					}
					else
					{
						int globalPtr = atomicAdd(&globalWorkQueueIdx, 2);
						int3* globalQueue = &workQueues[blockOffset * globalWorkQueueCapacity + globalPtr];
						
						globalQueue[0].x = left1;
						globalQueue[0].y = work_item.y;
						globalQueue[0].z = work_item.z;
						
						globalQueue[1].x = right1;
						globalQueue[1].y = work_item.y;
						globalQueue[1].z = work_item.z;
					}
				}
			}
		}
		
		__syncthreads();
		
		if((workQueueIdx >= workQueueCapacity - 3 * nThreads) || (globalWorkQueueIdx >= QUEUE_SIZE_PER_TASK_GLOBAL - nThreads * 4 - workQueueCapacity) || (workQueueIdx == 0))
		{
			callAbort<workQueueCapacity>(idleCount, threadOffset);
			break;
		}
		
		if(threadOffset == 0)
		{
			wantToAbort = *idleCount;
		}
		
		__syncthreads();
		
		if(wantToAbort > QUEUE_IDLETASKS_FOR_ABORT)
		{
			callAbort<workQueueCapacity>(idleCount, threadOffset);
			break;
		}
	}
	
	if(threadOffset == 0)
	{
		workQueueIdx = min(workQueueIdx, workQueueCapacity);
		workQueueCounts[blockOffset] = workQueueIdx + globalWorkQueueIdx;
	}
	__syncthreads();
	
	{
		int queueOffset = threadOffset;
		int3* globalQueue = &workQueues[blockOffset * globalWorkQueueCapacity + globalWorkQueueIdx];
		while(queueOffset < workQueueIdx * 3)
		{
			((int*)globalQueue)[queueOffset] = ((int*)localWorkQueue)[queueOffset];
			queueOffset += nThreads;
		}
	}
	__syncthreads();
}

// nWorkQueueInitItems should be smaller than workQueueCapacity, e.g. 1/2 workQueueCapacity
// CollisionTag: set the tag for collision. (for milestone 0, for ccd -1)
template <class TreeNode, class BV, int workQueueCapacity, int nWorkQueueInitItems, int nThreads, int CollisionTag>
__global__ void traverseTree(const TreeNode* tree_object1, const GPUVertex* vertex_object1, const uint3* tri_object1, GTransform* transform_object1, unsigned int* transformIdx_object1, unsigned int transformCacheSize,
                             const TreeNode* tree_object2, const GPUVertex* vertex_object2, const uint3* tri_object2, GTransform* transform_object2,
                             int3* workQueues, unsigned int* workQueueCounts, unsigned int* idleCount, const unsigned int globalWorkQueueCapacity,
                             unsigned int* collisionResults)
{
	__shared__ int workQueueIdx;
	__shared__ int globalWorkQueueIdx;
	__shared__ int3 localWorkQueue[workQueueCapacity];
	__shared__ unsigned int wantToAbort;
	
	const int blockOffset = gridDim.x * blockIdx.y + blockIdx.x;
	const int threadOffset = threadIdx.x;
	
	// read local counter
	if(threadOffset == 0)
	{
		globalWorkQueueIdx = workQueueCounts[blockOffset];
		workQueueIdx = min(nWorkQueueInitItems, globalWorkQueueIdx);
		globalWorkQueueIdx -= workQueueIdx;
	}
	
	__syncthreads();
	
	if(workQueueIdx == 0)
	{
		callAbort<workQueueCapacity>(idleCount, threadOffset);
		return;
	}
	
	
	{
		int3* globalQueue = &workQueues[blockOffset * globalWorkQueueCapacity + globalWorkQueueIdx];
		int queueOffset = threadOffset;
		while(queueOffset < workQueueIdx * 3)
		{
			((int*)localWorkQueue)[queueOffset] = ((int*)globalQueue)[queueOffset];
			queueOffset += nThreads;
		}
	}
	
	__syncthreads();
	
	while(workQueueIdx > 0)
	{
		int3 work_item;
		int nActive = min(nThreads, workQueueIdx);
		
		work_item.x = -1;
		if(threadOffset < workQueueIdx)
		{
			work_item = localWorkQueue[workQueueIdx - nActive + threadOffset];
			if(collisionResults[work_item.z] == CollisionTag) // already collision
				work_item.x = -1;
		}
		__syncthreads();
		
		if(threadOffset == 0)
		{
			workQueueIdx -= nActive;
		}
		__syncthreads();
		
		if(work_item.x >= 0)
		{
			TreeNode node1 = tree_object1[work_item.x];
			TreeNode node2 = tree_object2[work_item.y];
			GTransform trans1 = transform_object1[(int)(fmodf(work_item.z, transformCacheSize))];
			GTransform trans2 = transform_object2[0];
			
			trans1.apply(node1.bbox);
			trans2.apply(node2.bbox);
			
			if(intersect<BV>(node1.bbox, node2.bbox))
			{
				if(node1.isLeaf() && node2.isLeaf())    // both leaf
				{
					uint3 idx = tri_object1[node1.getTriID()];
					uint3 idx2 = tri_object2[node2.getTriID()];
					
					// transformed robot vertices
					float3 l1 = trans1.applyv(vertex_object1[idx.x].v);
					float3 l2 = trans1.applyv(vertex_object1[idx.y].v);
					float3 l3 = trans1.applyv(vertex_object1[idx.z].v);
					
					float3 r1 = trans2.applyv(vertex_object2[idx2.x].v);
					float3 r2 = trans2.applyv(vertex_object2[idx2.y].v);
					float3 r3 = trans2.applyv(vertex_object2[idx2.z].v);
					
					// intersect triangles:
					if(triangleIntersection(l1, l2, l3, r1, r2, r3))
					{
						// intersection found: write collision result and abort
						unsigned int* result = &collisionResults[work_item.z];
						atomicExch(result, CollisionTag);
					}
				}
				else if(node2.isLeaf() || (!node1.isLeaf() && (node1.bbox.getSize() > node2.bbox.getSize())))
				{
					int left1 = node1.getLeftChild() + work_item.x;
					int right1 = left1 + 1;
					
					int localPtr = atomicAdd(&workQueueIdx, 2);
					
					if(localPtr < workQueueCapacity)
					{
						localWorkQueue[localPtr].x = left1;
						localWorkQueue[localPtr].y = work_item.y;
						localWorkQueue[localPtr].z = work_item.z;
						localPtr++;
						
						if(localPtr < workQueueCapacity)
						{
							localWorkQueue[localPtr].x = right1;
							localWorkQueue[localPtr].y = work_item.y;
							localWorkQueue[localPtr].z = work_item.z;
						}
						else
						{
							int globalPtr = atomicAdd(&globalWorkQueueIdx, 1);
							int3* globalQueue = &workQueues[blockOffset * globalWorkQueueCapacity + globalPtr];
							
							globalQueue[0].x = right1;
							globalQueue[0].y = work_item.y;
							globalQueue[0].z = work_item.z;
						}
					}
					else
					{
						int globalPtr = atomicAdd(&globalWorkQueueIdx, 2);
						int3* globalQueue = &workQueues[blockOffset * globalWorkQueueCapacity + globalPtr];
						
						globalQueue[0].x = left1;
						globalQueue[0].y = work_item.y;
						globalQueue[0].z = work_item.z;
						
						globalQueue[1].x = right1;
						globalQueue[1].y = work_item.y;
						globalQueue[1].z = work_item.z;
					}
				}
				else
				{
					int left2 = node2.getLeftChild() + work_item.y;
					int right2 = left2 + 1;
					
					int localPtr = atomicAdd(&workQueueIdx, 2);
					
					if(localPtr < workQueueCapacity)
					{
						localWorkQueue[localPtr].x = work_item.x;
						localWorkQueue[localPtr].y = left2;
						localWorkQueue[localPtr].z = work_item.z;
						localPtr++;
						
						if(localPtr < workQueueCapacity)
						{
							localWorkQueue[localPtr].x = work_item.x;
							localWorkQueue[localPtr].y = right2;
							localWorkQueue[localPtr].z = work_item.z;
						}
						else
						{
							int globalPtr = atomicAdd(&globalWorkQueueIdx, 1);
							int3* globalQueue = &workQueues[blockOffset * globalWorkQueueCapacity + globalPtr];
							
							globalQueue[0].x = work_item.x;
							globalQueue[0].y = right2;
							globalQueue[0].z = work_item.z;
						}
					}
					else
					{
						int globalPtr = atomicAdd(&globalWorkQueueIdx, 2);
						int3* globalQueue = &workQueues[blockOffset * globalWorkQueueCapacity + globalPtr];
						
						globalQueue[0].x = work_item.x;
						globalQueue[0].y = left2;
						globalQueue[0].z = work_item.z;
						
						globalQueue[1].x = work_item.x;
						globalQueue[1].y = right2;
						globalQueue[1].z = work_item.z;
					}
				}
			}
		}
		
		__syncthreads();
		
		if((workQueueIdx >= workQueueCapacity - nThreads) || (globalWorkQueueIdx >= QUEUE_SIZE_PER_TASK_GLOBAL - nThreads * 2 - workQueueCapacity) || (workQueueIdx == 0))
		{
			callAbort<workQueueCapacity>(idleCount, threadOffset);
			break;
		}
		
		if(threadOffset == 0)
		{
			wantToAbort = *idleCount;
		}
		
		__syncthreads();
		
		if(wantToAbort > QUEUE_IDLETASKS_FOR_ABORT)
		{
			callAbort<workQueueCapacity>(idleCount, threadOffset);
			break;
		}
	}
	
	if(threadOffset == 0)
	{
		workQueueIdx = min(workQueueIdx, workQueueCapacity);
		workQueueCounts[blockOffset] = workQueueIdx + globalWorkQueueIdx;
	}
	__syncthreads();
	
	{
		int queueOffset = threadOffset;
		int3* globalQueue = &workQueues[blockOffset * globalWorkQueueCapacity + globalWorkQueueIdx];
		while(queueOffset < workQueueIdx * 3)
		{
			((int*)globalQueue)[queueOffset] = ((int*)localWorkQueue)[queueOffset];
			queueOffset += nThreads;
		}
	}
	__syncthreads();
}

/** Functor to generate max number of interpolation number for incremental mode
 */
class IncrementalMaxIterFunctor
{
public:
	/** Generator
	 * @param dist Distance between two configurations
	 * @param resolution Resolution for CCD checking
	 * @return Max number of interpolation number (the number of max partition - 1)
	 */
	__device__ unsigned int operator()(const float dist, const float resolution) const
	{
		int N = ceilf(dist / resolution);
		if(N == 1) N = 2;
		if(N > CCD_MAX_INTERPOLATION) N = CCD_MAX_INTERPOLATION;
		
		return N - 1;
	}
};


/** Functor to generate max number of interpolation number for subdivision mode
 */
class SubdivisionMaxIterFunctor
{
public:
	/** Generator
	 * @param dist Distance between two configurations
	 * @param resolution Resolution for CCD checking
	 * @return Max number of interpolation number (the number of max partition - 1)
	 */
	__device__ unsigned int operator()(const float dist, const float resolution) const
	{
		int N = ceilf(logf(dist / resolution) / logf(2.0));
		if(N < 0) N = 1;
		if(N > LOG_CCD_MAX_INTERPOLATION) N = LOG_CCD_MAX_INTERPOLATION;
		
		return (1 << N) - 1;
	}
};


/** Functor to generate the interpolation weight for ith interpolation in incremental mode
 */
class IncrementalInterpFunctor
{
public:
	/** Generator
	 * @param i Index of interpolation point
	 * @param N Max number of interpolation point
	 */
	__device__ float operator()(const unsigned int i, const unsigned int N) const
	{
		return (i + 1) / (float)(N + 1);
	}
};

/** Functor to generate the interpolation weight for ith interpolation in subdivision mode
 */
class SubdivisionInterpFunctor
{
public:
	/** Generator
	 * @param i Index of interpolation point
	 * @param N Max number of interpolation point
	 */
	__device__ float operator()(const unsigned int i, const unsigned int N) const
	{
		int tmp = ceilf(logf((float)i + 2.0f) / logf(2.0f));
		tmp = (1 << tmp);
		return (2.0f * (i + 1 - (tmp >> 1)) + 1) / (float)tmp;
	}
};

#endif