#ifndef CUDA_STATE_PRIMITIVES_H
#define CUDA_STATE_PRIMITIVES_H

#include "cuda_defs.h"
#include "cuda_rrt_types.h"

// assume 
// int gridX = (queryWidth + 31) / 32;
// dim3 grids = dim3(gridX, 1, 1);
// dim3 threads = dim3(7, 32, 1);

// G->S
__device__ void copyIndexedStatesFromGlobalToShared(int inNumQueries, SerializedStates* inGlobalStates, int* inStateIndices, 
	SerializedStates* outSharedStates)
{
	__shared__ int sharedStateId[32];

	int queryId = blockIdx.x * 32 + threadIdx.y;
	int sharedDataIdx = threadIdx.y;
	int componentIdx = threadIdx.x;

	if (threadIdx.x == 0)
		sharedStateId[queryId] = inStateIndices[queryId];
	__syncthreads();

	int globalStateIdx = sharedStateId[queryId];

	outSharedStates[sharedDataIdx].x[componentIdx] = inGlobalStates[globalStateIdx].x[componentIdx];
}

// G->S
__device__ void copyStatesFromGlobalToShared(int queryWidth, float* global, float* shared)
{
	int queryId = blockIdx.x * 32 + threadIdx.y;
	int sharedDataIdx = threadIdx.y * STATE_STRIDE + threadIdx.x;
	int globalDataIdx = queryId * STATE_STRIDE + threadIdx.x;

	shared[sharedDataIdx] = global[globalDataIdx];
}

// S->G
__device__ void copyStatesFromSharedToGlobal(int queryWidth, float* shared, float* global)
{
	int queryId = blockIdx.x * 32 + threadIdx.y;
	int sharedDataIdx = threadIdx.y * STATE_STRIDE + threadIdx.x;
	int globalDataIdx = queryId * STATE_STRIDE + threadIdx.x;

	global[globalDataIdx] = shared[sharedDataIdx];
}

// S->S
__device__ void computeStateDistances(SerializedStates* fromStates, SerializedStates* toStates, float* distancesR3, float* distancesSO3)
{
	__shared__ SerializedStates squared[32];

	int componentIdx = threadIdx.x;
	int sharedDataIdx = threadIdx.y;

	float src, dest;

	src = fromStates[sharedDataIdx].x[componentIdx];
	dest = toStates[sharedDataIdx].x[componentIdx];

	if (componentIdx < 3)
	{
		squared[sharedDataIdx].x[componentIdx] = (dest - src) * (dest - src);
	}
	else
	{
		squared[sharedDataIdx].x[componentIdx] = src * dest;
	}
	__syncthreads();

	if (componentIdx == 0)
	{
		float distR3 = sqrtf(squared[sharedDataIdx].components.r[0] + squared[sharedDataIdx].components.r[1] + squared[sharedDataIdx].components.r[2]);
		float x2 = fabs(squared[sharedDataIdx].components.q[0] + squared[sharedDataIdx].components.q[1] +
			squared[sharedDataIdx].components.q[2] + squared[sharedDataIdx].components.q[3]);
		x2 = (x2 > 1 - 1e-7) ? 0.0 : acosf(x2);
		float distSO3 = x2;

		distancesR3[threadIdx.y] = distR3;
		distancesSO3[threadIdx.y] = distSO3;
	}
}

__device__ void computeStateDistances(SerializedStates* fromStates, SerializedStates* toStates, float* distances)
{
	__shared__ SerializedStates squared[32];

	int componentIdx = threadIdx.x;
	int sharedDataIdx = threadIdx.y;

	float src, dest;

	src = fromStates[sharedDataIdx].x[componentIdx];
	dest = toStates[sharedDataIdx].x[componentIdx];

	if (componentIdx < 3)
	{
		squared[sharedDataIdx].x[componentIdx] = (dest - src) * (dest - src);
	}
	else
	{
		squared[sharedDataIdx].x[componentIdx] = src * dest;
	}
	__syncthreads();

	if (componentIdx == 0)
	{
		float distR3 = sqrtf(squared[sharedDataIdx].components.r[0] + squared[sharedDataIdx].components.r[1] + squared[sharedDataIdx].components.r[2]);
		float x2 = fabs(squared[sharedDataIdx].components.q[0] + squared[sharedDataIdx].components.q[1] +
			squared[sharedDataIdx].components.q[2] + squared[sharedDataIdx].components.q[3]);
		x2 = (x2 > 1 - 1e-7) ? 0.0 : acosf(x2);
		float distSO3 = x2;

		distances[threadIdx.y] = distR3 + distSO3;
	}
}

__device__ void computeStateDistancesToSingleState(SerializedStates* fromStates, SerializedStates* toState, float* distances)
{
	__shared__ SerializedStates squared[32];

	int componentIdx = threadIdx.x;
	int sharedDataIdx = threadIdx.y;

	float src, dest;

	src = fromStates[sharedDataIdx].x[componentIdx];
	dest = toState->x[componentIdx];

	if (componentIdx < 3)
	{
		squared[sharedDataIdx].x[componentIdx] = (dest - src) * (dest - src);
	}
	else
	{
		squared[sharedDataIdx].x[componentIdx] = src * dest;
	}
	__syncthreads();

	if (componentIdx == 0)
	{
		float distR3 = sqrtf(squared[sharedDataIdx].components.r[0] + squared[sharedDataIdx].components.r[1] + squared[sharedDataIdx].components.r[2]);
		float x2 = fabs(squared[sharedDataIdx].components.q[0] + squared[sharedDataIdx].components.q[1] +
			squared[sharedDataIdx].components.q[2] + squared[sharedDataIdx].components.q[3]);
		x2 = (x2 > 1 - 1e-7) ? 0.0 : acosf(x2);
		float distSO3 = x2;

		distances[threadIdx.y] = distR3 + distSO3;
	}
}

__device__ void interpolateStates(int queryWidth, SerializedStates* fromStates, SerializedStates* toStates, float t, float* distancesSO3, SerializedStates* interpState)
{
	__shared__ SerializedStates so3Inter[32];
	__shared__ float so3s0[32];
	__shared__ float so3s1[32];
	__shared__ float so3d[32];

	int sharedDataIdx = threadIdx.y;
	int componentIdx = threadIdx.x;

	float from, to;
	from = fromStates[sharedDataIdx].x[componentIdx];
	to = toStates[sharedDataIdx].x[componentIdx];

	// compute interpolation pt
	if (threadIdx.x < 3)
	{
		interpState[sharedDataIdx].x[componentIdx] = from + (to - from) * t;
	}
	else
	{
		// SO3 interpolation

		// compute length 
		if (threadIdx.x == 3)
		{
			float theta = distancesSO3[sharedDataIdx];
			so3d[sharedDataIdx] = 1.0 / sinf(theta);
			so3s0[sharedDataIdx] = sinf((1.0 - t) * theta);
			so3s1[sharedDataIdx] = sinf(t * theta);
		}

		so3Inter[sharedDataIdx].x[componentIdx] = from * to;
		__syncthreads();

		if (threadIdx.x == 3)
		{
			float dq = so3Inter[sharedDataIdx].components.q[0] + so3Inter[sharedDataIdx].components.q[1] +
				so3Inter[sharedDataIdx].components.q[2] + so3Inter[sharedDataIdx].components.q[3];
			if (dq < 0)
				so3s1[sharedDataIdx] = -so3s1[sharedDataIdx];
		}
		__syncthreads();

		interpState[sharedDataIdx].x[componentIdx] = (from * so3s0[sharedDataIdx] + to * so3s1[sharedDataIdx]) * so3d[sharedDataIdx];
	}
}

__device__ void scaledDiff(SerializedStates* from, SerializedStates* to, SerializedStates* diff, float scale)
{
	int componentIdx = threadIdx.x;
	int sharedDataIdx = threadIdx.y;

	if (componentIdx < 3)
	{
		diff[sharedDataIdx].x[componentIdx] = scale * (to[sharedDataIdx].x[componentIdx] - from[sharedDataIdx].x[componentIdx]);
	}
	else if (componentIdx < 6)
	{
		int cur = componentIdx - 3;
		int next = (componentIdx - 3 + 1) % 3;
		int next2 = (componentIdx - 3 + 2) % 3;
		diff[sharedDataIdx].x[componentIdx] = from[sharedDataIdx].components.q[3] * to[sharedDataIdx].components.q[cur]
			- from[sharedDataIdx].components.q[cur] * to[sharedDataIdx].components.q[3]
			- from[sharedDataIdx].components.q[next] * to[sharedDataIdx].components.q[next2]
			+ from[sharedDataIdx].components.q[next2] * to[sharedDataIdx].components.q[next];
	}
	else if (componentIdx == 6)
	{
		diff[sharedDataIdx].x[componentIdx] = from[sharedDataIdx].components.q[3] * to[sharedDataIdx].components.q[3]
		+ from[sharedDataIdx].components.q[0] * to[sharedDataIdx].components.q[0]
		+ from[sharedDataIdx].components.q[1] * to[sharedDataIdx].components.q[1]
		+ from[sharedDataIdx].components.q[2] * to[sharedDataIdx].components.q[2];
	}
}

__device__ void scaledAdd(SerializedStates* from, SerializedStates* add, SerializedStates* sum, float scale)
{
	int componentIdx = threadIdx.x;
	int sharedDataIdx = threadIdx.y;

	if (componentIdx < 3)
	{
		sum[sharedDataIdx].x[componentIdx] = from[sharedDataIdx].x[componentIdx] + scale * add[sharedDataIdx].x[componentIdx];
	}
	else if (componentIdx < 6)
	{
		int cur = componentIdx - 3;
		int next = (componentIdx - 3 + 1) % 3;
		int next2 = (componentIdx - 3 + 2) % 3;
		sum[sharedDataIdx].x[componentIdx] = from[sharedDataIdx].components.q[3] * add[sharedDataIdx].components.q[cur]
		+ from[sharedDataIdx].components.q[cur] * add[sharedDataIdx].components.q[3]
		+ from[sharedDataIdx].components.q[next] * add[sharedDataIdx].components.q[next2]
		- from[sharedDataIdx].components.q[next2] * add[sharedDataIdx].components.q[next];
	}
	else if (componentIdx == 6)
	{
		sum[sharedDataIdx].x[componentIdx] = from[sharedDataIdx].components.q[3] * add[sharedDataIdx].components.q[3]
		- from[sharedDataIdx].components.q[0] * add[sharedDataIdx].components.q[0]
		- from[sharedDataIdx].components.q[1] * add[sharedDataIdx].components.q[1]
		- from[sharedDataIdx].components.q[2] * add[sharedDataIdx].components.q[2];
	}
}

__device__ void applyAdaptiveTemplate(SerializedStates* state, SerializedStates* free, SerializedStates* templateStates, int numTemplateStates,
	int stateLevel, int* numSamples, SerializedStates* samples, int* levels, RRT_TREE_ID* isTreeNode)
{
	__shared__ SerializedStates sharedScaledState[32];
	__shared__ SerializedStates sharedClosestTemplateState[32];
	__shared__ int sharedNewSampleId[32];
	__shared__ float sharedDistances[32];
	__shared__ SerializedStates sharedTemplateState;

	int sharedDataIdx = threadIdx.y;
	int componentIdx = threadIdx.x;

	float scale = (float)(1 << stateLevel);
	scaledDiff(state, free, sharedScaledState, scale);
	__syncthreads();

	// TODO: get closest template state
	// nearest neighbor sharedClosestTemplateState from sharedScaledState;
	float minDist = FLOAT_MAX;
	for (int i = 0; i < numTemplateStates; ++i)
	{
		if (threadIdx.y == 0)
			sharedTemplateState.x[componentIdx] = templateStates[i].x[componentIdx];
		__syncthreads();

		computeStateDistancesToSingleState(sharedScaledState, &sharedTemplateState, sharedDistances);
		__syncthreads();

		if (sharedDistances[sharedDataIdx] < minDist)
		{
			minDist = sharedDistances[sharedDataIdx];
			sharedClosestTemplateState[sharedDataIdx].x[componentIdx] = sharedTemplateState.x[componentIdx];
		}

	}

	// add new state to samples
	int newSampleId = -1; 
	if (threadIdx.x == 0)
	{
		newSampleId = atomicAdd(numSamples, 1);
		sharedNewSampleId[sharedDataIdx] = newSampleId;

		levels[newSampleId] = stateLevel + 1;
		isTreeNode[newSampleId] = NO_TREE;
	}
	__syncthreads();

	newSampleId = sharedNewSampleId[sharedDataIdx];
	scaledAdd(state, sharedClosestTemplateState, sharedScaledState, 1.0 / scale);
	samples[newSampleId].x[componentIdx] = sharedScaledState[sharedDataIdx].x[componentIdx];

#ifdef DEBUG_NEW_SAMPLE
	if (threadIdx.x == 0)
	{
		__syncthreads();
		printf("%d New Sample(%.3f %.3f %.3f | %.3f %.3f %.3f %.3f)Lv %d \n", newSampleId,
			sharedScaledState[sharedDataIdx].x[0],
			sharedScaledState[sharedDataIdx].x[1],
			sharedScaledState[sharedDataIdx].x[2],
			sharedScaledState[sharedDataIdx].x[3],
			sharedScaledState[sharedDataIdx].x[4],
			sharedScaledState[sharedDataIdx].x[5],
			sharedScaledState[sharedDataIdx].x[6],
			levels[newSampleId]);
	}
#endif
}

__device__ bool subdivideDisk(SerializedStates* state, SerializedStates* free, SerializedStates* templateStates, int numTemplateStates, 
	int stateLevel, float stateRadius, int* numSamples, SerializedStates* samples, int* levels, RRT_TREE_ID* isTreeNode)
{
	__shared__ float sharedDistances[32];

	if (stateLevel == MAX_LEVEL - 1)
		return false;

	computeStateDistances(state, free, sharedDistances);
	__syncthreads();
	float distance = sharedDistances[threadIdx.y];

	if (distance < stateRadius)
	{
		applyAdaptiveTemplate(state, free, templateStates, numTemplateStates, stateLevel, numSamples, samples, levels, isTreeNode);
		return true;
	}

	return false;
}

__device__ float computeStateDistance(SerializedStates& fromState, SerializedStates& toState)
{
	SerializedStates squared;

	squared.x[0] = toState.x[0] - fromState.x[0];
	squared.x[1] = toState.x[1] - fromState.x[1];
	squared.x[2] = toState.x[2] - fromState.x[2];
	squared.x[0] *= squared.x[0];
	squared.x[1] *= squared.x[1];
	squared.x[2] *= squared.x[2];

	squared.x[3] = fromState.x[3] * toState.x[3];
	squared.x[4] = fromState.x[4] * toState.x[4];
	squared.x[5] = fromState.x[5] * toState.x[5];
	squared.x[6] = fromState.x[6] * toState.x[6];

	float distR3 = sqrtf(squared.components.r[0] + squared.components.r[1] + squared.components.r[2]);
	float x2 = fabs(squared.components.q[0] + squared.components.q[1] +
		squared.components.q[2] + squared.components.q[3]);
	x2 = (x2 > 1 - 1e-7) ? 0.0 : acosf(x2);
	float distSO3 = x2;

	return distR3 + distSO3;
}

__device__ void scaledDiffS(const SerializedStates& from, const SerializedStates& to, SerializedStates& diff, float scale)
{
	diff.x[0] = scale * (to.x[0] - from.x[0]);
	diff.x[1] = scale * (to.x[1] - from.x[1]);
	diff.x[2] = scale * (to.x[2] - from.x[2]);

	diff.x[3] = from.components.q[3] * to.components.q[0] - from.components.q[0] * to.components.q[3] - from.components.q[1] * to.components.q[2] + from.components.q[2] * to.components.q[1];
	diff.x[4] = from.components.q[3] * to.components.q[1] - from.components.q[1] * to.components.q[3] - from.components.q[2] * to.components.q[0] + from.components.q[0] * to.components.q[2];
	diff.x[5] = from.components.q[3] * to.components.q[2] - from.components.q[2] * to.components.q[3] - from.components.q[0] * to.components.q[1] + from.components.q[1] * to.components.q[0];
	diff.x[6] = from.components.q[3] * to.components.q[3] + from.components.q[0] * to.components.q[0] + from.components.q[1] * to.components.q[1] + from.components.q[2] * to.components.q[2];
}

__device__ void scaledAddS(const SerializedStates& from, const SerializedStates& add, SerializedStates& sum, float scale)
{
	sum.x[0] = from.x[0] + scale * add.x[0];
	sum.x[1] = from.x[1] + scale * add.x[1];
	sum.x[2] = from.x[2] + scale * add.x[2];

	sum.x[3] = from.components.q[3] * add.components.q[0] + from.components.q[0] * add.components.q[3] + from.components.q[1] * add.components.q[2] - from.components.q[2] * add.components.q[1];
	sum.x[4] = from.components.q[3] * add.components.q[1] + from.components.q[1] * add.components.q[3] + from.components.q[2] * add.components.q[0] - from.components.q[2] * add.components.q[2];
	sum.x[5] = from.components.q[3] * add.components.q[2] + from.components.q[2] * add.components.q[3] + from.components.q[0] * add.components.q[1] - from.components.q[2] * add.components.q[0];
	sum.x[6] = from.components.q[3] * add.components.q[3] - from.components.q[0] * add.components.q[0] - from.components.q[1] * add.components.q[1] - from.components.q[2] * add.components.q[2];
}

__device__ void applyAdaptiveTemplateS(SerializedStates& state, SerializedStates& free, SerializedStates* templateStates, int numTemplateStates,
	int stateLevel, int* numSamples, SerializedStates* samples, int* levels, RRT_TREE_ID* isTreeNode, curandState& randState)
{
	SerializedStates scaledState;
	SerializedStates closestTemplateState;

	float scale = (float)(1 << stateLevel);
	scaledDiffS(state, free, scaledState, scale);

	// get closest template state
	// nearest neighbor sharedClosestTemplateState from sharedScaledState;
	float minDist = FLOAT_MAX;
	for (int i = 0; i < numTemplateStates; ++i)
	{
		SerializedStates templateState = templateStates[i];

		float distance = computeStateDistance(scaledState, templateState);
		if (distance < minDist)
		{
			minDist = distance;
			closestTemplateState = templateState;
		}

	}

	// add new state to samples
	int newSampleId = atomicAdd(numSamples, 1);
	levels[newSampleId] = stateLevel + 1;
	isTreeNode[newSampleId] = NO_TREE;

	scaledAddS(state, closestTemplateState, scaledState, 1.0 / scale);

	/*
	// random rotation
	float x = curand_uniform(&randState);
	float x2 = curand_uniform(&randState);

	float c1 = sqrtf(1 - x);
	float c2 = sqrtf(x);
	
	float sinpix2, cospix2;
	sincospif(x2, &sinpix2, &cospix2);
	scaledState.components.q[0] = sinpix2 * c1;
	scaledState.components.q[1] = cospix2 * c1;
	scaledState.components.q[2] = sinpix2 * c2;
	scaledState.components.q[3] = cospix2 * c2;
	*/

	samples[newSampleId] = scaledState;

#ifdef DEBUG_NEW_SAMPLE
	printf("%d New Sample(%.3f %.3f %.3f | %.3f %.3f %.3f %.3f)Lv %d \n", newSampleId,
		scaledState.x[0],
		scaledState.x[1],
		scaledState.x[2],
		scaledState.x[3],
		scaledState.x[4],
		scaledState.x[5],
		scaledState.x[6],
		levels[newSampleId]);
#endif
}

__device__ bool subdivideDiskS(SerializedStates& state, SerializedStates& free, SerializedStates* templateStates, int numTemplateStates, 
	int stateLevel, float stateRadius, int* numSamples, SerializedStates* samples, int* levels, RRT_TREE_ID* isTreeNode, curandState& randState)
{
	if (stateLevel == MAX_LEVEL - 1)
	{
		return false;
	}

	float distance = computeStateDistance(state, free);

	if (distance < stateRadius)
	{
		applyAdaptiveTemplateS(state, free, templateStates, numTemplateStates, stateLevel, numSamples, samples, levels, isTreeNode, randState);

		return true;
	}

	return false;
}

#endif