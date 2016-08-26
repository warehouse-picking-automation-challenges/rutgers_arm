#ifndef CUDA_RRT_TYPES_H
#define CUDA_RRT_TYPES_H

#include "cuda_defs.h"

union SerializedStates
{
	float x[8];//(x, y, z) and (x, y, z, w) 8 is for alignment
	struct Components
	{
		float r[3];
		float q[5];
	} components;
};	
#endif
