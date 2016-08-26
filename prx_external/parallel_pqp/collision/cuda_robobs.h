#ifndef __CUDA_ROBOBS_H_
#define __CUDA_ROBOBS_H_

#include "cuda_obb.h"
#include "cuda_vertex.h"
#include <vector>
#include "../PQP/PQP.h"

/**
 * OBB structure on CPU
 */
struct OBB_host
{
	float3 center;		///< center point of OBB
	float3 axis1,		///< major axes specifying the OBB
	       axis2,
	       axis3;
	float3 extents;		///< half extensions along each axis
};

/**
 * OBB Node structure on CPU
 */
class OBBNode_host
{
public:
	OBB_host bbox;				///< bounding box (OBB)
	unsigned int left,			///< pointers to left/right children (right is hardly used)
	         right;
};
/**
 * triangle mesh on GPU
 */
class GMesh
{
public:
	OBBNode* models;
	int modelsize; // TriSize
	GPUVertex* vertexPointers;
	uint3* triIdxPointers;

	GMesh(PQP_Model* pTree, const std::vector<std::vector<float> >& triangles);
  int traverseTree(PQP_Model* pTree, int i, int level);
  void copyNodes(OBBNode_host* hTree, PQP_Model* pTree, int destIndex, int srcIndex);
};

#endif
