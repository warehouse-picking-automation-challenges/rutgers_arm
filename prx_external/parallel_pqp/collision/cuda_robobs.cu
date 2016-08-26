//#include <cuda.h>
//#include <cuda_runtime.h>
#include <algorithm>
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "cuda_robobs.h"
#include "../cuda_defs.h"
#include "../cuda_errorcheck.h"

int GMesh::traverseTree(PQP_Model* pTree, int i, int level)
{
	int isLeaf = pTree->child(i)->Leaf();
	if (isLeaf)
		return level;

	int leftLevel = traverseTree(pTree, pTree->b[i].first_child, level + 1);
	int rightLevel = traverseTree(pTree, pTree->b[i].first_child + 1, level + 1);

	return std::max(leftLevel, rightLevel);
}

void GMesh::copyNodes(OBBNode_host* hTree, PQP_Model* pTree, int destIndex, int srcIndex)
{
	OBBNode_host node;
	int isLeaf = pTree->child(srcIndex)->Leaf();
	if(isLeaf)
	{
		int triId = (- pTree->child(srcIndex)->first_child - 1);
		int largest_tri = pTree->num_tris;
		triId = pTree->tris[triId].id; ///< triangles' id will change in EndModel(), i.e. triId != id

		// left of leaf = 0
		node.left = 0;
		node.right = triId;

		node.bbox.axis1 = make_float3((float)(pTree->b[srcIndex].R[0][0]), (float)(pTree->b[srcIndex].R[1][0]), (float)(pTree->b[srcIndex].R[2][0]));
		node.bbox.axis2 = make_float3((float)(pTree->b[srcIndex].R[0][1]), (float)(pTree->b[srcIndex].R[1][1]), (float)(pTree->b[srcIndex].R[2][1]));
		node.bbox.axis3 = make_float3((float)(pTree->b[srcIndex].R[0][2]), (float)(pTree->b[srcIndex].R[1][2]), (float)(pTree->b[srcIndex].R[2][2]));
		node.bbox.center = make_float3((float)(pTree->b[srcIndex].To[0]), (float)(pTree->b[srcIndex].To[1]), (float)(pTree->b[srcIndex].To[2]));
		node.bbox.extents = make_float3((float)(pTree->b[srcIndex].d[0]), (float)(pTree->b[srcIndex].d[1]), (float)(pTree->b[srcIndex].d[2]));

		hTree[destIndex] = node;
	}
	else
	{
		int srcLeft = pTree->b[srcIndex].first_child;
		int srcRight = pTree->b[srcIndex].first_child + 1;
		int destLeft = (destIndex << 1);
		int destRight = (destIndex << 1) + 1;

		node.left = destLeft;
		// right of internal node = 0
		node.right = 0;
		node.bbox.axis1 = make_float3((float)(pTree->b[srcIndex].R[0][0]), (float)(pTree->b[srcIndex].R[1][0]), (float)(pTree->b[srcIndex].R[2][0]));
		node.bbox.axis2 = make_float3((float)(pTree->b[srcIndex].R[0][1]), (float)(pTree->b[srcIndex].R[1][1]), (float)(pTree->b[srcIndex].R[2][1]));
		node.bbox.axis3 = make_float3((float)(pTree->b[srcIndex].R[0][2]), (float)(pTree->b[srcIndex].R[1][2]), (float)(pTree->b[srcIndex].R[2][2]));
		node.bbox.center = make_float3((float)(pTree->b[srcIndex].To[0]), (float)(pTree->b[srcIndex].To[1]), (float)(pTree->b[srcIndex].To[2]));
		node.bbox.extents = make_float3((float)(pTree->b[srcIndex].d[0]), (float)(pTree->b[srcIndex].d[1]), (float)(pTree->b[srcIndex].d[2]));

		hTree[destIndex] = node;

		copyNodes(hTree, pTree, destLeft, srcLeft);
		copyNodes(hTree, pTree, destRight, srcRight);
	}
}

GMesh::GMesh(PQP_Model* pTree, const std::vector<std::vector<float> >& triangles)
{
	int maxLevel = traverseTree(pTree, 0, 0);
	int numNodes = (2 << maxLevel);

	OBBNode_host* hTree = new OBBNode_host[numNodes];
	copyNodes(hTree, pTree, 1, 0);
	/*
	OBBNode_host* hTree = new OBBNode_host[pTree->num_bvs];

	for(int i = 0; i < pTree->num_bvs; ++i)
	{
		OBBNode_host node;
		int isLeaf = pTree->child(i)->Leaf();
		if(isLeaf)
		{
			int triId = (- pTree->child(i)->first_child - 1);
			triId = pTree->tris[triId].id; ///< triangles' id will change in EndModel(), i.e. triId != id
			node.left = ((triId << 2) | 3);
			node.right = 0;
			node.bbox.axis1 = make_float3((float)(pTree->b[i].R[0][0]), (float)(pTree->b[i].R[1][0]), (float)(pTree->b[i].R[2][0]));
			node.bbox.axis2 = make_float3((float)(pTree->b[i].R[0][1]), (float)(pTree->b[i].R[1][1]), (float)(pTree->b[i].R[2][1]));
			node.bbox.axis3 = make_float3((float)(pTree->b[i].R[0][2]), (float)(pTree->b[i].R[1][2]), (float)(pTree->b[i].R[2][2]));
			node.bbox.center = make_float3((float)(pTree->b[i].To[0]), (float)(pTree->b[i].To[1]), (float)(pTree->b[i].To[2]));
			node.bbox.extents = make_float3((float)(pTree->b[i].d[0]), (float)(pTree->b[i].d[1]), (float)(pTree->b[i].d[2]));
		}
		else
		{
			node.left = ((pTree->b[i].first_child - i) << 5);
			node.right = 0;
			node.bbox.axis1 = make_float3((float)(pTree->b[i].R[0][0]), (float)(pTree->b[i].R[1][0]), (float)(pTree->b[i].R[2][0]));
			node.bbox.axis2 = make_float3((float)(pTree->b[i].R[0][1]), (float)(pTree->b[i].R[1][1]), (float)(pTree->b[i].R[2][1]));
			node.bbox.axis3 = make_float3((float)(pTree->b[i].R[0][2]), (float)(pTree->b[i].R[1][2]), (float)(pTree->b[i].R[2][2]));
			node.bbox.center = make_float3((float)(pTree->b[i].To[0]), (float)(pTree->b[i].To[1]), (float)(pTree->b[i].To[2]));
			node.bbox.extents = make_float3((float)(pTree->b[i].d[0]), (float)(pTree->b[i].d[1]), (float)(pTree->b[i].d[2]));
		}
		hTree[i] = node;
	}
	*/

	//
	OBBNode_host* object = hTree;
	//int numOBBs = 2 * triangles.size() - 1;

	OBBNode* d_obbTree = NULL;
	// printf("%d\n",numNodes);
	GPUMALLOC((void**)&d_obbTree, sizeof(OBBNode) * numNodes);
	OBBNode* h_obbTree = new OBBNode[numNodes];
	for(int j = 0; j < numNodes; ++j)
	{
		h_obbTree[j].bbox.axis1 = object[j].bbox.axis1;
		h_obbTree[j].bbox.axis2 = object[j].bbox.axis2;
		h_obbTree[j].bbox.axis3 = object[j].bbox.axis3;
		h_obbTree[j].bbox.center = object[j].bbox.center;
		h_obbTree[j].bbox.extents = object[j].bbox.extents;
		h_obbTree[j].left = object[j].left;
		h_obbTree[j].right = object[j].right;
	}


	TOGPU(d_obbTree, h_obbTree, sizeof(OBBNode) * numNodes);

	models = d_obbTree;

	modelsize = triangles.size();

	GPUVertex* d_vertexPointers = NULL;
	GPUMALLOC((void**)&d_vertexPointers, sizeof(GPUVertex) * triangles.size() * 3);
	GPUVertex* h_vertexPointers = new GPUVertex[triangles.size() * 3];
	for (unsigned int i = 0; i < triangles.size(); ++i)
	{
		h_vertexPointers[i * 3].v.x = triangles[i][0];
		h_vertexPointers[i * 3].v.y = triangles[i][1];
		h_vertexPointers[i * 3].v.z = triangles[i][2];

		h_vertexPointers[i * 3 + 1].v.x = triangles[i][3];
		h_vertexPointers[i * 3 + 1].v.y = triangles[i][4];
		h_vertexPointers[i * 3 + 1].v.z = triangles[i][5];

		h_vertexPointers[i * 3 + 2].v.x = triangles[i][6];
		h_vertexPointers[i * 3 + 2].v.y = triangles[i][7];
		h_vertexPointers[i * 3 + 2].v.z = triangles[i][8];
	}
	TOGPU(d_vertexPointers, h_vertexPointers, sizeof(GPUVertex) * triangles.size() * 3);
	delete[] h_vertexPointers;
	vertexPointers = d_vertexPointers;

	uint3* d_triIdxPointers = NULL;
	GPUMALLOC((void**)&d_triIdxPointers, sizeof(uint3) * triangles.size());
	uint3* h_triIdxPointers = new uint3[triangles.size()];
	for (unsigned int i = 0; i < triangles.size(); ++i)
	{
		h_triIdxPointers[i].x = i * 3;
		h_triIdxPointers[i].y = i * 3 + 1;
		h_triIdxPointers[i].z = i * 3 + 2;
	}
	TOGPU(d_triIdxPointers, h_triIdxPointers, sizeof(uint3) * triangles.size());
	delete[] h_triIdxPointers;
	triIdxPointers = d_triIdxPointers;

	delete[] hTree;
}
