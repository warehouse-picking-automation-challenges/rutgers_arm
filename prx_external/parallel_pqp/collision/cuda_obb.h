#ifndef __CUDA_OBB_H_	
#define __CUDA_OBB_H_

#include "cuda_vectors.h"
#include "defs.h"


/** Oriented bounding box (60 bytes)
*/
class OBB
{
  public:
    __device__ __inline__ void calcExtentsForOBB(OBB &bv, float3 &minExtent, float3 &maxExtent);

    __device__ __inline__ void init(OBB &bv1, OBB &bv2);

    // init OBB from triangle
    __device__ __inline__ void init(const float3 &v1, const float3 &v2, const float3 &v3);

    __device__ __inline__ void init(const float3 &v1, const float3 &v2, const float3 &v3, const float3 &u1, const float3 &u2, const float3 &u3);

    __device__ __inline__ float getSize() const
    {
      return extents.x * extents.x + extents.y * extents.y + extents.z * extents.z;
    }

    float3 center;		// center point of OBB
    float3 axis1,		// major axes specifying the OBB
           axis2,
           axis3;
    float3 extents;		// extents to boundary from center point,
    // along each axis
};



// OBBTree node (68 bytes)
class OBBNode
{
  public:
    OBB bbox;					// bounding box for node
    unsigned int left,			// pointers to left/right children
                 right;

    __device__ __inline__ bool isLeaf() const
    {
      return left == 0;
    }

    __device__ __inline__ int getLeftChild() const
    {
      return left;
    }
    __device__ __inline__ int getTriID() const
    {
      return right;
    }
};

  template<typename T>
__device__ __inline__  T isLeftOBBNode(T id)
{
  return (id & 1) == 0;
}

  template<typename T>
__device__ __inline__ T getParentOBBNodeID(T id)
{
  return (id >> 1);
}

  template<typename T>
__device__ __inline__ T getNextOBBNodeID(T id)
{
  do 
  {
    if (isLeftOBBNode(id))
      return id + 1;
    id = getParentOBBNodeID(id);
  } while (id != 0);
  return 0;
}



#endif
