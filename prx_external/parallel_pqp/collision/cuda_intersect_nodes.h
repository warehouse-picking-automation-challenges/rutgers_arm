#ifndef __CUDA_INTERSECT_NODES_H_
#define __CUDA_INTERSECT_NODES_H_

#include "cuda_obb.h"
#include <vector_functions.h>

#define OBB_ROTATION_MATRIX_EPSILON 0.000001f

template <class BV>
__device__ __inline__ bool intersect(const BV &node1, const BV &node2)
{
	return true;
}


template <>
__device__ __inline__ bool intersect<OBB>(const OBB &node1, const OBB &node2)
{

	//translation, in parent frame
	float3 v = f3v_sub(node1.center, node2.center);
	
	//translation, in A's frame
	float3 T = make_float3(f3v_dot(v, node1.axis1), f3v_dot(v, node1.axis2), f3v_dot(v, node1.axis3));
	
	
	//calculate rotation matrix (B's basis with respect to A', R1)
	float3 R1;
	R1.x = f3v_dot(node1.axis1, node2.axis1);
	R1.y = f3v_dot(node1.axis1, node2.axis2);
	R1.z = f3v_dot(node1.axis1, node2.axis3);
	
	/*
	ALGORITHM: Use the separating axis test for all 15 potential
	separating axes. If a separating axis could not be found, the two
	boxes overlap.
	*/
	
	// Axes: A's basis vectors
	{
		float rb;
		rb = node2.extents.x * fabs(R1.x) + node2.extents.y * fabs(R1.y) + node2.extents.z * fabs(R1.z) + OBB_ROTATION_MATRIX_EPSILON * (node2.extents.x + node2.extents.y + node2.extents.z);
		if(fabs(T.x) > (node1.extents.x + rb))
			return false;
	}
	
	//calculate rotation matrix (B's basis with respect to A', R2)
	float3 R2;
	R2.x = f3v_dot(node1.axis2, node2.axis1);
	R2.y = f3v_dot(node1.axis2, node2.axis2);
	R2.z = f3v_dot(node1.axis2, node2.axis3);
	
	{
		float rb;
		rb = node2.extents.x * fabs(R2.x) + node2.extents.y * fabs(R2.y) + node2.extents.z * fabs(R2.z) + OBB_ROTATION_MATRIX_EPSILON * (node2.extents.x + node2.extents.y + node2.extents.z);
		if(fabs(T.y) > (node1.extents.y + rb))
			return false;
	}
	
	//calculate rotation matrix (B's basis with respect to A', R3)
	float3 R3;
	R3.x = f3v_dot(node1.axis3, node2.axis1);
	R3.y = f3v_dot(node1.axis3, node2.axis2);
	R3.z = f3v_dot(node1.axis3, node2.axis3);
	
	{
		float rb;
		rb = node2.extents.x * fabs(R3.x) + node2.extents.y * fabs(R3.y) + node2.extents.z * fabs(R3.z) + OBB_ROTATION_MATRIX_EPSILON * (node2.extents.x + node2.extents.y + node2.extents.z);;
		if(fabs(T.z) > (node1.extents.z + rb))
			return false;
	}
	
	// Axes: B's basis vectors
	{
		float rb, t;
		rb = node1.extents.x * fabs(R1.x) + node1.extents.y * fabs(R2.x) + node1.extents.z * fabs(R3.x) + OBB_ROTATION_MATRIX_EPSILON * (node1.extents.x + node1.extents.y + node1.extents.z);;
		t = fabs(T.x * R1.x + T.y * R2.x + T.z * R3.x);
		if(t > (node2.extents.x + rb))
			return false;
			
		rb = node1.extents.x * fabs(R1.y) + node1.extents.y * fabs(R2.y) + node1.extents.z * fabs(R3.y) + OBB_ROTATION_MATRIX_EPSILON * (node1.extents.x + node1.extents.y + node1.extents.z);;;
		t = fabs(T.x * R1.y + T.y * R2.y + T.z * R3.y);
		if(t > (node2.extents.y + rb))
			return false;
			
		rb = node1.extents.x * fabs(R1.z) + node1.extents.y * fabs(R2.z) + node1.extents.z * fabs(R3.z) + OBB_ROTATION_MATRIX_EPSILON * (node1.extents.x + node1.extents.y + node1.extents.z);;;
		t = fabs(T.x * R1.z + T.y * R2.z + T.z * R3.z);
		if(t > (node2.extents.z + rb))
			return false;
	}
	
	// Axes: 9 cross products
	
	//L = A0 x B0
	{
		float ra, rb, t;
		ra = node1.extents.y * fabs(R3.x) + node1.extents.z * fabs(R2.x) + OBB_ROTATION_MATRIX_EPSILON * (node1.extents.y + node1.extents.z);
		rb = node2.extents.y * fabs(R1.z) + node2.extents.z * fabs(R1.y) + OBB_ROTATION_MATRIX_EPSILON * (node2.extents.y + node2.extents.z);
		t = fabs(T.z * R2.x - T.y * R3.x);
		
		if(t > ra + rb)
			return false;
	}
	
	//L = A0 x B1
	{
		float ra, rb, t;
		ra = node1.extents.y * fabs(R3.y) + node1.extents.z * fabs(R2.y) + OBB_ROTATION_MATRIX_EPSILON * (node1.extents.y + node1.extents.z);
		rb = node2.extents.x * fabs(R1.z) + node2.extents.z * fabs(R1.x) + OBB_ROTATION_MATRIX_EPSILON * (node2.extents.x + node2.extents.z);
		t = fabs(T.z * R2.y - T.y * R3.y);
		
		if(t > ra + rb)
			return false;
	}
	
	//L = A0 x B2
	{
		float ra, rb, t;
		ra = node1.extents.y * fabs(R3.z) + node1.extents.z * fabs(R2.z) + OBB_ROTATION_MATRIX_EPSILON * (node1.extents.y + node1.extents.z);
		rb = node2.extents.x * fabs(R1.y) + node2.extents.y * fabs(R1.x) + OBB_ROTATION_MATRIX_EPSILON * (node2.extents.x + node2.extents.y);
		t = fabs(T.z * R2.z - T.y * R3.z);
		
		if(t > ra + rb)
			return false;
	}
	
	//L = A1 x B0
	{
		float ra, rb, t;
		ra = node1.extents.x * fabs(R3.x) + node1.extents.z * fabs(R1.x) + OBB_ROTATION_MATRIX_EPSILON * (node1.extents.x + node1.extents.z);
		rb = node2.extents.y * fabs(R2.z) + node2.extents.z * fabs(R2.y) + OBB_ROTATION_MATRIX_EPSILON * (node2.extents.y + node2.extents.z);
		t = fabs(T.x * R3.x - T.z * R1.x);
		
		if(t > ra + rb)
			return false;
	}
	
	//L = A1 x B1
	{
		float ra, rb, t;
		ra = node1.extents.x * fabs(R3.y) + node1.extents.z * fabs(R1.y) + OBB_ROTATION_MATRIX_EPSILON * (node1.extents.x + node1.extents.z);
		rb = node2.extents.x * fabs(R2.z) + node2.extents.z * fabs(R2.x) + OBB_ROTATION_MATRIX_EPSILON * (node2.extents.y + node2.extents.z);
		t = fabs(T.x * R3.y - T.z * R1.y);
		
		if(t > ra + rb)
			return false;
	}
	
	//L = A1 x B2
	{
		float ra, rb, t;
		ra = node1.extents.x * fabs(R3.z) + node1.extents.z * fabs(R1.z) + OBB_ROTATION_MATRIX_EPSILON * (node1.extents.x + node1.extents.z);
		rb = node2.extents.x * fabs(R2.y) + node2.extents.y * fabs(R2.x) + OBB_ROTATION_MATRIX_EPSILON * (node2.extents.x + node2.extents.y);
		t = fabs(T.x * R3.z - T.z * R1.z);
		
		if(t > ra + rb)
			return false;
	}
	
	//L = A2 x B0
	{
		float ra, rb, t;
		ra = node1.extents.x * fabs(R2.x) + node1.extents.y * fabs(R1.x) + OBB_ROTATION_MATRIX_EPSILON * (node1.extents.x + node1.extents.y);
		rb = node2.extents.y * fabs(R3.z) + node2.extents.z * fabs(R3.y) + OBB_ROTATION_MATRIX_EPSILON * (node2.extents.y + node2.extents.z);
		t = fabs(T.y * R1.x - T.x * R2.x);
		
		if(t > ra + rb)
			return false;
	}
	
	//L = A2 x B1
	{
		float ra, rb, t;
		ra = node1.extents.x * fabs(R2.y) + node1.extents.y * fabs(R1.y) + OBB_ROTATION_MATRIX_EPSILON * (node1.extents.x + node1.extents.y);
		rb = node2.extents.x * fabs(R3.z) + node2.extents.z * fabs(R3.x) + OBB_ROTATION_MATRIX_EPSILON * (node2.extents.x + node2.extents.z);
		t = fabs(T.y * R1.y - T.x * R2.y);
		
		if(t > ra + rb)
			return false;
	}
	
	//L = A2 x B2
	{
		float ra, rb, t;
		ra = node1.extents.x * fabs(R2.z) + node1.extents.y * fabs(R1.z) + OBB_ROTATION_MATRIX_EPSILON * (node1.extents.x + node1.extents.y);
		rb = node2.extents.x * fabs(R3.y) + node2.extents.y * fabs(R3.x) + OBB_ROTATION_MATRIX_EPSILON * (node2.extents.x + node2.extents.y);
		t = fabs(T.y * R1.z - T.x * R2.z);
		
		if(t > ra + rb)
			return false;
	}
	
	// no separating axis found:
	// the two boxes overlap
	
	return true;
}

#endif