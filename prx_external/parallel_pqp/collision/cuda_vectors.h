#ifndef __CUDA_VECTORS_H_
#define __CUDA_VECTORS_H_

#include <vector_types.h>
#include <vector_functions.h>

// Vector operations on float2
#define f2_dot(v1, v2) ((v1).x * (v2).x + (v1).y * (v2).y)
#define f2_sub(result, v1, v2) { result.x = (v1).x - (v2).x; result.y = (v1).y - (v2).y;}
#define f2_add(result, v1, v2) { result.x = (v1).x + (v2).x; result.y = (v1).y + (v2).y;}
#define f2_mul(result, v1, v2) { result.x = (v1).x + (v2).x; result.y = (v1).y * (v2).y;}

// Vector operations on float[3]
#define f3_assign(v2, v1) { (v2)[0] = (v1)[0]; (v2)[1] = (v1)[1]; (v2)[2] = (v1)[2]; }
#define f3_dot(v1, v2) ((v1)[0] * (v2)[0] + (v1)[1] * (v2)[1] + (v1)[2] * (v2)[2])
#define f3_sub(result, v1, v2) { result[0] = (v1)[0] - (v2)[0]; result[1] = (v1)[1] - (v2)[1]; result[2] = (v1)[2] - (v2)[2]; }
#define f3_add(result, v1, v2) { result[0] = (v1)[0] + (v2)[0]; result[1] = (v1)[1] + (v2)[1]; result[2] = (v1)[2] + (v2)[2]; }
#define f3_mul(result, v1, v2) { result[0] = (v1)[0] + (v2)[0]; result[1] = (v1)[1] * (v2)[1]; result[2] = (v1)[2] * (v2)[2]; }
#define f3_cross(result, v1, v2) { result[0] = (v1)[1]*(v2)[2] - (v1)[2]*(v2)[1]; \
	result[1] = (v1)[2]*(v2)[0] - (v1)[0]*(v2)[2]; \
	result[2] = (v1)[0]*(v2)[1] - (v1)[1]*(v2)[0]; }

// Vector operations on float3, make_float3() versions
#define f3v_assign(v2, v1) do { v2 = make_float3((v1).x, (v1).y, (v1).z); } while(0)
#define f3v_add(v1, v2) make_float3((v1).x+(v2).x, (v1).y+(v2).y, (v1).z+(v2).z)
#define f3v_mul(v1, v2) make_float3((v1).x*(v2).x, (v1).y*(v2).y, (v1).z*(v2).z)
#define f3v_sub(v1, v2) make_float3((v1).x-(v2).x, (v1).y-(v2).y, (v1).z-(v2).z)
#define f3v_dot(v1, v2) ((v1).x * (v2).x + (v1).y * (v2).y + (v1).z * (v2).z)

#define f3v_add1(v1, f) make_float3((v1).x+f, (v1).y+f, (v1).z+f)
#define f3v_mul1(v1, f) make_float3((v1).x*f, (v1).y*f, (v1).z*f)
#define f3v_sub1(v1, f) make_float3((v1).x-f, (v1).y-f, (v1).z-f)

#define f3v_cross(v1, v2) make_float3((v1).y*(v2).z - (v1).z*(v2).y,\
	(v1).z*(v2).x - (v1).x*(v2).z,\
	(v1).x*(v2).y - (v1).y*(v2).x)

#define f3v_len(v) ((v).x * (v).x + (v).y * (v).y + (v).z * (v).z)

// Vector operations on float3, normal versions
#define f3s_assign(v2, v1) { (v2).x = (v1).x; (v2).y = (v1).y; (v2).z = (v1).z; }
#define f3s_dot(v1, v2) ((v1).x * (v2).x + (v1).y * (v2).y + (v1).z * (v2).z)
#define f3s_sub(result, v1, v2) { result.x = (v1).x - (v2).x; result.y = (v1).y - (v2).y; result.z = (v1).z - (v2).z; }
#define f3s_add(result, v1, v2) { result.x = (v1).x + (v2).x; result.y = (v1).y + (v2).y; result.z = (v1).z + (v2).z; }
#define f3s_mul(result, v1, v2) { result.x = (v1).x * (v2).x; result.y = (v1).y * (v2).y; result.z = (v1).z * (v2).z; }
#define f3s_cross(result, v1, v2) { result.x = (v1).y*(v2).z - (v1).z*(v2).y; result.y = (v1).z*(v2).x - (v1).x*(v2).z; result.z = (v1).x*(v2).y - (v1).y*(v2).x; }

#define f3v_mulSymM3x3(matrix,vec) make_float3(matrix[0]*vec.x + matrix[1]*vec.y + matrix[2]*vec.z,\
	matrix[1]*vec.x + matrix[3]*vec.y + matrix[4]*vec.z,\
	matrix[2]*vec.x + matrix[4]*vec.y + matrix[5]*vec.z)

#define f3v_mulM3x3(matrix, vec) make_float3(matrix[0]*vec.x + matrix[1]*vec.y + matrix[2]*vec.z,\
	matrix[3]*vec.x + matrix[4]*vec.y + matrix[5]*vec.z,\
	matrix[6]*vec.x + matrix[7]*vec.y + matrix[8]*vec.z)

#define f3v_mulM3x3T(matrix, vec) make_float3(matrix[0]*vec.x + matrix[3]*vec.y + matrix[6]*vec.z,\
	matrix[1]*vec.x + matrix[4]*vec.y + matrix[7]*vec.z,\
	matrix[2]*vec.x + matrix[5]*vec.y + matrix[8]*vec.z)

#define MT3x3_mulM(m1, m2) {m1[0]*m2[0]+m1[3]*m2[3]+m1[6]*m2[6], m1[0]*m2[1]+m1[3]*m2[4]+m1[6]*m2[7], m1[0]*m2[2]+m1[3]*m2[5]+m1[6]*m2[8],\
	m1[1]*m2[0]+m1[4]*m2[3]+m1[7]*m2[6], m1[1]*m2[1]+m1[4]*m2[4]+m1[7]*m2[7], m1[1]*m2[2]+m1[4]*m2[5]+m1[7]*m2[8],\
	m1[2]*m2[0]+m1[5]*m2[3]+m1[8]*m2[6], m1[2]*m2[1]+m1[5]*m2[4]+m1[8]*m2[7], m1[2]*m2[2]+m1[5]*m2[5]+m1[8]*m2[8]}

#define M3x3_mulM(m1, m2) {m1[0]*m2[0]+m1[1]*m2[3]+m1[2]*m2[6], m1[0]*m2[1]+m1[1]*m2[4]+m1[2]*m2[7], m1[0]*m2[2]+m1[1]*m2[5]+m1[2]*m2[8],\
	m1[3]*m2[0]+m1[4]*m2[3]+m1[5]*m2[6], m1[3]*m2[1]+m1[4]*m2[4]+m1[5]*m2[7], m1[3]*m2[2]+m1[4]*m2[5]+m1[5]*m2[8],\
	m1[6]*m2[0]+m1[7]*m2[3]+m1[8]*m2[6], m1[6]*m2[1]+m1[7]*m2[4]+m1[8]*m2[7], m1[6]*m2[2]+m1[7]*m2[5]+m1[8]*m2[8]}


#endif
