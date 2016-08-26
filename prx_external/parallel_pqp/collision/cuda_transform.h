#ifndef __CUDA_TRANSFORM_H_
#define __CUDA_TRANSFORM_H_

#include "mathdef.h"
#include "cuda_obb.h"
#include "cuda_vectors.h"

struct GTransform
{
	float3 translate;
	float3 rotate1, rotate2, rotate3;
	
	__device__ __inline__ void set_identity()
	{
		translate = make_float3(0.0f, 0.0f, 0.0f);
		rotate1 = make_float3(1.0f, 0.0f, 0.0f);
		rotate2 = make_float3(0.0f, 1.0f, 0.0f);
		rotate3 = make_float3(0.0f, 0.0f, 1.0f);
	}
	
	__device__ __inline__ bool is_identity() const
	{
		return FLOAT_EQ(rotate1.x, 1.0f) && FLOAT_EQ(rotate1.y, 0.0f) && FLOAT_EQ(rotate1.z, 0.0f)
		       && FLOAT_EQ(rotate2.x, 0.0f) && FLOAT_EQ(rotate2.y, 1.0f) && FLOAT_EQ(rotate2.z, 0.0f)
		       && FLOAT_EQ(rotate3.x, 0.0f) && FLOAT_EQ(rotate3.y, 0.0f) && FLOAT_EQ(rotate3.z, 1.0f)
		       && FLOAT_EQ(translate.x, 0.0f) && FLOAT_EQ(translate.y, 0.0f) && FLOAT_EQ(translate.z, 0.0f);
	}
	
	// concatenate X to the right to the Transform
	__device__ __inline__ void append(const GTransform& X)
	{
		if(is_identity())
		{
			translate = X.translate;
			rotate1 = X.rotate1;
			rotate2 = X.rotate2;
			rotate3 = X.rotate3;
		}
		else if(!X.is_identity())
		{
			translate.x = translate.x + f3v_dot(rotate1, X.translate);
			translate.y = translate.y + f3v_dot(rotate2, X.translate);
			translate.z = translate.z + f3v_dot(rotate3, X.translate);
			
			float3 tmp1 = make_float3(X.rotate1.x, X.rotate2.x, X.rotate3.x);
			float3 tmp2 = make_float3(X.rotate1.y, X.rotate2.y, X.rotate3.y);
			float3 tmp3 = make_float3(X.rotate1.z, X.rotate2.z, X.rotate3.z);
			
			float3 tmp = rotate1;
			rotate1.x = f3v_dot(tmp, tmp1);
			rotate1.y = f3v_dot(tmp, tmp2);
			rotate1.z = f3v_dot(tmp, tmp3);
			
			tmp = rotate2;
			rotate2.x = f3v_dot(tmp, tmp1);
			rotate2.y = f3v_dot(tmp, tmp2);
			rotate2.z = f3v_dot(tmp, tmp3);
			
			tmp = rotate3;
			rotate3.x = f3v_dot(tmp, tmp1);
			rotate3.y = f3v_dot(tmp, tmp2);
			rotate3.z = f3v_dot(tmp, tmp3);
		}
	}
	
	__device__ __inline__ void append_translate(float t, const float3& axis)
	{
		translate.x = translate.x + t * f3v_dot(rotate1, axis);
		translate.y = translate.y + t * f3v_dot(rotate2, axis);
		translate.z = translate.z + t * f3v_dot(rotate3, axis);
	}
	
	__device__ __inline__ void append_rotate(float a, const float3& axis)
	{
		float3 tmp1, tmp2, tmp3;
		{
			float co = cosf(a);
			float si = sinf(a);
			float co1 = 1.0f - co;
			
			tmp1 = make_float3(axis.x * axis.x * co1 + co, axis.x * axis.y * co1 + axis.z * si, axis.z * axis.x * co1 - axis.y * si);
			tmp2 = make_float3(axis.x * axis.y * co1 - axis.z * si, axis.y * axis.y * co1 + co, axis.z * axis.y * co1 + axis.x * si);
			tmp3 = make_float3(axis.x * axis.z * co1 + axis.y * si, axis.z * axis.y * co1 - axis.x * si, axis.z * axis.z * co1 + co);
		}
		
		float3 tmp = rotate1;
		rotate1.x = f3v_dot(tmp, tmp1);
		rotate1.y = f3v_dot(tmp, tmp2);
		rotate1.z = f3v_dot(tmp, tmp3);
		
		tmp = rotate2;
		rotate2.x = f3v_dot(tmp, tmp1);
		rotate2.y = f3v_dot(tmp, tmp2);
		rotate2.z = f3v_dot(tmp, tmp3);
		
		tmp = rotate3;
		rotate3.x = f3v_dot(tmp, tmp1);
		rotate3.y = f3v_dot(tmp, tmp2);
		rotate3.z = f3v_dot(tmp, tmp3);
		
	}
	
	
	// concatenate X to the left to the Transform
	__device__ __inline__ void append_left(const GTransform& X)
	{
		if(is_identity())
		{
			translate = X.translate;
			rotate1 = X.rotate1;
			rotate2 = X.rotate2;
			rotate3 = X.rotate3;
		}
		else if(!X.is_identity())
		{
			{
				float3 tmp;
				tmp.x = X.translate.x + f3v_dot(X.rotate1, translate);
				tmp.y = X.translate.y + f3v_dot(X.rotate2, translate);
				tmp.z = X.translate.z + f3v_dot(X.rotate3, translate);
				translate = tmp;
			}
			{
				float3 tmp1 = make_float3(rotate1.x, rotate2.x, rotate3.x);
				float3 tmp2 = make_float3(rotate1.y, rotate2.y, rotate3.y);
				float3 tmp3 = make_float3(rotate1.z, rotate2.z, rotate3.z);
				
				rotate1.x = f3v_dot(X.rotate1, tmp1);
				rotate1.y = f3v_dot(X.rotate1, tmp2);
				rotate1.z = f3v_dot(X.rotate1, tmp3);
				
				rotate2.x = f3v_dot(X.rotate2, tmp1);
				rotate2.y = f3v_dot(X.rotate2, tmp2);
				rotate2.z = f3v_dot(X.rotate2, tmp3);
				
				rotate3.x = f3v_dot(X.rotate3, tmp1);
				rotate3.y = f3v_dot(X.rotate3, tmp2);
				rotate3.z = f3v_dot(X.rotate3, tmp3);
			}
		}
	}
	
	__device__ __inline__ float3 applyv(const float3& vec) const
	{
		float3 t;
		t.x = f3v_dot(rotate1, vec) + translate.x;
		t.y = f3v_dot(rotate2, vec) + translate.y;
		t.z = f3v_dot(rotate3, vec) + translate.z;
		return t;
	}
	
	template <class BV> __device__ __inline__ void apply(BV &bv)
	{
		// do nothing
	}
	
	__device__ __inline__ void ToSE3(float* s, unsigned int stride)
	{
		translate.x = *s;
		s += stride;
		translate.y = *s;
		s += stride;
		translate.z = *s;
		s += stride;
		
		float c1 = cosf(*s);
		float s1 = sinf(*s);
		s += stride;
		float c2 = cosf(*s);
		float s2 = sinf(*s);
		s += stride;
		float c3 = cosf(*s);
		float s3 = sinf(*s);
		
		rotate1 = make_float3(c2 * c3, c3 * s1 * s2 - c1 * s3, c1 * c3 * s2 + s1 * s3);
		rotate2 = make_float3(c2 * s3, c1 * c3 + s1 * s2 * s3, c1 * s2 * s3 - c3 * s1);
		rotate3 = make_float3(- s2, c2 * s1, c1 * c2);
	}
	
	__device__ __inline__ void ToSE3(float t1, float t2, float t3, float r1, float r2, float r3)
	{
		translate.x = t1;
		translate.y = t2;
		translate.z = t3;
		
		float c1 = cosf(r1);
		float c2 = cosf(r2);
		float c3 = cosf(r3);
		float s1 = sinf(r1);
		float s2 = sinf(r2);
		float s3 = sinf(r3);
		
		rotate1 = make_float3(c2 * c3, c3 * s1 * s2 - c1 * s3, c1 * c3 * s2 + s1 * s3);
		rotate2 = make_float3(c2 * s3, c1 * c3 + s1 * s2 * s3, c1 * s2 * s3 - c3 * s1);
		rotate3 = make_float3(- s2, c2 * s1, c1 * c2);
	}
	
	__device__ __inline__ void ToTrans(float t1, float t2, float t3)
	{
		translate.x = t1;
		translate.y = t2;
		translate.z = t3;
		
		rotate1 = make_float3(1.0f, 0.0f, 0.0f);
		rotate2 = make_float3(0.0f, 1.0f, 0.0f);
		rotate3 = make_float3(0.0f, 0.0f, 1.0f);
	}
};

template <> __device__ void GTransform::apply<AABB>(AABB &bv)
{
    bv.bb_min = f3v_add(bv.bb_min, translate);
    bv.bb_max = f3v_add(bv.bb_max, translate);
}

template <> __device__ void GTransform::apply<OBB>(OBB& bv)
{
    float3 tmp;
    
    tmp.x = f3v_dot(bv.center, rotate1) + translate.x;  ///< tmp is now rotate and translate center
    tmp.y = f3v_dot(bv.center, rotate2) + translate.y;
    tmp.z = f3v_dot(bv.center, rotate3) + translate.z;
    bv.center = tmp;
    
    tmp.x = f3v_dot(bv.axis1, rotate1);	///< tmp is now rotate axes
    tmp.y = f3v_dot(bv.axis1, rotate2);
    tmp.z = f3v_dot(bv.axis1, rotate3);
    bv.axis1 = tmp;
    
    tmp.x = f3v_dot(bv.axis2, rotate1);
    tmp.y = f3v_dot(bv.axis2, rotate2);
    tmp.z = f3v_dot(bv.axis2, rotate3);
    bv.axis2 = tmp;
    
    tmp.x = f3v_dot(bv.axis3, rotate1);
    tmp.y = f3v_dot(bv.axis3, rotate2);
    tmp.z = f3v_dot(bv.axis3, rotate3);
    bv.axis3 = tmp;
}




#endif