/*
 * quaternions.c
 *
 *  Created on: Feb 21, 2019
 *      Author: marcus
 */

#include "quaternions.h"

static float copysign(float x,float y){
	  if ((x < 0 && y > 0) || (x > 0 && y < 0))
	    return -x;
	return x;
}



quaternion_t QUAT_Add(const quaternion_t* q,const quaternion_t* p){
	quaternion_t r;
	r.w = q->w + p->w;
	r.v = VEC_Add(&q->v, &p->v);

	return r;
}

quaternion_t QUAT_Subtract(const quaternion_t* q,const quaternion_t* p){
	quaternion_t r;
	r.w = q->w - p->w;
	r.v = VEC_Subtract(&q->v, &p->v);

	return r;
}

quaternion_t QUAT_Scale(const quaternion_t* q,float k){
	quaternion_t r;
	r.w = q->w*k;
	r.v = VEC_Scale(&q->v, k);

	return r;
}

quaternion_t QUAT_Conjugate(const quaternion_t * q){
	quaternion_t r;
	r.w = q->w;
	r.v = VEC_Scale(&q->v, -1.0f);

	return r;
}
//q*p
quaternion_t QUAT_Multiply(const quaternion_t * q,const quaternion_t * p){
	quaternion_t r;
	r.w = q->w*p->w - VEC_DotProduct(&q->v, &p->v);
	r.v = VEC_CrossProduct(&q->v, &p->v);

	vec3_t tmp1 = VEC_Scale(&p->v, q->w);
	vec3_t tmp2 = VEC_Scale(&q->v, p->w);
	r.v = VEC_Add(&r.v, &tmp1);
	r.v = VEC_Add(&r.v, &tmp2);

	return r;
}

float QUAT_Norm(const quaternion_t * q){
	float d = VEC_DotProduct(&q->v, &q->v);
	d += q->w*q->w;
	return sqrtf(d);
}

quaternion_t QUAT_Normalize(const quaternion_t * q){
	quaternion_t r;
	float norm = QUAT_Norm(q);
	r.w = q->w / norm;
	r.v = VEC_Scale(&q->v, 1.0 / norm);

	return r;
}

quaternion_t QUAT_Inverse(const quaternion_t * q){
	quaternion_t r;
	float d = QUAT_Norm(q);
	d = d*d;

	r = QUAT_Conjugate(q);

	return QUAT_Scale(&r, 1.0 /d);
}

// r = qpq^-1 = qpq* || Body2Nav
vec3_t QUAT_RotatePoint(const quaternion_t * q,const vec3_t* point){
	quaternion_t p,r;

	p.w = 0;
	p.v = *point;

	quaternion_t q_inv = QUAT_Inverse(q);
	r = QUAT_Multiply(q, &p);
	r = QUAT_Multiply(&r, &q_inv);

	return r.v;
}

// r = q^-1pq = q*pq || Nav2Body
vec3_t QUAT_RotateFrame(const quaternion_t * q,const vec3_t* point){
	quaternion_t p,r;

//	quaternion_t q_conj = QUAT_Conjugate(q);
	p.w = 0;
	p.v = *point;

//	quaternion_t q_inv = QUAT_Inverse(&q_conj);
//	r = QUAT_Multiply(&q_conj, &p);
//	r = QUAT_Multiply(&r, &q_inv);

	quaternion_t q_inv = QUAT_Inverse(q);
	r = QUAT_Multiply(&q_inv, &p);
	r = QUAT_Multiply(&r, q);



	return r.v;
}
vec3_t QUAT_RotatePointAxis (const vec3_t* axis, double angle, const vec3_t * point)
{
	quaternion_t q;
    q.w = cos (angle/2.0);
    q.v = VEC_Scale(axis, sin(angle/2.0));

    return QUAT_RotatePoint(&q, point);

}


ang3_t QUAT_toEuler(const quaternion_t* q){
	ang3_t angles;
	// roll (x-axis rotation)
	float sinr_cosp = +2.0 * (q->w * q->v.x + q->v.y* q->v.z);
	float cosr_cosp = +1.0 - 2.0 * (q->v.x * q->v.x + q->v.y * q->v.y);
	angles.roll = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	float sinp = +2.0 * (q->w * q->v.y - q->v.z * q->v.x);
	if (fabs(sinp) >= 1)
		angles.pitch = copysign(PI / 2, sinp); // use 90 degrees if out of range
	else
		angles.pitch = asin(sinp);

//	 yaw (z-axis rotation)
	float siny_cosp = +2.0 * (q->w * q->v.z + q->v.x * q->v.y);
	float cosy_cosp = +1.0 - 2.0 * (q->v.y * q->v.y + q->v.z * q->v.z);
	angles.yaw = atan2(siny_cosp, cosy_cosp);
	return angles;
}








