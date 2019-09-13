/*
 * quaternions.h
 *
 *  Created on: Feb 21, 2019
 *      Author: marcus
 */

#ifndef QUATERNIONS_H_
#define QUATERNIONS_H_

#include <vector3.h>
#include <stdint.h>

#ifndef ARM_MATH_CM4
#define ARM_MATH_CM4
#endif

#ifndef __FPU_PRESENT
#define __FPU_PRESENT 1
#endif

#include "arm_math.h"

#define QUAT_Body2Nav QUAT_RotatePoint
#define QUAT_Nav2Body QUAT_RotateFrame

typedef struct _Quaternion {
	float w; //real
	vec3_t v;
}quaternion_t;

typedef struct _IQuaternion {
	int w; //real
	Ivec3_t v;
}Iquaternion_t;


vec3_t QUAT_RotatePoint(const quaternion_t * q,const vec3_t* point); //Body2Nav
vec3_t QUAT_RotateFrame(const quaternion_t * q,const vec3_t* point); //Nav2Bod
vec3_t QUAT_RotatePointAxis (const vec3_t* axis, double angle, const vec3_t * point);
quaternion_t QUAT_Multiply(const quaternion_t * q,const quaternion_t * p);
quaternion_t QUAT_Scale(const quaternion_t* q,float k);
quaternion_t QUAT_Add(const quaternion_t* q,const quaternion_t* p);
quaternion_t QUAT_Subtract(const quaternion_t* q,const quaternion_t* p);
quaternion_t QUAT_Normalize(const quaternion_t * q);
ang3_t QUAT_toEuler(const quaternion_t* q);







#endif /* QUATERNIONS_H_ */
