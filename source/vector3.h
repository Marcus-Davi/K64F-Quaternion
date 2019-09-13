/*
 * vector.h
 *
 *  Created on: Feb 21, 2019
 *      Author: marcus
 */

#ifndef VECTOR3_H_
#define VECTOR3_H_

#include <math.h>

/*3D vector operations*/



typedef struct _3Dvec {
	float x;
	float y;
	float z;
}vec3_t;

typedef struct _3DAng {
	float roll; //phi
	float pitch; //theta
	float yaw; //psi
}ang3_t;

typedef struct _3DIvec {
	int x;
	int y;
	int z;
}Ivec3_t;

// Soma vetorial.
static inline vec3_t VEC_Add(const vec3_t* a, const vec3_t* b){
	vec3_t r;
	r.x = a->x + b->x;
	r.y = a->y + b->y;
	r.z = a->z + b->z;
	return r;
}

// Subtracao vetorial.
static inline vec3_t VEC_Subtract(const vec3_t* a,const vec3_t* b){
	vec3_t r;
	r.x = a->x - b->x;
	r.y = a->y - b->y;
	r.z = a->z - b->z;
	return r;
}

// Scaling vetorial.
static inline vec3_t VEC_Scale(const vec3_t* a,float k){
	vec3_t r;
	r.x = k*a->x;
	r.y = k*a->y;
	r.z = k*a->z;
	return r;
};

//Produto Escalar.
static inline float VEC_DotProduct(const vec3_t* a,const vec3_t* b){
	return a->x*b->x + a->y*b->y + a->z*b->z;
}

//Norma
static inline  float VEC_Norm(const vec3_t* a){
	return sqrtf((VEC_DotProduct(a, a))); //cuidado c erro numerico
}

static inline float VEC_Distance(const vec3_t*a, const vec3_t*b){
	return sqrtf((a->x-b->x)*(a->x-b->x) + (a->y-b->y)*(a->y-b->y) + (a->z-b->z)*(a->z-b->z));
}

//Produto Vetorial. k > 0 se "a" estiver a esquerda de "b".
static inline vec3_t VEC_CrossProduct(const vec3_t* a,const vec3_t* b){
	vec3_t r;
	r.x = a->y*b->z - a->z*b->y; //i
	r.y = a->z*b->x - a->x*b->z; //j
	r.z = a->x*b->y - a->y*b->x; //k
	return r;
};

static inline vec3_t V_Normalize(const vec3_t* a){
	float norm = VEC_Norm(a);
	return VEC_Scale(a,1/norm);

};




#endif /* VECTOR3_H_ */
