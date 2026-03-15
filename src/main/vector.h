// Created by James Schaffer on 28/01/2026.

#ifndef CUBERENDER_VECTORS_H
#define CUBERENDER_VECTORS_H

#include <math.h>

// ========== VECTOR STRUCTS ==========

typedef struct v3 {
	double x;
	double y;
	double z;
} v3;
typedef struct v3i {
	int x;
	int y;
	int z;
} v3i;

typedef struct v2 {
	double x;
	double y;
} v2;
typedef struct v2i {
	int x;
	int y;
} v2i;

// ========== TRANSFORM STRUCT ==========

typedef struct  {
	v3 position;
	v3 rotation;
	v3 scale;
} Transform;

// ========== VECTOR FUNCTIONS ==========

int clampi(int d, int min, int max);
double clampd(double d, double min, double max);

v3 v3Scale(v3 a, double s);
v3 v3Add(v3 a, v3 b);
v3 v3Sub(v3 a, v3 b);

double dotProduct(v3 a, v3 b);
v3 crossProduct(v3 a, v3 b);

double v3Len(v3 a);
v3 normalize(v3 v);

v2 v2Sub(v2 a, v2 b);
v2 scalev2(v2 a, double s);
v2 normalizev2(v2 v);
float crossProduct_v2(v2 a, v2 b);

v3 transformV3(const v3* v, const Transform* t);

#endif //CUBERENDER_VECTORS_H