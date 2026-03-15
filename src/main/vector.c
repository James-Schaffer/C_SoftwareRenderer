// Created by James Schaffer on 29/01/2026.

// ========== VECTOR FUNCTIONS ==========

#include "vector.h"

int clampi(int d, int min, int max) {
	const int t = d < min ? min : d;
	return t > max ? max : t;
}
double clampd(double d, double min, double max) {
	const double t = d < min ? min : d;
	return t > max ? max : t;
}
v3 v3Scale(const v3 a, double s) {
	v3 r;
	r.x = a.x * s;
	r.y = a.y * s;
	r.z = a.z * s;
	return r;
}
v3 v3Add(const v3 a, const v3 b) {
	v3 r;
	r.x = a.x + b.x;
	r.y = a.y + b.y;
	r.z = a.z + b.z;
	return r;
}
v3 v3Sub(const v3 a, const v3 b) {
	v3 r;
	r.x = a.x - b.x;
	r.y = a.y - b.y;
	r.z = a.z - b.z;
	return r;
}
double dotProduct(const v3 a, const v3 b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}
v3 crossProduct(const v3 a, const v3 b) {
	v3 r;
	r.x = a.y*b.z - a.z*b.y;
	r.y = a.z*b.x - a.x*b.z;
	r.z = a.x*b.y - a.y*b.x;
	return r;
}
double v3Len(const v3 a) {
	return sqrt(dotProduct(a, a));
}
v3 normalize(const v3 v) {
	double len = v3Len(v);
	if (len == 0.0) return v;
	return v3Scale(v, 1.0 / len);
}

v2 v2Sub(const v2 a, const v2 b) {
	v2 r;
	r.x = a.x - b.x;
	r.y = a.y - b.y;
	return r;
}
v2 scalev2(const v2 a, const double s) {
	v2 r;
	r.x = a.x * s;
	r.y = a.y * s;
	return r;
}
v2 normalizev2(const v2 v) {
	double len = sqrt( (v.x*v.x) + (v.y*v.y) );
	if (len == 0.0) return v;
	return scalev2(v, 1.0 / len);
}

float crossProduct_v2(const v2 a, const v2 b) {
	return (float) ((a.x * b.y) - (a.y * b.x));
}

// ========== TRANSFORM FUNCTIONS ==========

v3 transformV3(const v3* v, const Transform* t) {
	v3 ret = *v;

	// Scale
	ret.x *= t->scale.x;
	ret.y *= t->scale.y;
	ret.z *= t->scale.z;

	// Rotate
	double x = ret.x;
	double y = ret.y;
	double z = ret.z;

	double cx = cos(t->rotation.x);
	double sx = sin(t->rotation.x);
	double cy = cos(t->rotation.y);
	double sy = sin(t->rotation.y);
	double cz = cos(t->rotation.z);
	double sz = sin(t->rotation.z);

	ret.x = (x*(cy*cz)) + (y*((sx*sy*cz)-(cx*sz))) + (z*((cx*sy*cz)+(sx*sz)));
	ret.y = (x*(cy*sz)) + (y*((sx*sy*sz)+(cx*cz))) + (z*((cx*sy*sz)-(sx*cz)));
	ret.z = (x*(-sy)) + (y*(sx*cy)) + (z*(cx*cy));

	// Transform position
	ret.x += t->position.x;
	ret.y += t->position.y;
	ret.z += t->position.z;

	return ret;
}