#include <cmath>
#include "VectorOperations.h"

float VectorOperations::dotProduct(const Vector3f &v1, const Vector3f &v2) {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

Vector3f VectorOperations::crossProduct(const Vector3f &v1, const Vector3f &v2) {
    Vector3f result;
    result.x = (v1.y*v2.z)-(v1.z*v2.y);
    result.y = (v1.z*v2.x)-(v1.x*v2.z);
    result.z = (v1.x*v2.y)-(v1.y*v2.x);
    return result;
}

Vector3f VectorOperations::vectorSum(const Vector3f &v1, const Vector3f &v2) {
    return {v1.x + v2.x, v1.y + v2.y, v1.z + v2.z};
}

Vector3f VectorOperations::vectorSubtract(const Vector3f &v1, const Vector3f &v2) {
    return {v1.x - v2.x, v1.y - v2.y, v1.z - v2.z};
}

Vector3f VectorOperations::vectorWithConstantMult(const Vector3f &v, float k) {
    return {k*v.x, k*v.y, k*v.z};
}

float VectorOperations::magnitude(const Vector3f &v) {
    return sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
}

Vector3f VectorOperations::normalize(const Vector3f &v) {
    return vectorWithConstantMult(v, 1/magnitude(v));
}