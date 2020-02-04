#ifndef CENG477_HW1_VECTOROPERATIONS_H
#define CENG477_HW1_VECTOROPERATIONS_H

#include "defs.h"

namespace VectorOperations {
    float dotProduct(const Vector3f &v1, const Vector3f &v2);
    Vector3f crossProduct(const Vector3f &v1, const Vector3f &v2);
    Vector3f vectorSum(const Vector3f &v1, const Vector3f &v2);
    Vector3f vectorSubtract(const Vector3f &v1, const Vector3f &v2);
    Vector3f vectorWithConstantMult(const Vector3f &v, float k);
    float magnitude(const Vector3f &v);
    Vector3f normalize(const Vector3f &v);
}

#endif //CENG477_HW1_VECTOROPERATIONS_H
