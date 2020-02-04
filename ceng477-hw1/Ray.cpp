#include "Ray.h"

Ray::Ray()
{
}

Ray::Ray(const Vector3f& origin, const Vector3f& direction)
    : origin(origin), direction(direction)
{
}

/* Takes a parameter t and returns the point accoring to t. t is the parametric variable in the ray equation o+t*d.*/
Vector3f Ray::getPoint(float t) const 
{
	/***********************************************
     * Implemented                                 *
     ***********************************************/
    // r(t) = o + td
    return {origin.x + t*direction.x,
            origin.y + t*direction.y,
            origin.z + t*direction.z};
}

/* Takes a point p and returns the parameter t according to p such that p = o+t*d. */
float Ray::gett(const Vector3f & p) const
{
	/***********************************************
     * Implemented                                 *
     ***********************************************/
    // t = (p-o)/d
    // Do the calculation on only x, unless d.x is 0.
    if (direction.x != 0) {
        return (float) (p.x - origin.x) / direction.x;
    }
    else if (direction.y != 0) { // Try y
        return (float) (p.y - origin.y) / direction.y;
    }
    else { // Try z
        return (float) (p.z - origin.z) / direction.z;
    }
}

