#include "Light.h"
#include <cmath>

/* Constructor. Implemented for you. */
PointLight::PointLight(const Vector3f & position, const Vector3f & intensity)
    : position(position), intensity(intensity)
{
}

// Compute the contribution of light at point p using the
// inverse square law formula
Vector3f PointLight::computeLightContribution(const Vector3f& p)
{
	/***********************************************
     * Implemented                                 *
     ***********************************************/
	// Find the distance to the light
	float distanceSquared = powf(position.x-p.x, 2) + powf(position.y-p.y, 2) + powf(position.z-p.z, 2);
	return {intensity.r/distanceSquared, intensity.g/distanceSquared, intensity.b/distanceSquared};
}
