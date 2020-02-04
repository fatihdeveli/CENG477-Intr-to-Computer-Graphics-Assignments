#include "Camera.h"
#include "VectorOperations.h"
#include <cstring>

Camera::Camera(int id,                      // Id of the camera
               const char* imageName,       // Name of the output PPM file 
               const Vector3f& pos,         // Camera position
               const Vector3f& gaze,        // Camera gaze direction
               const Vector3f& up,          // Camera up direction
               const ImagePlane& imgPlane)  // Image plane parameters
: id(id), imageName(), imgPlane(imgPlane), pos(pos), gaze(gaze), up(up)
{
	/***********************************************
     * Implemented                                 *
     ***********************************************/
	 strcpy(this->imageName, imageName);
}

/* Takes coordinate of an image pixel as row and col, and
 * returns the ray going through that pixel. 
 */
Ray Camera::getPrimaryRay(int col, int row) const
{
    /***********************************************
     * Implemented                                 *
     ***********************************************/
	using namespace VectorOperations;
	// m: center of the image, e: camera position, -w: gaze
	// m = e + -w*distance

	// q: top-left corner coordinates, v: up vector
	// q = m + l*u + t*v

	// s = q + s_u*u - s_v*v
	// i,j: pixel coordinates, n_x, n_y: image width-height
	// s_u: (i+0.5) * (r-l)/n_x
	// s_v: (j+0.5) * (t-b)/n_y

	// Final ray equation
	// r(t) = e + (s-e)*t = e + dt

	// Image center
	Vector3f m = vectorSum(pos, vectorWithConstantMult(gaze, imgPlane.distance));

	// Calculate the u vector by taking the cross product of gaze and up vector.
	Vector3f u = crossProduct(gaze, up);

	// Calculate s_u = (i+0.5)(r-l)/n_x
	float coefficient = col + 0.5; // i+0.5
	float rhs = (imgPlane.right - imgPlane.left) / imgPlane.nx; // (r-l)/n_x
	float s_u = coefficient*rhs;

	// Calculate s_v = (j+0.5)(t-b)/n_y
	coefficient = row + 0.5; // j+0.5
	rhs = (imgPlane.top - imgPlane.bottom) / imgPlane.ny; // (t-b)/n_y
	float s_v = coefficient*rhs;

	// Calculate q = m + l*u + t*v
	Vector3f lu = vectorWithConstantMult(u, imgPlane.left);
	Vector3f tv = vectorWithConstantMult(up, imgPlane.top);
	Vector3f lutv = vectorSum(lu, tv); // l*u + t*v
	Vector3f q = vectorSum(m, lutv);

	// Calculate s = q + s_u*u - s_v*v
	Vector3f t1 = vectorWithConstantMult(u, s_u); // s_u*u
	Vector3f t2 = vectorWithConstantMult(up, s_v); // s_v*v
	Vector3f t = vectorSubtract(t1, t2); // s_u*u - s_v*v
	Vector3f s = vectorSum(q, t);

	// Final ray equation
	Vector3f direction = vectorSubtract(s, pos);
	Ray r(pos, direction);
	return r;
}