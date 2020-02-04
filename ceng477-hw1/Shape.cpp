#include "Shape.h"
#include "Scene.h"
#include <cmath>
#include "VectorOperations.h"

Shape::Shape(void)
{
}

Shape::Shape(int id, int matIndex)
: id(id), matIndex(matIndex)
{
}

Sphere::Sphere(void)
{}


/* Constructor for sphere. You will implement this. */
Sphere::Sphere(int id, int matIndex, int cIndex, float R, vector<Vector3f> *vertices)
        : Shape(id, matIndex), cIndex(cIndex-1), R(R)
{
    /***********************************************
     * Implemented                                 *
     ***********************************************/
}

/* Sphere-ray intersection routine. You will implement this.
Note that ReturnVal structure should hold the information related to the intersection point, e.g., coordinate of that point, normal at that point etc.
You should to declare the variables in ReturnVal structure you think you will need. It is in defs.h file. */
ReturnVal Sphere::intersect(const Ray & ray) const
{
    /***********************************************
     * Implemented                                 *
     ***********************************************/
    using namespace VectorOperations;

    Vector3f o = ray.origin;
    Vector3f d = ray.direction;
    Vector3f c = pScene->vertices[cIndex];

    float delta = powf(dotProduct(d, vectorSubtract(o, c)), 2) - dotProduct(d, d) * (dotProduct(vectorSubtract(o, c), vectorSubtract(o, c)) - R*R);
    if (delta < 0) {
        return {false, {0,0,0}, {0,0,0},0};
    }
    else {
        float t1 = (-dotProduct(d, vectorSubtract(o, c)) + sqrtf(delta)) / dotProduct(d, d);
        float t2 = (-dotProduct(d, vectorSubtract(o, c)) - sqrtf(delta)) / dotProduct(d, d);

        float big_t = std::max(t1, t2);
        float small_t = std::min(t1, t2);

        if (big_t < 0) { // There are no positive t's
            return {false, {0,0,0}, {0,0,0},0};
        }
        else if (small_t < 0) {
            // Return big_t, it is the only positive t
            Vector3f intersectPoint = {o.x + big_t*d.x, o.y + big_t*d.y, o.z + big_t*d.z};
            Vector3f normal = vectorWithConstantMult(vectorSubtract(intersectPoint, c),1/R); // p-c/R
            return {true, intersectPoint, normal, big_t};
        }
        else { // Both t's are positive, return the small t
            Vector3f intersectPoint = {o.x + small_t*d.x, o.y + small_t*d.y, o.z + small_t*d.z};
            Vector3f normal = vectorWithConstantMult(vectorSubtract(intersectPoint, c),1/R); // p-c/R
            return {true, intersectPoint, normal, small_t};
        }
    }
}

Triangle::Triangle(void)
{}

/* Constructor for triangle. You will implement this. */
Triangle::Triangle(int id, int matIndex, int p1Index, int p2Index, int p3Index, vector<Vector3f> *vertices)
        : Shape(id, matIndex),
        p1(vertices->at(p1Index-1)),
        p2(vertices->at(p2Index-1)),
        p3(vertices->at(p3Index-1))
{
    /***********************************************
     * Implemented                                 *
     ***********************************************/
}

/* Triangle-ray intersection routine. You will implement this.
Note that ReturnVal structure should hold the information related to the intersection point, e.g., coordinate of that point, normal at that point etc.
You should to declare the variables in ReturnVal structure you think you will need. It is in defs.h file. */
ReturnVal Triangle::intersect(const Ray & ray) const
{
    /***********************************************
     * Implemented                                 *
     ***********************************************/
    using namespace VectorOperations;

    Vector3f origin = ray.origin;
    Vector3f direction = ray.direction;

    // Find the normal
    Vector3f normal = crossProduct(vectorSubtract(p2,p1),vectorSubtract(p3,p1));
    float t=0;

    if(dotProduct(direction, normal) == 0)
        return {false,{0,0,0},{0,0,0},0};

    t = (dotProduct(p1,normal)-dotProduct(origin,normal))/(dotProduct(direction,normal));

    if (t < 0) { // Do not return any intersection point behind the camera
        return {false,{0,0,0},{0,0,0},0};
    }

    // o+td now is on the triangle plane, but is it in the triangle ?
    Vector3f point = vectorSum(origin,vectorWithConstantMult(direction,t)); // o + td
    float ab = dotProduct(crossProduct(vectorSubtract(point,p1),vectorSubtract(p3,p1)),crossProduct(vectorSubtract(p2,p1),vectorSubtract(p3,p1))); // ((p-b)x(a-b)).((c-b)x(a-b))
    float bc = dotProduct(crossProduct(vectorSubtract(point,p2),vectorSubtract(p1,p2)),crossProduct(vectorSubtract(p3,p2),vectorSubtract(p1,p2)));
    float ac = dotProduct(crossProduct(vectorSubtract(point,p3),vectorSubtract(p2,p3)),crossProduct(vectorSubtract(p1,p3),vectorSubtract(p2,p3)));

    if (ab >= 0 && bc >= 0 && ac >= 0){
        return {true,point,normal,t};
    }
    else {
        return {false,{0,0,0},{0,0,0},0};
    }
}

Mesh::Mesh()
{}

/* Constructor for mesh. You will implement this. */
Mesh::Mesh(int id, int matIndex, const vector<Triangle>& faces, vector<int> *pIndices, vector<Vector3f> *vertices)
        : Shape(id, matIndex)
{
    /***********************************************
     * Implemented                                 *
     ***********************************************/
    this->faces.reserve(faces.size());
    this->faces = faces;
}

/* Mesh-ray intersection routine. You will implement this.
Note that ReturnVal structure should hold the information related to the intersection point, e.g., coordinate of that point, normal at that point etc.
You should to declare the variables in ReturnVal structure you think you will need. It is in defs.h file. */
ReturnVal Mesh::intersect(const Ray & ray) const
{
    /***********************************************
     * Implemented                                 *
     ***********************************************/
    // Check all faces, find smaller t; if none intersects return false.
    ReturnVal result={false,{0,0,0},{0,0,0},MAXFLOAT};

    for (const auto & face : faces) {
        ReturnVal returnVal = face.intersect(ray);
        if(returnVal.intersect && returnVal.t<result.t){
            result = returnVal;
        }
    }
    return result;
}
