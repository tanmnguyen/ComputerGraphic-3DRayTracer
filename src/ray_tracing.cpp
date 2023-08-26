#include "ray_tracing.h"
// Suppress warnings in third-party code.
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/geometric.hpp>
#include <glm/gtx/component_wise.hpp>
#include <glm/vector_relational.hpp>
DISABLE_WARNINGS_POP()
#include <cmath>
#include <iostream>
#include <limits>
#include <math.h>
#include <algorithm>
using namespace std;
static constexpr double EPSILON = 1e-6f;
bool equalFloats(float f1, float f2)
{
    return std::abs(f1 - f2) < EPSILON;
}

double computeArea(glm::vec3 a, glm::vec3 b, glm::vec3 c) {
    glm::vec3 u = b - a;
    glm::vec3 v = c - a;
    return glm::length(glm::cross(u, v));
}
bool pointInTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& n, const glm::vec3& p)
{
    float dot1 = glm::dot((glm::cross((v1 - v0), (p - v0))), n);
    float dot2 = glm::dot((glm::cross((v2 - v1), (p - v1))), n);
    float dot3 = glm::dot((glm::cross((v0 - v2), (p - v2))), n);
    if (dot1 >= 0 && dot2 >= 0 && dot3 >= 0) {
        return true;
    }
    return false;
}

bool intersectRayWithPlane(const Plane& plane, Ray& ray)
{
    if (glm::dot(ray.direction, plane.normal) == 0) {
        return false;
    }
    float t = (plane.D - glm::dot(ray.origin, plane.normal)) / glm::dot(ray.direction, plane.normal);
    if (t <= 0) {
        return false;
    }
    glm::vec3 p = ray.origin + t * ray.direction;
    glm::vec3 orig = ray.origin + ray.t * ray.direction;
    if (glm::length(p - ray.origin) < glm::length(orig - ray.origin)) {
        ray.t = t;
        return true;
    }

    return false;
}

Plane trianglePlane(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2)
{
    Plane plane;
    plane.normal = glm::cross((v1 - v0), (v2 - v0));
    plane.normal = plane.normal / glm::length(plane.normal);
    plane.D = glm::dot(plane.normal, v0);
    return plane;
}

/// Input: the three vertices of the triangle
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false


/// Input: the three vertices of the triangle
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, Ray& ray, HitInfo& hitInfo)
{
    float copiedT = ray.t;
    Plane plane = trianglePlane(v0, v1, v2);
    
    if (intersectRayWithPlane(plane, ray)) {

        glm::vec3 point = ray.origin + ray.t * ray.direction;

        if (pointInTriangle(v0, v1, v2, plane.normal, point)) {
            hitInfo.normal = plane.normal;
            hitInfo.toBeInterpolated = true;
            return true;
        }
    }

    // roll back
    ray.t = copiedT;
    return false;
}

/// Input: a sphere with the following attributes: sphere.radius, sphere.center
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithShape(const Sphere& sphere, Ray& ray, HitInfo& hitInfo)
{
    glm::vec3 d = ray.direction;
    glm::vec3 o = ray.origin;
    glm::vec3 c = sphere.center;


    float r = sphere.radius;
    float A = d.x * d.x + d.y * d.y + d.z * d.z;
    float B = 2 * ((o.x - c.x) * d.x + (o.y - c.y) * d.y + (o.z - c.z) * d.z);
    float C = (o.x - c.x) * (o.x - c.x) + (o.y - c.y) * (o.y - c.y) + (o.z - c.z) * (o.z - c.z) - r * r;

    float delta = B * B - 4 * A * C;

    if (delta < 0.0f) {
        return false;
    }

    float x[2];
    x[0] = (-B - sqrt(delta)) / (2 * A);
    x[1] = (-B + sqrt(delta)) / (2 * A);


    if (x[0] <= 0 && x[1] <= 0) return false;
    if (x[0] <= 0) x[0] = x[1];
    if (x[1] <= 0) x[1] = x[0];

    if (min(x[0], x[1]) < ray.t) {
        ray.t = min(x[0], x[1]);
        hitInfo.toBeInterpolated = false;
        hitInfo.material = sphere.material;
        glm::vec3 p = ray.origin + ray.t * ray.direction;
        hitInfo.normal = p - sphere.center;
        hitInfo.normal = glm::normalize(hitInfo.normal);
        return true;
    }

    return false;
}


/// Input: an axis-aligned bounding box with the following parameters: minimum coordinates box.lower and maximum coordinates box.upper
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithShape(const AxisAlignedBox& box, Ray& ray)
{

    float txmin = (box.lower.x - ray.origin.x) / ray.direction.x;
    float tymin = (box.lower.y - ray.origin.y) / ray.direction.y;
    float tzmin = (box.lower.z - ray.origin.z) / ray.direction.z;

    float txmax = (box.upper.x - ray.origin.x) / ray.direction.x;
    float tymax = (box.upper.y - ray.origin.y) / ray.direction.y;
    float tzmax = (box.upper.z - ray.origin.z) / ray.direction.z;

    float tinx = glm::min(txmin, txmax);
    float toutx = glm::max(txmin, txmax);

    float tiny = glm::min(tymin, tymax);
    float touty = glm::max(tymin, tymax);

    float tinz = glm::min(tzmin, tzmax);
    float toutz = glm::max(tzmin, tzmax);

    float tin = glm::max(glm::max(tinx, tiny), tinz);
    float tout = glm::min(glm::min(toutx, touty), toutz);

    if (tin > tout || tout < 0) {
        return false;
    }


    glm::vec3 p = ray.origin + tin * ray.direction;
    glm::vec3 orig = ray.origin + ray.t * ray.direction;
    if (glm::length(p - ray.origin) < glm::length(orig - ray.origin)) {
        ray.t = tin;
        return true;
    }

    return true;
}
