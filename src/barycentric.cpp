#include "draw.h"

#include <framework/opengl_includes.h>
// Suppress warnings in third-party code.
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#ifdef __APPLE__
#include <OpenGL/GLU.h>
#else
#ifdef WIN32
// Windows.h includes a ton of stuff we don't need, this macro tells it to include less junk.
#define WIN32_LEAN_AND_MEAN
// Disable legacy macro of min/max which breaks completely valid C++ code (std::min/std::max won't work).
#define NOMINMAX
// GLU requires Windows.h on Windows :-(.
#include <Windows.h>
#endif
#include <GL/glu.h>
#endif
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/mat4x4.hpp>
DISABLE_WARNINGS_POP()
#include <algorithm>
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>
#include<iostream>

using namespace std;

// compute area using the length of cross product
float computeArea(glm::vec3 v0, glm::vec3 v1, glm::vec3 v2) {
	glm::vec3 u = v1 - v0;
	glm::vec3 v = v2 - v0;
	return glm::length(glm::cross(u, v));
}

// compute barycentric coordinate
glm::vec3 computeBarycentric(glm::vec3 point, glm::vec3 triangleVertex0, glm::vec3 triangleVertex1, glm::vec3 triangleVertex2) {
	glm::vec3 v0 = triangleVertex0;
	glm::vec3 v1 = triangleVertex1;
	glm::vec3 v2 = triangleVertex2;
	glm::vec3 p = point;

	// compute the corresponding area ratio
	float area = computeArea(v0, v1, v2);
	float alpha = computeArea(v1, v2, p) / area;
	float beta = computeArea(v0, v2, p) / area;
	float gamma = computeArea(v0, v1, p) / area;

	return { alpha, beta, gamma };
}

 //interpolate when 3 vertices have different normals
glm::vec3 interpolateNormal(const Ray& ray, Vertex v0, Vertex v1, Vertex v2, bool isDebug) {

	// normalzing the given normals
	glm::vec3 n0 = glm::normalize(v0.normal);
	glm::vec3 n1 = glm::normalize(v1.normal);
	glm::vec3 n2 = glm::normalize(v2.normal);
	glm::vec3 point = ray.origin + ray.t * ray.direction;

	glm::vec3 bary = computeBarycentric(point, v0.position, v1.position, v2.position);
	glm::vec3 result = glm::normalize( n0* bary.x + n1 * bary.y + n2 * bary.z);

	if (glm::dot(result, ray.direction) > 0) {
		result = -result;
		n0 = -n0;
		n1 = -n1;
		n2 = -n2;
	}
#ifndef NDEBUG
	if (isDebug) {
		drawVector(v0.position, n0, glm::vec3{ 0, 1, 0 });
		drawVector(v1.position, n1, glm::vec3{ 0, 1, 0 });
		drawVector(v2.position, n2, glm::vec3{ 0, 1, 0 });
		drawVector(point, result, glm::vec3{ 1, 1, 0 });
	}
#endif // 

	return result;
}

glm::vec2 interpolateCoord(const Ray& ray, const Vertex v0, const Vertex v1, const Vertex v2) {

	glm::vec3 bary = computeBarycentric(ray.origin + ray.t * ray.direction, v0.position, v1.position, v2.position);
	return bary.x * v0.texCoord + bary.y * v1.texCoord + bary.z * v2.texCoord;

}