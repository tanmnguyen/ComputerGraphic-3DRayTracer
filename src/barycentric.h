#pragma once
#include "scene.h"
#include <framework/mesh.h>
#include <framework/ray.h>
#include <utility> // std::forward
#include<iostream>

using namespace std;

// interpolate when 3 vertices have the same normal
glm::vec3 interpolateNormal(const glm::vec3& point, const glm::vec3& triangleVertex0, const glm::vec3& triangleVertex1, 
	const glm::vec3& triangleVertex2);

// interpolate when 3 vertices have different normals
//glm::vec3 interpolateNormal(const glm::vec3& point, const glm::vec3& triangleVertex0, const glm::vec3& triangleVertex1, const glm::vec3& triangleVertex2,
//	const glm::vec3& normal0, const glm::vec3& normal1, const glm::vec3& normal2);