#pragma once
#include "ray_tracing.h"
#include "scene.h"
#include <array>
#include <span>
#include <iostream>
#include <algorithm>
#include <vector>
#include <math.h>
using namespace std;


struct Node {
    bool isLeaf;
    int level, left, right;
    vector<int> triIndices;
    glm::vec3 lower, upper;

    Node(glm::vec3 min = glm::vec3{ 0 }, glm::vec3 max = glm::vec3{ 0 }) {
        this->lower = min;
        this->upper = max;
        left = -1;
        right = -1;
        level = 0;
        isLeaf = false;
        triIndices.clear();
    }

};

class BoundingVolumeHierarchy {
    
public:
    vector<Node> nodes;
    BoundingVolumeHierarchy(Scene* pScene);

    // Implement these two functions for the Visual Debug.
    // The first function should return how many levels there are in the tree that you have constructed.
    // The second function should draw the bounding boxes of the nodes at the selected level.
    int numLevels() const;
    void debugDraw(int level);
    void debugDrawLeaf();

    // Return true if something is hit, returns false otherwise.
    // Only find hits if they are closer than t stored in the ray and the intersection
    // is on the correct side of the origin (the new t >= 0).
    bool intersect(Ray& ray, HitInfo& hitInfo) const;
    void set_debug_traversal(bool debug);
private:
    bool debug_traversal;
    Scene* m_pScene;
};
