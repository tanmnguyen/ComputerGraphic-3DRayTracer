#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include <algorithm>
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>
#include <iostream>
#include <map>
#include "draw.h"
// Suppress warnings in third-party code.
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec4.hpp>
#include <imgui.h>
#include <nfd.h>
#include <tbb/blocked_range2d.h>
#include <tbb/parallel_for.h>
DISABLE_WARNINGS_POP()
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <framework/image.h>
#include <framework/imguizmo.h>
#include <framework/trackball.h>
#include <framework/variant_helper.h>
#include <framework/window.h>
#include <fstream>
#include <iostream>
#include <optional>
#include <random>
#include <string>
#include <type_traits>
#include <variant>

using namespace std;
// STRUCTS
struct Triangle {
    Vertex v0, v1, v2;
    int materialIndex;
    Triangle(Vertex v0, Vertex v1, Vertex v2, int materialIndex) {
        this->v0 = v0;
        this->v1 = v1;
        this->v2 = v2;
        this->materialIndex = materialIndex;
    }

    glm::vec3 getCenter() {
        return { (v0.position.x + v1.position.x + v2.position.x) / 3.0, (v0.position.y, v1.position.y, v2.position.y) / 3.0,
            (v0.position.z + v1.position.z + v2.position.z) / 3.0 };
    }

};

// END OF STRUCTS

// CONSTANT 
const float INF = numeric_limits<float>::max();
const int MAX_LEVEL = 50;
// END OF CONSTANT 

// VARIABLE
int depth = 1, globalAxis = 0, cntLeaf = 0, global_level = -1;
vector<Mesh> globalMeshes;
vector<Triangle> triangles;
vector<Node> globalNodes, globalLeafs;
// END OF VARIABLE

static constexpr double EPSILON = 1e-6f;
bool compare(float f1, float f2)
{
    return std::abs(f1 - f2) < EPSILON;
}

// find the box contain the triangles.
AxisAlignedBox findBox(int from, int end) {
    glm::vec3 lower = { INF, INF, INF }, upper = { -INF, -INF, -INF };

    for (int i = from; i < end; i++) {
        Triangle tri = triangles[i];
        lower.x = min(lower.x, min(min(tri.v0.position.x, tri.v1.position.x), tri.v2.position.x));
        lower.y = min(lower.y, min(min(tri.v0.position.y, tri.v1.position.y), tri.v2.position.y));
        lower.z = min(lower.z, min(min(tri.v0.position.z, tri.v1.position.z), tri.v2.position.z));
        
        upper.x = max(upper.x, max(max(tri.v0.position.x, tri.v1.position.x), tri.v2.position.x));
        upper.y = max(upper.y, max(max(tri.v0.position.y, tri.v1.position.y), tri.v2.position.y));
        upper.z = max(upper.z, max(max(tri.v0.position.z, tri.v1.position.z), tri.v2.position.z));
    }

    return AxisAlignedBox{ lower, upper };
}

// Compare for sorting function using the center of each triangle.
bool cmp(Triangle a, Triangle b) {
    if (globalAxis == 0) {
        return a.getCenter().x < b.getCenter().x;
    }
    else if (globalAxis == 1) {
        return a.getCenter().y < b.getCenter().y;
    }
    return a.getCenter().z < b.getCenter().z;
}

Node build(int from, int end, int axis, int level) {

    AxisAlignedBox box = findBox(from, end);

    Node node = Node(box.lower, box.upper);
    node.level = level - 1; // set node level
    depth = max(depth, level); // set tree depth

    if (from == end - 1 || level == MAX_LEVEL) {
        node.isLeaf = true;

        for (int i = from; i < end; i++) {
            node.triIndices.push_back(i);
        }
        globalLeafs.push_back(node);


        return node;
    }

    globalAxis = axis;
    sort(triangles.begin() + from, triangles.begin() + end, cmp);

    int mid = (from + end) / 2.0;
    if (from < mid) {
        Node left = build(from, mid, (axis + 1) % 3, level + 1);
        globalNodes.push_back(left);

        node.left = globalNodes.size() - 1;
    }
    if (mid < end) {
        Node right = build(mid, end, (axis + 1) % 3, level + 1);
        globalNodes.push_back(right);

        node.right = globalNodes.size() - 1;
    }

    return node;
}
vector<float> ratio_set_A, ratio_set_B;


void update(AxisAlignedBox& box, Triangle tri) {
    box.lower.x = min(box.lower.x, min(min(tri.v0.position.x, tri.v1.position.x), tri.v2.position.x));
    box.lower.y = min(box.lower.y, min(min(tri.v0.position.y, tri.v1.position.y), tri.v2.position.y));
    box.lower.z = min(box.lower.z, min(min(tri.v0.position.z, tri.v1.position.z), tri.v2.position.z));

    box.upper.x = max(box.upper.x, max(max(tri.v0.position.x, tri.v1.position.x), tri.v2.position.x));
    box.upper.y = max(box.upper.y, max(max(tri.v0.position.y, tri.v1.position.y), tri.v2.position.y));
    box.upper.z = max(box.upper.z, max(max(tri.v0.position.z, tri.v1.position.z), tri.v2.position.z));
}
// compute the volumn of box
float compute_volumn(AxisAlignedBox a) {
    return (a.upper.x - a.lower.x) * (a.upper.y - a.lower.y) * (a.upper.z - a.lower.z);
}

// compute the surface area of the box
float compute_surface_area(AxisAlignedBox a) {
    return (a.upper.x - a.lower.x) * (a.upper.y - a.lower.y) * 2 +
        (a.upper.x - a.lower.x) * (a.upper.z - a.lower.z) * 2 +
        (a.upper.y - a.lower.y) * (a.upper.z - a.lower.z) * 2;
}

// compute the ratio volumn(a) / volumn(b)
float compute_ratio(AxisAlignedBox a, AxisAlignedBox b) {
    float va = compute_surface_area(a);
    float vb = compute_surface_area(b);

#ifndef NDEBUG
    assert(va >= 0);
    assert(vb >= 0);
#endif

    return (!compare(vb, 0.0f)) ? va / vb : 0;
}
void SAH_split(int from, int end, int& min_idx, int& min_axis, AxisAlignedBox original_box, int n_bin) {
    int n = end - from;
    n_bin = min(n_bin, n);

    float optimal_cost = INF;
    for (int axis = 0; axis < 3; axis++) {
        globalAxis = axis;
        sort(triangles.begin() + from, triangles.begin() + end, cmp);
        AxisAlignedBox bv = AxisAlignedBox{ glm::vec3{INF, INF, INF}, glm::vec3{-INF, -INF, -INF} };

        for (int i = from; i < end; i++) {
            update(bv, triangles[i]);
            ratio_set_A[i] = compute_ratio(bv, original_box);
        }

        bv = AxisAlignedBox{ glm::vec3{INF, INF, INF}, glm::vec3{-INF, -INF, -INF} };
        for (int i = end - 1; i >= from; i--) {
            update(bv, triangles[i]);
            ratio_set_B[i] = compute_ratio(bv, original_box);
        }

        int cnt = 1, d = (int) n / n_bin;

        for (int i = from; i < end - 1; i += d) {
            float cost = ratio_set_A[i] * cnt + ratio_set_B[i + 1] * (n - cnt);
            if (cost < optimal_cost) {
                optimal_cost = cost;
                min_idx = i;
                min_axis = axis;
            }
            cnt+=d;
        }
    }
}

// construction using SAH (and binning)
Node buildSAH(int from, int end, int level) {
    AxisAlignedBox box = findBox(from, end);

    Node node = Node(box.lower, box.upper);
    node.level = level - 1;
    
    depth = max(depth, level);

    // Base case
    if (from == end - 1 || level == MAX_LEVEL) {
        node.isLeaf = true;

        for (int i = from; i < end; i++) {
            node.triIndices.push_back(i);
        }

        globalLeafs.push_back(node);

        return node;
    }

    int min_idx = -1, min_axis = -1;
    SAH_split(from, end, min_idx, min_axis, box, 100);

#ifndef NDEBUG
    assert(min_idx > -1 && min_axis > -1);

#endif // !1

    globalAxis = min_axis;
    sort(triangles.begin() + from, triangles.begin() + end, cmp);

    if (from < min_idx + 1) {
        globalNodes.push_back(buildSAH(from, min_idx + 1, level + 1));
        node.left = globalNodes.size() - 1;
    }
    if (min_idx + 1 < end) {
        globalNodes.push_back(buildSAH(min_idx + 1, end, level + 1));
        node.right = globalNodes.size() - 1;
    }

    return node;

}

vector<vector<Node> > nodesAtLevel;
void fastenDebug() {
    global_level = -1;
    nodesAtLevel.clear();
    nodesAtLevel.resize(depth);

    // can be even faster when using sorting algorithm. but this has also achieved O(nlogn) already.
    for (int i = 0; i < depth; i++) {
        for (int j = 0; j < globalNodes.size(); j++) {
            if (globalNodes[j].level == i) {
                nodesAtLevel[i].push_back(globalNodes[j]);
            }
        }
    }
}

BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene* pScene)
    : m_pScene(pScene)
{

    // EXTRACT TRIANGLES
    triangles.clear();
    globalMeshes = pScene->meshes;

  
    for (int i = 0; i < globalMeshes.size(); i++) {
        Mesh mesh = globalMeshes[i];
        for (const auto& tri : mesh.triangles) {
            const auto v0 = mesh.vertices[tri[0]];
            const auto v1 = mesh.vertices[tri[1]];
            const auto v2 = mesh.vertices[tri[2]];
            triangles.push_back(Triangle(v0, v1, v2, i));
        }
    }



    // FINISH EXTRACTING
   
    // INIT VALUE TO BUILD THE HIERACHY
    ratio_set_A.resize(triangles.size()); // does not have to clear
    ratio_set_B.resize(triangles.size()); // does not have to clear
    depth = 1;

    globalLeafs.clear();
    globalNodes.clear();

    if (triangles.size()) {
        //globalNodes.push_back(build(0, triangles.size(), 0, 1));
        globalNodes.push_back(buildSAH(0, triangles.size(), 1));
    }
    
    nodes = globalNodes;
    // FINISH BUILDING

#ifndef NDEBUG
    //cout << "triangle size = " << triangles.size() << " cnt leaf = " << cntLeaf << endl;
     assert(triangles.size() >= globalLeafs.size());
#endif
   
    fastenDebug();
}

// Return the depth of the tree that you constructed. This is used to tell the
// slider in the UI how many steps it should display.
int BoundingVolumeHierarchy::numLevels() const
{   
    return depth;
}

// Use this function to visualize your BVH. This can be useful for debugging. Use the functions in
// draw.h to draw the various shapes. We have extended the AABB draw functions to support wireframe
// mode, arbitrary colors and transparency.
void BoundingVolumeHierarchy::debugDraw(int level)
{
    // example of box
    AxisAlignedBox aabb { glm::vec3(0.0f), glm::vec3(0.0f, 1.05f, 1.05f) };

    // cnt variable to change between 2 colors
    int cnt = 0;
    for (int i = 0; i < nodesAtLevel[level].size(); i++) {
        Node node = nodesAtLevel[level][i];
        aabb = { node.lower, node.upper };

        // changing color 
        if (cnt % 2 == 0) {
            drawAABB(aabb, DrawMode::Filled, glm::vec3(1.0f, 0.0f, 0.00f), 0.5f);
        }
        else {
            drawAABB(aabb, DrawMode::Filled, glm::vec3(1.0f, 1.0f, 0.0f), 0.5f);
        }
        cnt = (cnt + 1) % 2;
    }
}

void BoundingVolumeHierarchy::debugDrawLeaf() {
    srand(time(NULL));
    for (int i = 0; i < globalLeafs.size(); i++) {
        Node node = globalLeafs[i];
        for (int j = 0; j < node.triIndices.size(); j++) {

            Triangle tri = triangles[node.triIndices[j]];

            float v[3];
            for (int i = 0; i < 3; i++) {
                int x = rand() % 256;
                int y = rand() % 256;
                if (x > y) swap(x, y);
                v[i] = (float)x / y;
            }
            //cout << "tri " << glm::to_string(tri.v0.position) << " " << glm::to_string(tri.v1.position) << " " << glm::to_string(tri.v2.position) << endl;
            drawTriangle(tri.v0, tri.v1, tri.v2, glm::vec3{ v[0], v[1], v[2] });
        }
    }
}

pair<Vertex, pair<Vertex, Vertex> > final_triangle;
vector<AxisAlignedBox> BVH_boxes;

bool checkIntersection(Node node, Ray& boxRay, Ray& ray, HitInfo& hitInfo) {
    AxisAlignedBox box;
    box.lower = node.lower;
    box.upper = node.upper;

    bool hit = false;

    Ray copied_ray = boxRay;
    if (intersectRayWithShape(box, copied_ray)) {

        if (node.isLeaf) {
            for (int i = 0; i < node.triIndices.size(); i++) {
                Triangle tri = triangles[node.triIndices[i]];
                if (intersectRayWithTriangle(tri.v0.position, tri.v1.position, tri.v2.position, ray, hitInfo)) {
                    hitInfo.material = globalMeshes[tri.materialIndex].material;
                    hitInfo.v0 = tri.v0;
                    hitInfo.v1 = tri.v1;
                    hitInfo.v2 = tri.v2;
                    hit = true;
#ifndef NDEBUG
                    final_triangle = make_pair(tri.v0, make_pair(tri.v1, tri.v2));
#endif
                }
            }

            return hit;
        }
        if (node.left != -1) {
            copied_ray = boxRay;
            if (checkIntersection(globalNodes[node.left], copied_ray, ray, hitInfo)) {
                hit = true;
            }
        }
        if (node.right != -1) {
            copied_ray = boxRay;
            if (checkIntersection(globalNodes[node.right], copied_ray, ray, hitInfo)) {
                hit = true;
            }
        }
#ifndef NDEBUG
        if (hit) {
            BVH_boxes.push_back(box);
        }
#endif

        return hit;
    }

    return false;
}

void visualDebug(bool hit, const Ray ray) {
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glDisable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);

    // Enable alpha blending. More info at:
    // https://learnopengl.com/Advanced-OpenGL/Blending
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    enableDrawRay = true;

    // Start visualizing
    for (int i = 0; i < BVH_boxes.size(); i++) {
        drawAABB(BVH_boxes[i], DrawMode::Wireframe, { 1,0,0 }, 0.5f);
    }

    if (hit) {
        drawTriangle(final_triangle.first, final_triangle.second.first, final_triangle.second.second, { 0,1,0 });
        drawRay(ray, { 0.5,0.5,0 });

    }
    else {
        drawRay(ray, { 1,0,0 });
    }
    enableDrawRay = false;
    glPopAttrib();
}

// Return true if something is hit, returns false otherwise. Only find hits if they are closer than t stored
// in the ray and if the intersection is on the correct side of the origin (the new t >= 0). Replace the code
// by a bounding volume hierarchy acceleration structure as described in the assignment. You can change any
// file you like, including bounding_volume_hierarchy.h .
bool BoundingVolumeHierarchy::intersect(Ray& ray, HitInfo& hitInfo) const
{
    const Ray ray_no_modified = ray;
    const HitInfo hitInfo_no_modified = hitInfo;

    bool hit = false;

    

#ifndef NDEBUG
    BVH_boxes.clear();
#endif

    if (nodes.size()) {
        // BVH Traversal
        Node root = nodes.back(); // Root of BVH tree

        // Ray for intersecting boxes
        Ray boxRay = ray;
        hit = checkIntersection(root, boxRay, ray, hitInfo);
    }
    

    /*
    for (const auto& mesh : m_pScene->meshes) {
        for (const auto& tri : mesh.triangles) {
            const auto v0 = mesh.vertices[tri[0]];
            const auto v1 = mesh.vertices[tri[1]];
            const auto v2 = mesh.vertices[tri[2]];
            if (intersectRayWithTriangle(v0.position, v1.position, v2.position, ray, hitInfo)) {
                hitInfo.material = mesh.material;
                hitInfo.v0 = v0;
                hitInfo.v1 = v1;
                hitInfo.v2 = v2;
                hit = true;
            }
        }
    }
    */
    

#ifndef NDEBUG
    if (debug_traversal) visualDebug(hit, ray);
#endif

// VALUE DEBUG - NOT VISUAL DEBUG 
#ifndef NDEBUG
    // Given code
    // Intersect with all triangles of all meshes.
    Ray ray1 = ray_no_modified;
    HitInfo hitInfo1 = hitInfo_no_modified;

    bool hit1 = false;
    for (const auto& mesh : m_pScene->meshes) {
        for (const auto& tri : mesh.triangles) {
            const auto v0 = mesh.vertices[tri[0]];
            const auto v1 = mesh.vertices[tri[1]];
            const auto v2 = mesh.vertices[tri[2]];
            if (intersectRayWithTriangle(v0.position, v1.position, v2.position, ray1, hitInfo1)) {
                hitInfo1.material = mesh.material;
                hitInfo1.v0 = v0;
                hitInfo1.v1 = v1;
                hitInfo1.v2 = v2;
                hit1 = true;
            }
        }
    }

    assert(compare(ray.t, ray1.t));
    assert(hit == hit1);
#endif
    // Intersect with spheres.

    for (const auto& sphere : m_pScene->spheres)
        hit |= intersectRayWithShape(sphere, ray, hitInfo);
    return hit;
}

void BoundingVolumeHierarchy::set_debug_traversal(bool debug) {
    debug_traversal = debug;
}