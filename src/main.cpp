#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include "ray_tracing.h"
#include "screen.h"
#include "barycentric.cpp"

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
#include <tbb/blocked_range3d.h>
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

constexpr glm::ivec2 windowResolution{ 800, 800 };
const std::filesystem::path dataPath{ DATA_DIR };
enum class ViewMode {
    Rasterization = 0,
    RayTracing = 1
};

static constexpr double EPSILON = 1e-6f;

float DOF_focus, DOF_aperture;
bool DOF;

Scene scene = loadScene(SceneType{ SceneType::SingleTriangle }, dataPath);
BoundingVolumeHierarchy bvh{ &scene };

bool isClose(float f1, float f2)
{
    return std::abs(f1 - f2) < EPSILON;
}

bool debug_traversal, debug_normal_interpolation, bloom_effect, multipleRays, glossy;

glm::vec3 diffuseOnly(HitInfo hitinfo, const glm::vec3& vertexPos, const glm::vec3& lightPos, const glm::vec3& color)
{//diffuse component of phong
    glm::vec3 light = lightPos - vertexPos;
    float mag = glm::length(light);
    light = light / mag;

    float dot = glm::dot(hitinfo.normal, light);
    if (dot < 0) {
        dot = 0;
    }

    float x = hitinfo.material.kd.x * color.x * dot;
    float y = hitinfo.material.kd.y * color.y * dot;
    float z = hitinfo.material.kd.z * color.z * dot;
    return glm::vec3(x, y, z);
}
glm::vec3 phongSpecularOnly(HitInfo hitinfo, const glm::vec3& vertexPos, const glm::vec3& lightPos, const glm::vec3& cameraPos, const glm::vec3& color)
{//speculaar component of phong
    glm::vec3 view = cameraPos - vertexPos;
    view = view / glm::length(view);
    glm::vec3 light = lightPos - vertexPos;
    float mag = glm::length(light);
    light = light / mag;
    float dotr = glm::dot(light, hitinfo.normal);
    if (dotr < 0) {
        return glm::vec3(0, 0, 0);
    }
    glm::vec3 ref = 2 * dotr * hitinfo.normal - light;
    ref = ref / glm::length(ref);
    float dot = glm::dot(view, ref);
    if (dot < 0) {
        dot = 0;
    }
    glm::vec3 spec = hitinfo.material.ks * color * pow(dot, hitinfo.material.shininess);


    return spec;
}

static glm::vec3 computeReflection(glm::vec3 point, glm::vec3 normal, Ray ray) {
    normal = glm::normalize(normal);
    glm::vec3 toward = glm::normalize(-ray.direction);
    glm::vec3 reflection = glm::normalize((2 * glm::dot(normal, toward) * normal - toward));



    assert(isClose(glm::dot(toward, normal), glm::dot(reflection, normal)));
    //cout << "checking " << glm::dot(toward, normal) << " " << glm::dot(reflection, normal) << endl;
    return reflection;
}
static glm::vec3 glossyReflection(glm::vec3 reflection, HitInfo hitInfo) {
    if (!glossy)
        return reflection;

    glm::vec3 w = glm::normalize(reflection);
    glm::vec3 t = reflection;
    if (glm::abs(t.x) < glm::abs(t.y) && glm::abs(t.x) < glm::abs(t.z)) {
        t.x = 1;
    }
    else if (glm::abs(t.y) < glm::abs(t.x) && glm::abs(t.y) < glm::abs(t.z)) {
        t.y = 1;
    }
    else {
        t.z = 1;
    }

    glm::vec3 uvec = glm::normalize(glm::cross(t, w));
    glm::vec3 vvec = glm::cross(w, uvec);

    float a = 0.35 / hitInfo.material.shininess;

    float u = -a / 2 + ((rand() % 100) / 100.0f) * a;
    float v = -a / 2 + ((rand() % 100) / 100.0f) * a;

    glm::vec3 glossyr = reflection + u * uvec + v * vvec;
    return glossyr;
}

bool hardShadow(Ray ray, glm::vec3 lightPosition, HitInfo hitInfo, glm::vec3 color) {
    // Hard Shadow incl. Visual Debug
    Ray rayVD;
    HitInfo hitInfoVD;

    rayVD.origin = ray.origin + ray.direction * ray.t;
    rayVD.direction = glm::normalize(lightPosition - rayVD.origin);
    float t = rayVD.t = glm::distance(lightPosition, rayVD.origin);


    // introduce delta, so .intersect doesn't collide with the object where rayVD.origin is on
    float delta = 0.0001; //0.000005f;
    rayVD.origin = rayVD.origin + rayVD.direction * delta;

    bvh.intersect(rayVD, hitInfoVD); //doesn't matter what is returned, if ray intersect anything else before the light,  the t value is decreased

    rayVD.origin = rayVD.origin - rayVD.direction * delta; // reset origin to intersection point without delta


    float dotCheck = glm::dot(rayVD.direction, hitInfo.normal);
    float dotCheck2 = glm::dot(ray.direction, hitInfo.normal);
    //printf("vd: %f, ray:  %f \n", dotCheck, dotCheck2);

    if (rayVD.t < t || (dotCheck <= 0 && dotCheck2 <= 0) || (dotCheck > 0 && dotCheck2 > 0)) { // hard shadow
        drawRay(rayVD, glm::vec3(1.0f, 0.0f, 0.0f));
        return true;
    }
    drawRay(rayVD, glm::vec3(color));
    return false;
}

glm::vec3 sampleSegmentLight(SegmentLight segmentLight, Ray ray, HitInfo hitInfo, const glm::vec3& intersectionPoint) {
    int samples = 10;

    float length = glm::distance(segmentLight.endpoint0, segmentLight.endpoint1);
    float stepSize = length / samples;

    glm::vec3 result(0.0f);

    //when using random values change <= to <
    for (size_t i = 0; i < samples; i++)
    {
        glm::vec3 position = segmentLight.endpoint0 + glm::normalize(segmentLight.endpoint1 - segmentLight.endpoint0) * ((i * stepSize) + (rand() % ((int)(stepSize * 100.0f)) / 100.0f));
        float alpha = glm::distance(position, segmentLight.endpoint1) / length;
        glm::vec3 color = segmentLight.color0 * alpha + segmentLight.color1 * (1 - alpha);
        if (hardShadow(ray, position, hitInfo, color))
            continue;
        result += diffuseOnly(hitInfo, intersectionPoint, position, color) + phongSpecularOnly(hitInfo, intersectionPoint, position, ray.origin, color);
    }

    return result / (float)samples;
}

glm::vec3 sampleParalellogramLight(ParallelogramLight parallelogramLight, Ray ray, HitInfo hitInfo, const glm::vec3& intersectionPoint) {
    int samples = 5;

    float width = glm::length(parallelogramLight.edge01);
    float height = glm::length(parallelogramLight.edge02);

    glm::vec3 v1 = parallelogramLight.v0 + parallelogramLight.edge01;
    glm::vec3 v2 = parallelogramLight.v0 + parallelogramLight.edge02;
    glm::vec3 v3 = v1 + parallelogramLight.edge02;

    float stepSizeWidth = width / samples;
    float stepSizeHeight = height / samples;

    glm::vec3 result(0.0f);

    //when using random values change <= to <
    for (size_t i = 0; i < samples; i++)
    {
        for (size_t j = 0; j < samples; j++)
        {
            glm::vec3 position = parallelogramLight.v0
                + glm::normalize(v1 - parallelogramLight.v0) * ((i * stepSizeWidth) + (rand() % ((int)(stepSizeWidth * 100.0f)) / 100.0f)) +
                +glm::normalize(v2 - parallelogramLight.v0) * ((j * stepSizeHeight) + (rand() % ((int)(stepSizeHeight * 100.0f)) / 100.0f));

            float alpha = glm::distance(position - glm::normalize(v2 - parallelogramLight.v0) * (j * stepSizeHeight), v1) / width;
            float beta = glm::distance(position - glm::normalize(v1 - parallelogramLight.v0) * (i * stepSizeWidth), v2) / height;

            glm::vec3 color = (parallelogramLight.color0 * alpha + parallelogramLight.color1 * (1 - alpha)) * (beta)
                +((1 - alpha) * parallelogramLight.color3 + alpha * parallelogramLight.color2) * (1 - beta);

            if (hardShadow(ray, position, hitInfo, color))
                continue;

            result += diffuseOnly(hitInfo, intersectionPoint, position, color) + phongSpecularOnly(hitInfo, intersectionPoint, position, ray.origin, color);
        }
    }

    return result / (float)(samples * samples);
}
static Ray getRefractedRay(Ray r, glm::vec3 point) {
    Ray reflected;
    reflected.origin = point;
    reflected.direction = r.direction;
    return reflected;
}


//glm::vec3 getFinalColor(const Scene& scene, const BoundingVolumeHierarchy& bvh, Ray ray);
glm::vec3 getFinalColor(Ray ray, bool motionBlur);

glm::vec3 reflectionColor(Ray ray, HitInfo hitInfo, glm::vec3 point) {
    glm::vec3 ref = computeReflection(point, hitInfo.normal, ray);
    Ray reflected;
    reflected.direction = glm::normalize(ref);
    reflected.origin = point;

    int glossyTimes = 1;
    if (glossy) glossyTimes = 100;

    glm::vec3 color(0.0f);
    for (size_t i = 0; i < glossyTimes; i++)
    {
        glm::vec3 glossyref = glossyReflection(ref, hitInfo);
        //draws normal reflected ray for visual debug
        drawRay(reflected, getFinalColor(reflected, false));
        Ray rayn;
        rayn.origin = point;

        rayn.direction = glm::normalize(glossyref);
        //cout << "-> " << glm::to_string(rayn.direction) << endl;
        glm::vec3 refcolor = getFinalColor(rayn, false);
        //draws glossy reflection ray for visual debug
        drawRay(rayn, glm::vec3(1.0f, 0.0f, 0.0f));
        color += refcolor;
    }



    return  color / (float)glossyTimes;
}

static glm::vec3 bilinearInterpolation(float u, float v, const glm::vec3 c00, const glm::vec3 c01, const glm::vec3 c10, const glm::vec3 c11) {

    glm::vec3 result = ((1 - u) * (1 - v) * c00) + (u * (1 - v) * c10) + ((1 - u) * v * c01) + (u * v * c11);
    return result;
}

static glm::vec3 getFinalColor(Ray ray, bool motionBlur = false)
{
    //cout << "called " << glm::to_string(ray.origin) << " " << glm::to_string(ray.direction) << endl;
    HitInfo hitInfo;
    if (bvh.intersect(ray, hitInfo)) {
        //cout << "MATERIAL " << glm::to_string(hitInfo.material.ks) << endl;
        // Draw a white debug ray if the ray hits.

        ray.t -= (float)(1e-6);
        if (hitInfo.toBeInterpolated) {
            hitInfo.normal = interpolateNormal(ray, hitInfo.v0, hitInfo.v1, hitInfo.v2, debug_normal_interpolation);
        }

        //by tan




        if (hitInfo.material.kdTexture) {
            Vertex v0 = hitInfo.v0;
            Vertex v1 = hitInfo.v1;
            Vertex v2 = hitInfo.v2;

            // define texture position for intersected position p
            glm::vec2 texturePosition = interpolateCoord(ray, v0, v1, v2);
            hitInfo.material.kd = hitInfo.material.kdTexture->getTexel(texturePosition);
        }


        //by ara
        glm::vec3 point = ray.origin + ray.t * ray.direction;

        //drawRay(ray, glm::vec3(1.0f));
        // Set the color of the pixel to white if the ray hits.
        glm::vec3 finalcolor = { 0,0,0 };
        for (const auto& light : scene.lights) {
            if (std::holds_alternative<PointLight>(light)) {
                const PointLight pointLight = std::get<PointLight>(light);

                // if intersection in hard shadow, light source does not contribute to it

                if (hardShadow(ray, pointLight.position, hitInfo, glm::vec3(1.0f))) {
                    continue;

                }

                // Perform your calculations for a point light.
                // sum to get phong
                finalcolor = finalcolor + diffuseOnly(hitInfo, point, pointLight.position, pointLight.color) + phongSpecularOnly(hitInfo, point, pointLight.position, ray.origin, pointLight.color);

                if (hitInfo.material.ks != glm::vec3(0.0f))
                    finalcolor = finalcolor + reflectionColor(ray, hitInfo, point);



            }
            else if (std::holds_alternative<SegmentLight>(light)) {
                const SegmentLight segmentLight = std::get<SegmentLight>(light);
                // Perform your calculations for a segment light.

                if (hitInfo.material.ks != glm::vec3(0.0f))
                    finalcolor = finalcolor + reflectionColor(ray, hitInfo, point);


                finalcolor += sampleSegmentLight(segmentLight, ray, hitInfo, point); //as in pointlight calculate diffuceonly and specularonly with the sampled light
            }
            else if (std::holds_alternative<ParallelogramLight>(light)) {
                const ParallelogramLight parallelogramLight = std::get<ParallelogramLight>(light);
                // Perform your calculations for a parallelogram light.
                finalcolor += sampleParalellogramLight(parallelogramLight, ray, hitInfo, point);

                if (hitInfo.material.ks != glm::vec3(0.0f))
                    finalcolor = finalcolor + reflectionColor(ray, hitInfo, point);

            }
            //cout << "chekcing " << glm::to_string(finalcolor) << endl;
        }

        if (motionBlur) {
            glm::vec3 movement = (ray.direction / 50.0f + glm::vec3(-0.01f, 0.0f, 0.0f));
            for (size_t i = 0; i < 15; i++)
            {
                Ray newRay;
                newRay.direction = ray.direction;
                newRay.origin = ray.origin + movement * (float)i;
                //HitInfo newHitInfo;
                //bvh.intersect(newRay, hitInfo);
                //newRay.t += 1e3;
                getFinalColor(newRay);
            }
        }
        else
            drawRay(ray, finalcolor);
        //drawRay(ray, { 1,1,1 }); // draw ray using white for visually observable
        //return finalcolor;



        //vd mutipleRays 
        if (multipleRays) {
            for (size_t i = 0; i < 20; i++)
            {
                Ray r;
                r.origin = ray.origin + glm::vec3(0.5f - (rand() % 100) / 100.0f, 0.5f - (rand() % 100) / 100.0f, 0.0f) * 0.1f;
                r.direction = ray.direction;

                drawRay(r, glm::vec3(0, 1.0f, 0));
            }
        }



        if (hitInfo.material.transparency < 1.0f) {
            Ray refracted;
            refracted.direction = ray.direction;
            refracted.origin = point + 0.0001f * ray.direction;

            drawRay(refracted, glm::vec3(0.0f, 0.0f, 1.0f));
            glm::vec3 color = getFinalColor(refracted);

            finalcolor = hitInfo.material.transparency * finalcolor + (1 - hitInfo.material.transparency) * color;

        }
        return finalcolor;

    }
    else {
        // Draw a red debug ray if the ray missed.
        drawRay(ray, glm::vec3(1.0f, 0.0f, 0.0f));
        // Set the color of the pixel to black if the ray misses.
        return glm::vec3(0.0f);
    }
}


float random_float(float low, float high) {
    float dif = ((float)rand()) / (float)RAND_MAX * (high - low);
    float res = low + dif;

    assert(res >= low && res <= high);
    return res;
}
// compute many rays from the lens around the original ray
vector<Ray> compute_DOF_rays(Ray ray) {
    if (!DOF)
        return { ray };

    vector<Ray> res(0);
    res.push_back(ray);


    glm::vec3 convergence_point = ray.origin + DOF_focus * ray.direction;

    int n_rays = 100;
    for (int i = 0; i < n_rays; i++) {
        Ray cur;

        glm::vec3 v = glm::vec3{ random_float(-0.5, 0.5), random_float(-0.5, 0.5), random_float(-0.5, 0.5) } *DOF_aperture;
        cur.origin = ray.origin + v;
        cur.direction = convergence_point - cur.origin;
        cur.t = numeric_limits<float>::max();
        res.push_back(cur);
    }


    return res;
}
glm::vec3 get_DOF_color(const Ray cameraRay) {
    glm::vec3 color = glm::vec3{ 0, 0, 0 };
    vector<Ray> rays = compute_DOF_rays(cameraRay);

    for (int i = 0; i < rays.size(); i++) {
        color += getFinalColor(rays[i]);
    }
    color /= rays.size();
    return color;
}


static void setOpenGLMatrices(const Trackball& camera);
static void drawLightsOpenGL(const Scene& scene, const Trackball& camera, int selectedLight);
static void drawSceneOpenGL(const Scene& scene);

// This is the main rendering function. You are free to change this function in any way (including the function signature).
static void renderRayTracing(const Scene& scene, const Trackball& camera, const BoundingVolumeHierarchy& bvh, Screen& screen, bool motionBlur = false)
{
    int interval = 1;
    int rays = 1;

    if (motionBlur)
        interval = 20;
    if (multipleRays)
        rays = 20;

#ifndef NDEBUG
    // Single threaded in debug mode
    for (int y = 0; y < windowResolution.y; y++) {
        for (int x = 0; x != windowResolution.x; x++) {
            /*// NOTE: (-1, -1) at the bottom left of the screen, (+1, +1) at the top right of the screen.
            const glm::vec2 normalizedPixelPos {
                float(x) / windowResolution.x * 2.0f - 1.0f,
                float(y) / windowResolution.y * 2.0f - 1.0f
            };
            const Ray cameraRay = camera.generateRay(normalizedPixelPos);
            screen.setPixel(x, y, getFinalColor(scene, bvh, cameraRay));*/

            glm::vec3 multipleRayColor(0.0f);
            for (int j = 0; j < rays; j++) {
                glm::vec2 offset(0.5f - (rand() % 100) / 100.0f, 0.5f - (rand() % 100) / 100.0f);
                //cout << offset.x << " " << offset.y << std::endl;


                // define movement here
                glm::vec3 movement = camera.forward() / 50.0f; // cameraCopy.left() / 40.0f;


                glm::vec3 finalColor(0.0f);
                for (int i = 0; i < interval; i++) {
                    // NOTE: (-1, -1) at the bottom left of the screen, (+1, +1) at the top right of the screen.
                    const glm::vec2 normalizedPixelPos{
                        (float(x) + offset.x) / windowResolution.x * 2.0f - 1.0f,
                        (float(y) + offset.y) / windowResolution.y * 2.0f - 1.0f
                    };

                    Ray cameraRay;
                    if (motionBlur) cameraRay = camera.generateRay(normalizedPixelPos, movement * (float)(i)); // +(rand() % 100 / 100.0f)));
                    else cameraRay = camera.generateRay(normalizedPixelPos);

                    finalColor += get_DOF_color(cameraRay);
                }

                multipleRayColor += (finalColor / (float)interval);
            }

            screen.setPixel(x, y, multipleRayColor / (float)rays);
        }
    }
#else
    // Multi-threaded in release mode 

    glm::vec2 offset(0.5f - (rand() % 100) / 100.0f, 0.5f - (rand() % 100) / 100.0f);
    // define movement here
    glm::vec3 movement = camera.forward() / 50.0f; // cameraCopy.left() / 40.0f;


    const tbb::blocked_range2d<int, int> windowRange{ 0, windowResolution.y, 0, windowResolution.x };
    tbb::parallel_for(windowRange, [&](tbb::blocked_range2d<int, int> localRange) {
        for (int y = std::begin(localRange.rows()); y != std::end(localRange.rows()); y++) {
            for (int x = std::begin(localRange.cols()); x != std::end(localRange.cols()); x++) {

                glm::vec3 multipleRayColor(0.0f);
                for (int j = 0; j < rays; j++) {
                    //cout << offset.x << " " << offset.y << std::endl;

                    glm::vec3 finalColor(0.0f);
                    for (int i = 0; i < interval; i++) {
                        // NOTE: (-1, -1) at the bottom left of the screen, (+1, +1) at the top right of the screen.
                        const glm::vec2 normalizedPixelPos{
                            (float(x) + offset.x) / windowResolution.x * 2.0f - 1.0f,
                            (float(y) + offset.y) / windowResolution.y * 2.0f - 1.0f
                        };

                        Ray cameraRay;
                        if (motionBlur) cameraRay = camera.generateRay(normalizedPixelPos, movement * (float)(i));
                        else cameraRay = camera.generateRay(normalizedPixelPos);

                        finalColor += get_DOF_color(cameraRay);
                    }

                    multipleRayColor += (finalColor / (float)interval);
                }

                screen.setPixel(x, y, multipleRayColor / (float)rays);
            }
        }
        });
    if (bloom_effect) screen.bloom(25, 0.1f, 0.8f);
#endif
}

int main(int argc, char** argv)
{
    Trackball::printHelp();
    std::cout << "\n Press the [R] key on your keyboard to create a ray towards the mouse cursor" << std::endl
        << std::endl;

    Window window{ "Final Project", windowResolution, OpenGLVersion::GL2 };
    Screen screen{ windowResolution };
    Trackball camera{ &window, glm::radians(50.0f), 3.0f };
    camera.setCamera(glm::vec3(0.0f, 0.0f, 0.0f), glm::radians(glm::vec3(20.0f, 20.0f, 0.0f)), 3.0f);

    SceneType sceneType{ SceneType::SingleTriangle };
    scene = loadScene(sceneType, dataPath);
    bvh = { &scene };

    std::optional<Ray> optDebugRay;


    int bvhDebugLevel = 0;
    bool debugBVH{ false };
    DOF = false;
    ViewMode viewMode{ ViewMode::Rasterization };
    //by linus
    bool motionBlur{ false };
    multipleRays = false;
    glossy = false;

    // to debug leave of bvh tree
    bool debug_leaf = false;
    debug_traversal = false;
    debug_normal_interpolation = false;
    bloom_effect = false;
    bool dof = false;

    window.registerKeyCallback([&](int key, int /* scancode */, int action, int /* mods */) {
        if (action == GLFW_PRESS) {
            switch (key) {
            case GLFW_KEY_R: {
                // Shoot a ray. Produce a ray from camera to the far plane.
                const auto tmp = window.getNormalizedCursorPos();
                optDebugRay = camera.generateRay(tmp * 2.0f - 1.0f);
            } break;
            case GLFW_KEY_ESCAPE: {
                window.close();
            } break;
            };
        }
        });

    int selectedLightIdx = scene.lights.empty() ? -1 : 0;
    while (!window.shouldClose()) {
        window.updateInput();

        // === Setup the UI ===
        ImGui::Begin("Final Project");
        {
            constexpr std::array items{ "SingleTriangle", "Cube (segment light)", "Cornell Box (with mirror)", "Cornell Box (parallelogram light and mirror)", "Monkey", "Teapot", "Dragon", /* "AABBs",*/ "Spheres", /*"Mixed",*/ "Custom" };
            if (ImGui::Combo("Scenes", reinterpret_cast<int*>(&sceneType), items.data(), int(items.size()))) {
                optDebugRay.reset();
                scene = loadScene(sceneType, dataPath);
                selectedLightIdx = scene.lights.empty() ? -1 : 0;
                bvh = BoundingVolumeHierarchy(&scene);

                scene = scene;
                bvh = bvh;

                if (optDebugRay) {
                    HitInfo dummy{};
                    bvh.intersect(*optDebugRay, dummy);
                }
            }

            //by linus
            ImGui::Checkbox("Motion Blur", &motionBlur);
            ImGui::Checkbox("Multiple Rays", &multipleRays);
            ImGui::Checkbox("Glossy Reflection", &glossy);
            ImGui::Checkbox("Enable Depth Of Field", &DOF);
            ImGui::Checkbox("Enable Bloom Effect", &bloom_effect);
        }
        {
            constexpr std::array items{ "Rasterization", "Ray Traced" };
            ImGui::Combo("View mode", reinterpret_cast<int*>(&viewMode), items.data(), int(items.size()));
        }
        if (ImGui::Button("Render to file")) {
            // Show a file picker.
            nfdchar_t* pOutPath = nullptr;
            const nfdresult_t result = NFD_SaveDialog("bmp", nullptr, &pOutPath);
            if (result == NFD_OKAY) {
                std::filesystem::path outPath{ pOutPath };
                free(pOutPath); // NFD is a C API so we have to manually free the memory it allocated.
                outPath.replace_extension("bmp"); // Make sure that the file extension is *.bmp

                // Perform a new render and measure the time it took to generate the image.
                using clock = std::chrono::high_resolution_clock;
                const auto start = clock::now();
                //by linus (the if and motionblur)
                if (motionBlur)
                    renderRayTracing(scene, camera, bvh, screen, true);
                else
                    renderRayTracing(scene, camera, bvh, screen);
                const auto end = clock::now();
                std::cout << "Time to render image: " << std::chrono::duration<float, std::milli>(end - start).count() << " milliseconds" << std::endl;

                // Store the new image.
                screen.writeBitmapToFile(outPath);
            }
        }
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Text("Debugging");
        if (viewMode == ViewMode::Rasterization) {
            ImGui::Checkbox("Draw BVH", &debugBVH);
            if (debugBVH)
                ImGui::SliderInt("BVH Level", &bvhDebugLevel, 0, bvh.numLevels() - 1);

            ImGui::Checkbox("Visual Debug triangles at leaf BVH", &debug_leaf);
            ImGui::Checkbox("Visual Debug intersected box when shooting ray", &debug_traversal);
            ImGui::Checkbox("Visual Debug normal interpolation", &debug_normal_interpolation);
            ImGui::Checkbox("Visual Debug depth of field", &dof);

        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Text("Lights");
        {
            std::vector<std::string> options;
            options.push_back("None");
            for (size_t i = 0; i < scene.lights.size(); i++) {
                options.push_back("Light " + std::to_string(i));
            }
            std::vector<const char*> optionsPointers;
            std::transform(std::begin(options), std::end(options), std::back_inserter(optionsPointers), [](const auto& str) { return str.c_str(); });

            // Offset such that selectedLightIdx=-1 becomes item 0 (None).
            ++selectedLightIdx;
            ImGui::Combo("Selected light", &selectedLightIdx, optionsPointers.data(), static_cast<int>(optionsPointers.size()));
            --selectedLightIdx;

            if (selectedLightIdx >= 0) {
                setOpenGLMatrices(camera);
                std::visit(
                    make_visitor(
                        [&](PointLight& light) {
                            showImGuizmoTranslation(window, camera, light.position); // 3D controls to translate light source.
                            ImGui::DragFloat3("Light position", glm::value_ptr(light.position), 0.01f, -3.0f, 3.0f);
                            ImGui::ColorEdit3("Light color", glm::value_ptr(light.color));
                        },
                        [&](SegmentLight& light) {
                            static int selectedEndpoint = 0;
                            // 3D controls to translate light source.
                            if (selectedEndpoint == 0)
                                showImGuizmoTranslation(window, camera, light.endpoint0);
                            else
                                showImGuizmoTranslation(window, camera, light.endpoint1);

                            const std::array<const char*, 2> endpointOptions{ "Endpoint 0", "Endpoint 1" };
                            ImGui::Combo("Selected endpoint", &selectedEndpoint, endpointOptions.data(), (int)endpointOptions.size());
                            ImGui::DragFloat3("Endpoint 0", glm::value_ptr(light.endpoint0), 0.01f, -3.0f, 3.0f);
                            ImGui::DragFloat3("Endpoint 1", glm::value_ptr(light.endpoint1), 0.01f, -3.0f, 3.0f);
                            ImGui::ColorEdit3("Color 0", glm::value_ptr(light.color0));
                            ImGui::ColorEdit3("Color 1", glm::value_ptr(light.color1));
                        },
                            [&](ParallelogramLight& light) {
                            glm::vec3 vertex1 = light.v0 + light.edge01;
                            glm::vec3 vertex2 = light.v0 + light.edge02;

                            static int selectedVertex = 0;
                            // 3D controls to translate light source.
                            if (selectedVertex == 0)
                                showImGuizmoTranslation(window, camera, light.v0);
                            else if (selectedVertex == 1)
                                showImGuizmoTranslation(window, camera, vertex1);
                            else
                                showImGuizmoTranslation(window, camera, vertex2);

                            const std::array<const char*, 3> vertexOptions{ "Vertex 0", "Vertex 1", "Vertex 2" };
                            ImGui::Combo("Selected vertex", &selectedVertex, vertexOptions.data(), (int)vertexOptions.size());
                            ImGui::DragFloat3("Vertex 0", glm::value_ptr(light.v0), 0.01f, -3.0f, 3.0f);
                            ImGui::DragFloat3("Vertex 1", glm::value_ptr(vertex1), 0.01f, -3.0f, 3.0f);
                            light.edge01 = vertex1 - light.v0;
                            ImGui::DragFloat3("Vertex 2", glm::value_ptr(vertex2), 0.01f, -3.0f, 3.0f);
                            light.edge02 = vertex2 - light.v0;

                            ImGui::ColorEdit3("Color 0", glm::value_ptr(light.color0));
                            ImGui::ColorEdit3("Color 1", glm::value_ptr(light.color1));
                            ImGui::ColorEdit3("Color 2", glm::value_ptr(light.color2));
                            ImGui::ColorEdit3("Color 3", glm::value_ptr(light.color3));
                        },
                            [](auto) { /* any other type of light */ }),
                    scene.lights[selectedLightIdx]);
            }
        }



        if (ImGui::Button("Add point light")) {
            selectedLightIdx = int(scene.lights.size());
            scene.lights.push_back(PointLight{ .position = glm::vec3(0.0f), .color = glm::vec3(1.0f) });
        }
        if (ImGui::Button("Add segment light")) {
            selectedLightIdx = int(scene.lights.size());
            scene.lights.push_back(SegmentLight{ .endpoint0 = glm::vec3(0.0f), .endpoint1 = glm::vec3(1.0f), .color0 = glm::vec3(1, 0, 0), .color1 = glm::vec3(0, 0, 1) });
        }
        if (ImGui::Button("Add parallelogram light")) {
            selectedLightIdx = int(scene.lights.size());
            scene.lights.push_back(ParallelogramLight{
                .v0 = glm::vec3(0.0f),
                .edge01 = glm::vec3(1, 0, 0),
                .edge02 = glm::vec3(0, 1, 0),
                .color0 = glm::vec3(1, 0, 0), // red
                .color1 = glm::vec3(0, 1, 0), // green
                .color2 = glm::vec3(0, 0, 1), // blue
                .color3 = glm::vec3(1, 1, 1) // white
                });
        }
        if (selectedLightIdx >= 0 && ImGui::Button("Remove selected light")) {
            scene.lights.erase(std::begin(scene.lights) + selectedLightIdx);
            selectedLightIdx = -1;
        }

        // Clear screen.
        glViewport(0, 0, window.getFrameBufferSize().x, window.getFrameBufferSize().y);
        glClearDepth(1.0f);
        glClearColor(0.0, 0.0, 0.0, 0.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        setOpenGLMatrices(camera);

        // Draw either using OpenGL (rasterization) or the ray tracing function.
        switch (viewMode) {
        case ViewMode::Rasterization: {
            glPushAttrib(GL_ALL_ATTRIB_BITS);
            drawSceneOpenGL(scene);
            if (optDebugRay) {
                // Call getFinalColor for the debug ray. Ignore the result but tell the function that it should
                // draw the rays instead.
                enableDrawRay = true;

                if (dof) get_DOF_color(*optDebugRay);
                (void)getFinalColor(*optDebugRay);

                enableDrawRay = false;
            }
            glPopAttrib();
        } break;
        case ViewMode::RayTracing: {
            screen.clear(glm::vec3(0.0f));
            renderRayTracing(scene, camera, bvh, screen);
            screen.setPixel(0, 0, glm::vec3(1.0f));
            screen.draw(); // Takes the image generated using ray tracing and outputs it to the screen using OpenGL.
        } break;
        default:
            break;
        };

        drawLightsOpenGL(scene, camera, selectedLightIdx);

        if (debugBVH) {
            glPushAttrib(GL_ALL_ATTRIB_BITS);
            setOpenGLMatrices(camera);
            glDisable(GL_LIGHTING);
            glEnable(GL_DEPTH_TEST);

            // Enable alpha blending. More info at:
            // https://learnopengl.com/Advanced-OpenGL/Blending
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            enableDrawRay = true;
            bvh.debugDraw(bvhDebugLevel);
            enableDrawRay = false;
            glPopAttrib();
        }

        if (DOF) {
            ImGui::DragFloat("FOCUS", &DOF_focus, 10.0f);
            ImGui::DragFloat("APERTURE", &DOF_aperture, 80.0f);
        }
        else {
            DOF_focus = 3.1f;
            DOF_aperture = 0.2;
        }
        if (debug_leaf) {
            glPushAttrib(GL_ALL_ATTRIB_BITS);
            setOpenGLMatrices(camera);
            glDisable(GL_LIGHTING);
            glEnable(GL_DEPTH_TEST);

            // Enable alpha blending. More info at:
            // https://learnopengl.com/Advanced-OpenGL/Blending
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            enableDrawRay = true;
            bvh.debugDrawLeaf();
            enableDrawRay = false;
            glPopAttrib();
        }

        bvh.set_debug_traversal(debug_traversal);


        ImGui::End();
        window.swapBuffers();
    }



    return 0;
}

static void setOpenGLMatrices(const Trackball& camera)
{
    // Load view matrix.
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    const glm::mat4 viewMatrix = camera.viewMatrix();
    glMultMatrixf(glm::value_ptr(viewMatrix));

    // Load projection matrix.
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    const glm::mat4 projectionMatrix = camera.projectionMatrix();
    glMultMatrixf(glm::value_ptr(projectionMatrix));
}

static void drawLightsOpenGL(const Scene& scene, const Trackball& camera, int selectedLight)
{
    // Normals will be normalized in the graphics pipeline.
    glEnable(GL_NORMALIZE);
    // Activate rendering modes.
    glEnable(GL_DEPTH_TEST);
    // Draw front and back facing triangles filled.
    glPolygonMode(GL_FRONT, GL_FILL);
    glPolygonMode(GL_BACK, GL_FILL);
    // Interpolate vertex colors over the triangles.
    glShadeModel(GL_SMOOTH);

    glDisable(GL_LIGHTING);
    // Draw all non-selected lights.
    for (size_t i = 0; i < scene.lights.size(); i++) {
        std::visit(
            make_visitor(
                [](const PointLight& light) { drawSphere(light.position, 0.01f, light.color); },
                [](const SegmentLight& light) {
                    glPushAttrib(GL_ALL_ATTRIB_BITS);
                    glBegin(GL_LINES);
                    glColor3fv(glm::value_ptr(light.color0));
                    glVertex3fv(glm::value_ptr(light.endpoint0));
                    glColor3fv(glm::value_ptr(light.color1));
                    glVertex3fv(glm::value_ptr(light.endpoint1));
                    glEnd();
                    glPopAttrib();
                    drawSphere(light.endpoint0, 0.01f, light.color0);
                    drawSphere(light.endpoint1, 0.01f, light.color1);
                },
                [](const ParallelogramLight& light) {
                    glPushAttrib(GL_ALL_ATTRIB_BITS);
                    glBegin(GL_QUADS);
                    glColor3fv(glm::value_ptr(light.color0));
                    glVertex3fv(glm::value_ptr(light.v0));
                    glColor3fv(glm::value_ptr(light.color1));
                    glVertex3fv(glm::value_ptr(light.v0 + light.edge01));
                    glColor3fv(glm::value_ptr(light.color3));
                    glVertex3fv(glm::value_ptr(light.v0 + light.edge01 + light.edge02));
                    glColor3fv(glm::value_ptr(light.color2));
                    glVertex3fv(glm::value_ptr(light.v0 + light.edge02));
                    glEnd();
                    glPopAttrib();
                },
                    [](auto) { /* any other type of light */ }),
            scene.lights[i]);
    }

    // Draw a colored sphere at the location at which the trackball is looking/rotating around.
    glDisable(GL_LIGHTING);
    drawSphere(camera.lookAt(), 0.01f, glm::vec3(0.2f, 0.2f, 1.0f));
}

void drawSceneOpenGL(const Scene& scene)
{
    // Activate the light in the legacy OpenGL mode.
    glEnable(GL_LIGHTING);

    // Tell OpenGL where the lights are (so it nows how to shade surfaces in the scene).
    // This is only used in the rasterization view. OpenGL only supports point lights so
    // we replace segment/parallelogram lights by point lights.
    int i = 0;
    const auto enableLight = [&](const glm::vec3& position, const glm::vec3 color) {
        glEnable(GL_LIGHT0 + i);
        const glm::vec4 position4{ position, 1 };
        glLightfv(GL_LIGHT0 + i, GL_POSITION, glm::value_ptr(position4));
        const glm::vec4 color4{ glm::clamp(color, 0.0f, 1.0f), 1.0f };
        const glm::vec4 zero4{ 0.0f, 0.0f, 0.0f, 1.0f };
        glLightfv(GL_LIGHT0 + i, GL_AMBIENT, glm::value_ptr(zero4));
        glLightfv(GL_LIGHT0 + i, GL_DIFFUSE, glm::value_ptr(color4));
        glLightfv(GL_LIGHT0 + i, GL_SPECULAR, glm::value_ptr(zero4));
        // NOTE: quadratic attenuation doesn't work like you think it would in legacy OpenGL.
        // The distance is not in world space but in NDC space!
        glLightf(GL_LIGHT0 + i, GL_CONSTANT_ATTENUATION, 1.0f);
        glLightf(GL_LIGHT0 + i, GL_LINEAR_ATTENUATION, 0.0f);
        glLightf(GL_LIGHT0 + i, GL_QUADRATIC_ATTENUATION, 0.0f);
        i++;
    };
    for (const auto& light : scene.lights) {
        std::visit(
            make_visitor(
                [&](const PointLight& light) {
                    enableLight(light.position, light.color);
                },
                [&](const SegmentLight& light) {
                    // Approximate with two point lights: one at each endpoint.
                    enableLight(light.endpoint0, 0.5f * light.color0);
                    enableLight(light.endpoint1, 0.5f * light.color1);
                },
                    [&](const ParallelogramLight& light) {
                    enableLight(light.v0, 0.25f * light.color0);
                    enableLight(light.v0 + light.edge01, 0.25f * light.color1);
                    enableLight(light.v0 + light.edge02, 0.25f * light.color2);
                    enableLight(light.v0 + light.edge01 + light.edge02, 0.25f * light.color3);
                },
                    [](auto) { /* any other type of light */ }),
            light);
    }

    // Draw the scene and the ray (if any).
    drawScene(scene);
}
