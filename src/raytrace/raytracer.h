#ifndef _RAY_TRACER_
#define _RAY_TRACER_

#include <vector>
#include "../image.h"
#include "../meshdata.h"
#include "../primitive.h"
#include "ray.h"
#include "hit.h"
#include "bvh.h"

class Mesh;
class ArgParser;
class PhotonMapping;

// ====================================================================
// ====================================================================
// This class manages the ray casting and ray tracing work.

class Pixel {
public:
    Vec3f v1,v2,v3,v4;
    Vec3f color;
};


class RayTracer {

public:

    // CONSTRUCTOR & DESTRUCTOR
    RayTracer(Mesh *m, ArgParser *a) {
        mesh = m;
        args = a;
        render_to_a = true;
        SetupRaytracedGeometry();
    }  
    // set access to the other modules for hybrid rendering options
    void setPhotonMapping(PhotonMapping *pm) { photon_mapping = pm; }

    // Recompute the primitives in the scene
    void SetupRaytracedGeometry();

    // TraceRay traces the ray throughout the scene, returning the color that is hit.
    // If there is a hit, then the information is written out to 'hit'.
    Vec3f TraceRay(Ray &ray, Hit &hit, int bounce_count = 0, Material* origin_material = nullptr) const;

    // Get the BVH tree used for scene geometry
    const BVH* getBVH() const { return bvh; } 

private:

    void drawVBOs_a();
    void drawVBOs_b();

    // REPRESENTATION
    Mesh *mesh;
    ArgParser *args;
    PhotonMapping *photon_mapping;
    BVH *bvh = nullptr;

    std::vector<Primitive*> primitives;

public:
    bool render_to_a;

    std::vector<Pixel> pixels_a;
    std::vector<Pixel> pixels_b;

    float exclusion_radius = 0.01; // the radius around the origin of a ray cast to ignore all intersections

    int triCount();
    void packMesh(float* &current);
    void packPixels(float* &current);
};

// ====================================================================
// ====================================================================

int RayTraceDrawPixel();

Vec3f VisualizeTraceRay(double i, double j);

Image DrawRayTracedImage();


#endif
