#ifndef _PHOTON_MAPPING_H_
#define _PHOTON_MAPPING_H_

#include <vector>

#include "photon.h"
#include "../material.h"

class Mesh;
class ArgParser;
class KDTree;
class Ray;
class Hit;
class RayTracer;

// =========================================================================
// The basic class to shoot photons within the scene and collect and
// process the nearest photons for use in the raytracer

class PhotonMapping {

public:

    // CONSTRUCTOR & DESTRUCTOR
    PhotonMapping(Mesh *_mesh, ArgParser *_args) {
        mesh = _mesh;
        args = _args;
        raytracer = NULL;
        kdtree = NULL;

    }
    ~PhotonMapping() { Clear(); }
    void setRayTracer(RayTracer *r) { raytracer = r; }

    // step 1: send the photons throughout the scene
    void TracePhotons();
    // step 2: collect the photons and return the contribution from indirect illumination
    Vec3f GatherIndirect(const Vec3f &point, const Vec3f &normal, const Vec3f &direction_from);


    void Clear();

    int triCount() const;
    int pointCount() const;
    void packMesh(float* &current, float* &current_points);

private:

    // trace a single photon
    void TracePhoton(const Vec3f &position, const Vec3f &direction, const Vec3f &energy, int iter, Material *origin_material=nullptr);

    // Collect the photons to be gathered in a convienent bin, returning the minimum radius which encloses them
    float CollectPhotons(std::vector<Photon>& result, Vec3f point, Vec3f normal);

    // REPRESENTATION
    KDTree *kdtree;
    Mesh *mesh;
    ArgParser *args;
    RayTracer *raytracer;

    int max_gathering_iters = 10;
    float photon_alignment_threshold = 0.5; // how aligned incoming photons need to be for them to be gathered
    float energy_threshold = 0.01;
};

// =========================================================================

#endif
