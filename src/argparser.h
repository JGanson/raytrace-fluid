// ================================================================
// Parse the command line arguments and the input file
// ================================================================

#ifndef __ARG_PARSER_H__
#define __ARG_PARSER_H__

#include "camera.h"
#include <string>
#include <random>

struct MeshData;
class Mesh;
class Fluid;
class RayTracer;
class PhotonMapping;
class BoundingBox;

// ======================================================================
// Class to collect all the high-level rendering parameters controlled
// by the command line or the keyboard input
// ======================================================================

class ArgParser {

public:

    ArgParser(int argc, const char *argv[], MeshData *_mesh_data);


    double rand() {
#if 1
        // random seed
        static std::random_device rd;    
        static std::mt19937 engine(rd());
#else
        // deterministic randomness
        static std::mt19937 engine(37);
#endif
        static std::uniform_real_distribution<double> dist(0.0, 1.0);
        return dist(engine);
    }

    // helper functions
    void separatePathAndFile(const std::string &input, std::string &path, std::string &file);

    void Load();
    void DefaultValues();

    // ==============
    // REPRESENTATION
    // all public! (no accessors)

    // Arguments
    std::string input_file = "";
    std::string path = "";
    std::string output_filename = "image.ppm";

    // Components of the scene
    MeshData *mesh_data = nullptr;
    Mesh *mesh = nullptr;
    Camera* camera = nullptr;
    Fluid *fluid = nullptr;
    RayTracer *raytracer = nullptr;
    PhotonMapping *photon_mapping = nullptr;

    Vec3f background_color;

    // Compute the global bounding box.
    BoundingBox getBoundingBox() const;
};

extern ArgParser *GLOBAL_args;
void packMesh(MeshData *mesh_data, Mesh *mesh, Fluid* fluid, RayTracer *raytracer, PhotonMapping *photonmapping);

#endif
