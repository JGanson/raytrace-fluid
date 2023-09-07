// ================================================================
// Parse the command line arguments and the input file
// ================================================================

#include <iostream>
#include <fstream>

#include "mesh.h"
#include "argparser.h"
#include "meshdata.h"
#include "boundingbox.h"
#include "camera.h"
#include "fluid/fluid.h"
#include "raytrace/raytracer.h"
#include "raytrace/raytree.h"
#include "raytrace/photon_mapping.h"

#if __APPLE__
#include "matrix.h"
#else
#include <glm/gtc/type_ptr.hpp>
#endif


ArgParser *GLOBAL_args;

// ================================================================

void ArgParser::DefaultValues() {
    // BASIC RENDERING PARAMETERS
    mesh_data->width = 500;
    mesh_data->height = 500;
    mesh_data->raytracing_divs_x = 1;
    mesh_data->raytracing_divs_y = 1;
    mesh_data->raytracing_x = 0;
    mesh_data->raytracing_y = 0;
    mesh_data->raytracing_animation = false;

    mesh_data->render_mode = RENDER_NORMAL;
    mesh_data->wireframe = false;

    // FLUID PARAMETERS
    mesh_data->animate = false;
    mesh_data->particles = true;
    mesh_data->surface = true;
    mesh_data->velocity = false;
    mesh_data->bounding_box = false;

    mesh_data->face_velocity = false;
    mesh_data->dense_velocity = false;
    mesh_data->isosurface = 0.5;
    mesh_data->cubes = false;
    mesh_data->pressure = false;

    mesh_data->gravity.data[0] = 0;
    mesh_data->gravity.data[1] = -9.8;
    mesh_data->gravity.data[2] = 0;

    // RAYTRACING PARAMETERS
    mesh_data->num_bounces = 0;
    mesh_data->num_shadow_samples = 0;
    mesh_data->num_antialias_samples = 1;
    mesh_data->num_glossy_samples = 1;
    mesh_data->ambient_light = {0.1f,0.1f,0.1f};
    mesh_data->intersect_backfacing = false;

    // PHOTON MAPPING PARAMETERS
    mesh_data->render_photons = true;
    mesh_data->render_photon_directions = false;
    mesh_data->render_kdtree = false;
    mesh_data->num_photons_to_shoot = 10000;
    mesh_data->num_photons_to_collect = 100;
    mesh_data->gather_indirect = false;
    mesh_data->initial_gather_radius = 0.15;

    // RENDERING GEOMETRY
    mesh_data->meshTriCount = 0;
    mesh_data->meshTriData = NULL;
    mesh_data->meshPointCount = 0;
    mesh_data->meshPointData = NULL;
    mesh_data->meshTriCount_allocated = 0;
    mesh_data->meshPointCount_allocated = 0;

    mesh_data->fluidTriCount = 0;
    mesh_data->fluidTriData = NULL;

    mesh_data->fluidPointCount = 0;
    mesh_data->fluidPointData = NULL;

    mesh_data->bounding_box_frame = false;
}


// ================================================================

// The command line arguments
ArgParser::ArgParser(int argc, const char *argv[], MeshData *_mesh_data) {
    mesh_data = _mesh_data;
    mesh = NULL;
    fluid = NULL;
    raytracer = NULL;
    photon_mapping = NULL;

    DefaultValues();

    // parse the command line arguments
    for (int i = 1; i < argc; i++) {
        std::string arg(argv[i]);

        if (arg == std::string("--input")) {

            i++; assert (i < argc); 
            separatePathAndFile(argv[i], path, input_file);

        } else if (arg == std::string("--output")) {
            i++; assert(i < argc);
            output_filename = std::string(argv[i]);
        } else if (arg == std::string("--size")) {
            i++; assert (i < argc); 
            mesh_data->width = atoi(argv[i]);
            i++; assert (i < argc); 
            mesh_data->height = atoi(argv[i]);
        } else if (arg == std::string("--num_bounces")) {
            i++; assert (i < argc); 
            mesh_data->num_bounces = atoi(argv[i]);
        } else if (arg == std::string("--num_shadow_samples")) {
            i++; assert (i < argc); 
            mesh_data->num_shadow_samples = atoi(argv[i]);
        } else if (arg == std::string("--num_antialias_samples")) {
            i++; assert (i < argc); 
            mesh_data->num_antialias_samples = atoi(argv[i]);
            assert (mesh_data->num_antialias_samples > 0);
        } else if (arg == std::string("--num_glossy_samples")) {
            i++; assert (i < argc); 
            mesh_data->num_glossy_samples = atoi(argv[i]);
            assert (mesh_data->num_glossy_samples > 0);
        } else if (arg == std::string("--ambient_light")) {
            i++; assert (i < argc);
            float r = atof(argv[i]);
            i++; assert (i < argc);
            float g = atof(argv[i]);
            i++; assert (i < argc);
            float b = atof(argv[i]);
            mesh_data->ambient_light = {r,g,b};
        } else if (arg == std::string("--intersect-backfacing")) {
            mesh_data->intersect_backfacing = true;
        }  else if (arg == std::string("--num_photons_to_shoot")) {
            i++; assert (i < argc);
            mesh_data->num_photons_to_shoot = atoi(argv[i]);
        } else if (arg == std::string("--num_photons_to_collect")) {
            i++; assert (i < argc);
            mesh_data->num_photons_to_collect = atoi(argv[i]);
        } else if (arg == std::string("--gather_indirect")) {
            mesh_data->gather_indirect = true;
        } else if (arg == std::string("--initial_gather_radius")) {
            i++; assert(i < argc);
            mesh_data->initial_gather_radius = atof(argv[i]);
        } else if (arg == std::string("--bvh_heuristic")) {
            i++; assert(i < argc);
            mesh_data->bvh_heuristic = bvh_parse_heuristic(argv[i]);
        } else {
            std::cout << "ERROR: unknown command line argument " 
                << i << ": '" << argv[i] << "'" << std::endl;
            exit(1);
        }
    }
    Load();
    GLOBAL_args = this;
    packMesh(mesh_data, mesh, fluid, raytracer, photon_mapping);
}

// ================================================================

void ArgParser::Load() {
    delete mesh;
    delete camera;
    delete fluid;
    delete raytracer;
    delete photon_mapping;

    //
    // Make all components, but do not initialize
    // 
    mesh = new Mesh(this);
    camera = nullptr;
    fluid = nullptr;
    raytracer = nullptr;
    photon_mapping = nullptr;
    background_color = Vec3f(1,1,1);

    std::string filepath = path + '/' + input_file;

    std::ifstream file(filepath.c_str());
    if (!file.good()) {
        std::cerr << "Error: can not open scene file at '" << filepath << "', quitting.\n";
        exit(1);
    }
    // parse the input file
    std::string token;
    Material* active_material = nullptr;
    Matrix transform = Matrix::Identity();

    std::cout << "Parsing '" << filepath << "'...\n";

    while (file >> token) {
        if (token == "#") {
            getline(file, token);
            continue;
        }
        if (token == "PerspectiveCamera") {
            camera = new PerspectiveCamera();
            file >> *(PerspectiveCamera*)camera;

        } else if (token == "OrthographicCamera") {
            camera = new OrthographicCamera();
            file >> *(OrthographicCamera*)camera;

        } else if (token == "background_color") {
            float r,g,b;
            file >> r >> g >> b;
            background_color = Vec3f(r,g,b);
        } else if (token == "material") {
            file >> token;
            active_material = mesh->LoadMaterial(token);
        } else if (token == "transform") {
            std::string op;
            file >> op;
            if (file.fail()) {
                std::cerr << "Expected operation following key word 'transform'.\n";
                assert(0);
            }

            if (op == "clear") {
                transform.setToIdentity();
            } else if (op == "translate") {
                double x, y, z;
                file >> x >> y >> z;
                transform *= Matrix::MakeTranslation(Vec3f(x,y,z));
            } else if (op == "scale") {
                double x, y, z;
                file >> x >> y >> z;
                transform *= Matrix::MakeScale(Vec3f(x,y,z));
            } else if (op == "matrix") {
                float values[16];
                file >> values[ 0] >> values[ 1] >> values[ 2] >> values[ 3]
                     >> values[ 4] >> values[ 5] >> values[ 6] >> values[ 7]
                     >> values[ 8] >> values[ 9] >> values[10] >> values[11]
                     >> values[12] >> values[13] >> values[14] >> values[15];
                transform *= Matrix(values);
            } else if (op == "rotate") {
                std::string kind;
                double theta, x, y, z;
                file >> kind;
                if (kind == "x") {
                    file >> theta;
                    transform *= Matrix::MakeXRotation(theta);
                } else if (kind == "y") {
                    file >> theta;
                    transform *= Matrix::MakeYRotation(theta);
                } else if (kind == "z") {
                    file >> theta;
                    transform *= Matrix::MakeZRotation(theta);
                } else if (kind == "axis") {
                    file >> x >> y >> z >> theta;
                    transform *= Matrix::MakeAxisRotation(Vec3f(x,y,z), theta);
                } else {
                    std::cerr << "Invalid rotation operation '" << kind << "'\n";
                    assert(0);
                }
            } else {
                std::cerr << "Invalid transform operation '" << op << "'\n";
                assert(0);
            }

        } else if (token == "model") {
            file >> token;
            if (token == "load") {
                // load from an external file
                file >> token;
                std::string objfilepath = path + '/' + token;
                std::ifstream objfile(objfilepath.c_str());
                if (!objfile.good()) {
                    std::cerr << "Error: can not open external obj file at '" << objfilepath << "', skipping...\n";
                    continue;
                }
                mesh->Load(objfile, active_material, transform);
                objfile.close();
            } else {
                assert (token == "{");
                // parses in-line, consumes the matching '}'
                mesh->Load(file, active_material, transform);
            }
        } else if (token == "fluid") {
            file >> token;
            delete fluid;
            fluid = new Fluid(this);
            if (token == "load") {
                // load from external file
                file >> token;
                std::string fluid_filepath = path + '/' + token;
                std::ifstream ifstr(fluid_filepath.c_str());
                if (!ifstr.good()) {
                    std::cerr << "Error: can not open external fluid file at '" << fluid_filepath << "', skipping...\n";
                    continue;
                }
                fluid->Load(ifstr, active_material);
            } else {
                assert(token == "{");
                fluid->Load(file, active_material);
            }
        }
    }

    std::cout << "done.\n";
     
    if (camera == NULL) {
        std::cout << "NO CAMERA PROVIDED, CREATING DEFAULT CAMERA" << std::endl;
        // if not initialized, position a perspective camera and scale it so it fits in the window
        auto bbox = getBoundingBox();
        Vec3f point_of_interest = bbox.getCenter();
        float max_dim = bbox.maxDim();
        Vec3f camera_position = point_of_interest + Vec3f(0,0,4*max_dim);
        Vec3f up = Vec3f(0,1,0);
        camera = new PerspectiveCamera(camera_position, point_of_interest, up, 20 * M_PI/180.0);    
    }

    raytracer = new RayTracer(mesh, this);
    photon_mapping = new PhotonMapping(mesh, this);

    raytracer->setPhotonMapping(photon_mapping);
    photon_mapping->setRayTracer(raytracer);
}

BoundingBox ArgParser::getBoundingBox() const {
    BoundingBox bbox;
    if (fluid) {
        bbox.Extend(fluid->getBoundingBox());
    }
    bbox.Extend(*mesh->getBoundingBox());
    return bbox;
}


// ================================================================

void ArgParser::separatePathAndFile(const std::string &input, std::string &path, std::string &file) {
    // we need to separate the filename from the path
    // (we assume the vertex & fragment shaders are in the same directory)
    // first, locate the last '/' in the filename
    size_t last = std::string::npos;  
    while (1) {
        int next = input.find('/',last+1);
        if (next != (int)std::string::npos) { 
            last = next;
            continue;
        }
        next = input.find('\\',last+1);
        if (next != (int)std::string::npos) { 
            last = next;
            continue;
        }
        break;
    }
    if (last == std::string::npos) {
        // if there is no directory in the filename
        file = input;
        path = ".";
    } else {
        // separate filename & path
        file = input.substr(last+1,input.size()-last-1);
        path = input.substr(0,last);
    }
}

// ================================================================

void packMesh(MeshData *mesh_data, Mesh *mesh, Fluid* fluid, RayTracer *raytracer, PhotonMapping *photonmapping) {

    GLOBAL_args->camera->glPlaceCamera();

    // new desired counts
    int triCount = 0;
    triCount += mesh? mesh->triCount() : 0;
    triCount += fluid? fluid->triCount() : 0;
    triCount += raytracer? raytracer->triCount() : 0;
    triCount += photonmapping? photonmapping->triCount() : 0;
    triCount += RayTree::triCount();
;
    int pointCount = 0;
    pointCount += fluid? fluid->pointCount() : 0;
    pointCount += photonmapping? photonmapping->pointCount() : 0;

    mesh_data->meshTriCount = triCount;
    if (mesh_data->meshTriCount > mesh_data->meshTriCount_allocated) {
        //std::cout << "resize triCount " << mesh_data->meshTriCount << std::endl;
        delete [] mesh_data->meshTriData;
        mesh_data->meshTriData = new float[2 * 12*3* mesh_data->meshTriCount];
        mesh_data->meshTriCount_allocated = 2 * triCount;
    }

    mesh_data->meshPointCount = pointCount;
    if (mesh_data->meshPointCount > mesh_data->meshPointCount_allocated) {
        //std::cout << "resize pointCount " << mesh_data->meshPointCount << std::endl;
        delete [] mesh_data->meshPointData;
        mesh_data->meshPointData = new float[2 * 12 * mesh_data->meshPointCount];
        mesh_data->meshPointCount_allocated = 2 * pointCount;
    }

    float* current = mesh_data->meshTriData;
    float* current_points = mesh_data->meshPointData;

    mesh->packMesh(current);
    raytracer->packMesh(current);
    photonmapping->packMesh(current, current_points);
    RayTree::packMesh(current);

    /******Fluid*****/
    if (fluid) {
        fluid->PackMesh(current,current_points);
        BoundingBox bbox = fluid->getBoundingBox();
        Vec3f center = bbox.getCenter();
        mesh_data->bb_center.data[0] = center.x();
        mesh_data->bb_center.data[1] = center.y();
        mesh_data->bb_center.data[2] = center.z();
        mesh_data->bb_max_dim = bbox.maxDim();
        mesh_data->bb_scale = 1.8 / float(bbox.maxDim());
    }
}

// ================================================================
