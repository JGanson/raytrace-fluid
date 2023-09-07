#ifndef _MESH_DATA_H_
#define _MESH_DATA_H_

// ====================================================================

#include "raytrace/bvh.h"
#define NUM_RENDER_MODES 1
enum RENDER_MODE {
    RENDER_NORMAL
};

// ====================================================================

// a homogeneous 3D point or a color with alpha
typedef struct float3 {
    float data[3];
} float3;

// a homogenous 3D point or a color with alpha
typedef struct float4 {
    float data[4];
} float4;

// a vertex with position, normal, and color
typedef struct float12 {
    float data[12];
} float12;

// a 4x4 matrix
typedef struct float16 {
    float data[16];
} float16;


typedef struct MeshData {

    // REPRESENTATION
    int width;
    int height;

    // animation control
    bool raytracing_animation;
    bool wireframe;
    RENDER_MODE render_mode;

    // FLUID PARAMETERS
    bool animate;
    float timestep;
    float3 gravity;
    bool particles;
    bool velocity;
    bool surface;
    bool bounding_box;
    bool gouraud;
    bool perspective;

    int face_velocity;
    int dense_velocity;
    double isosurface;
    bool cubes;
    bool pressure;

    // RAYTRACING PARAMETERS
    int num_bounces;
    int num_shadow_samples;
    int num_antialias_samples;
    int num_glossy_samples;
    float3 ambient_light;
    bool intersect_backfacing;
    int raytracing_divs_x;
    int raytracing_divs_y;
    int raytracing_x;
    int raytracing_y;

    // BVH PARAMETERS
    bool render_bvh = false;
    BVH_Heuristic bvh_heuristic = BVH_DEFAULT_HEURISTIC;

    // PHOTON MAPPING PARAMETERS
    int num_photons_to_shoot;
    int num_photons_to_collect;
    bool render_photons;
    bool render_photon_directions;
    bool render_kdtree;
    bool gather_indirect;
    float initial_gather_radius;

    bool bounding_box_frame;


   
    int fluidTriCount;
    float* fluidTriData;
    int fluidPointCount;
    float* fluidPointData;

    int meshTriCount;
    float* meshTriData;
    int meshPointCount;
    float* meshPointData;

    int meshTriCount_allocated;
    int meshPointCount_allocated;

    float3 bb_center;
    float bb_max_dim;
    float bb_scale;
    float16 proj_mat;
    float16 view_mat;

} MeshData;


void INIT_MeshData(MeshData *mesh_data);
void loadOBJ(MeshData *mesh_data);

// ====================================================================
// ====================================================================

#endif
