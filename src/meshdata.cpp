#include <string.h>
#include <string.h>
#include <cassert>
#include <iostream>
#include <fstream>
#include <random>

#include "matrix.h"
#include "mesh.h"
#include "meshdata.h"
#include "argparser.h"
#include "camera.h"
#include "fluid/fluid.h"
#include "raytrace/raytracer.h"
#include "raytrace/raytree.h"
#include "raytrace/photon_mapping.h"




// Fluid Initialization: default values for the MeshData variables
void INIT_MeshData(MeshData *mesh_data) {
  mesh_data->width = 400;
  mesh_data->height = 400;
  mesh_data->timestep = 0.01;
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
  
  mesh_data->perspective = true;
  mesh_data->wireframe = true;
  mesh_data->gouraud = true;

  mesh_data->fluidTriCount = 0;
  mesh_data->fluidTriData = NULL;

  mesh_data->fluidPointCount = 0;
  mesh_data->fluidPointData = NULL;

  mesh_data->gravity.data[0] = 0;
  mesh_data->gravity.data[1] = -9.8;
  mesh_data->gravity.data[2] = 0;
}




// ====================================================================
// ====================================================================

// NOTE: These functions are called by the Objective-C code, so we
// need this extern to allow C code to call C++ functions (without
// function name mangling confusion).

// Also, they use global variables...  

extern "C" {

  void RayTreeActivate() {
    RayTree::Activate();
  }

  void RayTreeDeactivate() {
    RayTree::Deactivate();
  }

  void PhotonMappingTracePhotons() {
    GLOBAL_args->photon_mapping->TracePhotons();
  }


  void RaytracerClear() {
    GLOBAL_args->raytracer->pixels_a.clear();
    GLOBAL_args->raytracer->pixels_b.clear();
    GLOBAL_args->raytracer->render_to_a = true;
  }

  void PhotonMappingClear() {
    GLOBAL_args->photon_mapping->Clear();
  }
  
  void PackMesh() {
    packMesh(GLOBAL_args->mesh_data, GLOBAL_args->mesh, GLOBAL_args->fluid, GLOBAL_args->raytracer, GLOBAL_args->photon_mapping);
  }

  void Load() {
    GLOBAL_args->Load();
  }

  bool DrawPixel() {
    return (bool)RayTraceDrawPixel();
  }

  void cameraTranslate(float x, float y) {
    GLOBAL_args->camera->truckCamera(x,y);
  }
  void cameraRotate(float x, float y) {
    GLOBAL_args->camera->rotateCamera(x,y);
  }
  void cameraZoom(float y) {
    GLOBAL_args->camera->zoomCamera(y);
  }

  void TraceRay(float x, float y) {
    VisualizeTraceRay(x,y);
  }
  
  void placeCamera() {
    GLOBAL_args->camera->glPlaceCamera();
  }
  
  /*Adding for Fluid*/
  void Step(){
    if(GLOBAL_args->fluid) { GLOBAL_args->fluid->Animate(); }
  }

  /*Adding for Fluid*/
  void Animate_Fluid() {
    if(GLOBAL_args->mesh_data->animate) {
      for (int i = 0; i < 10; i++) {
        Step();
      }
      PackMesh();
    }
  }


}

// ====================================================================
// ====================================================================
