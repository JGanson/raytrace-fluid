#ifndef _FLUID_H_
#define _FLUID_H_

#include <cassert>
#include <vector>
#include "../argparser.h"
#include "../boundingbox.h"
#include "../meshdata.h"
#include "../material.h"
#include "cell.h"

class ArgParser;
class MarchingCubes;

// ========================================================================
// ========================================================================

class Fluid {

public:

  // ========================
  // CONSTRUCTOR & DESTRUCTOR
  Fluid(ArgParser *_args);
  ~Fluid();

  // Load fluid data
  void Load(std::ifstream& istr, Material* active_material);

  // ===============================
  // ANIMATION & RENDERING FUNCTIONS
  // PAINTING & ANIMATING
  void PackMesh(float* &current, float* &current_points);
  void PackFluidParticles(float* &current_points);
  void PackFluidSurface(float* &current);
  void PackFluidVelocities(float* &current);
  void PackFluidFaceVelocities(float* &current);
  void PackFluidPressures(float* &current);
  void PackFluidCells(float* &current);

  MarchingCubes* GenerateMarchingCubesSurface();

  int triCount();
  int pointCount();
  
  void Animate();

  BoundingBox getBoundingBox() const {
    return BoundingBox(Vec3f(0,0,0),Vec3f(nx*dx,ny*dy,nz*dz));
  }

  Material* getMaterial() { return surfaceMaterial; }
  const Material* getMaterial() const { return surfaceMaterial; }

private:

  // ==============
  // CELL ACCESSORS
  int Index(int i, int j, int k) const {
    assert (i >= -1 && i <= nx);
    assert (j >= -1 && j <= ny);
    assert (k >= -1 && k <= nz);
    return (i+1)*(ny+2)*(nz+2) + (j+1)*(nz+2) + (k+1);
  }
  Cell* getCell(int i, int j, int k) const { return &cells[Index(i,j,k)]; }

  // =================
  // ANIMATION HELPERS
  void ComputeNewVelocities();
  void SetBoundaryVelocities();
  void EmptyVelocities(int i, int j, int k);
  void CopyVelocities();
  double AdjustForIncompressibility();
  void UpdatePressures();
  void MoveParticles();
  void ReassignParticles();
  void SetEmptySurfaceFull();

  // =====================
  // NAVIER-STOKES HELPERS
  Vec3f getInterpolatedVelocity(const Vec3f &pos) const;
  double getPressure(int i, int j, int k) const { return getCell(i,j,k)->getPressure(); }
  // velocity accessors
  double get_u_plus(int i, int j, int k) const { return getCell(i,j,k)->get_u_plus(); }
  double get_v_plus(int i, int j, int k) const { return getCell(i,j,k)->get_v_plus(); }  
  double get_w_plus(int i, int j, int k) const { return getCell(i,j,k)->get_w_plus(); }  
  double get_new_u_plus(int i, int j, int k) const { return getCell(i,j,k)->get_new_u_plus(); }  
  double get_new_v_plus(int i, int j, int k) const { return getCell(i,j,k)->get_new_v_plus(); }  
  double get_new_w_plus(int i, int j, int k) const { return getCell(i,j,k)->get_new_w_plus(); }  
  double get_u_avg(int i, int j, int k) const { return 0.5*(get_u_plus(i-1,j,k)+get_u_plus(i,j,k)); }
  double get_v_avg(int i, int j, int k) const { return 0.5*(get_v_plus(i,j-1,k)+get_v_plus(i,j,k)); }
  double get_w_avg(int i, int j, int k) const { return 0.5*(get_w_plus(i,j,k-1)+get_w_plus(i,j,k)); }
  double get_uv_plus(int i, int j, int k) const { 
    return 0.5*(get_u_plus(i,j,k) + get_u_plus(i,j+1,k)) * 0.5*(get_v_plus(i,j,k) + get_v_plus(i+1,j,k)); }
  double get_uw_plus(int i, int j, int k) const { 
    return 0.5*(get_u_plus(i,j,k) + get_u_plus(i,j,k+1)) * 0.5*(get_w_plus(i,j,k) + get_w_plus(i+1,j,k)); }
  double get_vw_plus(int i, int j, int k) const { 
    return 0.5*(get_v_plus(i,j,k) + get_v_plus(i,j,k+1)) * 0.5*(get_w_plus(i,j,k) + get_w_plus(i,j+1,k)); }
  // velocity modifiers
  void set_new_u_plus(int i, int j, int k, double f) { getCell(i,j,k)->set_new_u_plus(f); }
  void set_new_v_plus(int i, int j, int k, double f) { getCell(i,j,k)->set_new_v_plus(f); }
  void set_new_w_plus(int i, int j, int k, double f) { getCell(i,j,k)->set_new_w_plus(f); }
  void adjust_new_u_plus(int i, int j, int k, double f) { getCell(i,j,k)->adjust_new_u_plus(f); }
  void adjust_new_v_plus(int i, int j, int k, double f) { getCell(i,j,k)->adjust_new_v_plus(f); }
  void adjust_new_w_plus(int i, int j, int k, double f) { getCell(i,j,k)->adjust_new_w_plus(f); }

  // ========================================
  // RENDERING SURFACE (using Marching Cubes)
  double interpolateIsovalue(const Vec3f &c) const;
  double getIsovalue(int i, int j, int k) const;

  // ============
  // LOAD HELPERS
  bool inShape(Vec3f &pos, const std::string &shape);
  void GenerateParticles(const std::string &shape, const std::string &placement);

  // don't use this constructor
  Fluid() { assert(0); }

  //helper method
  double get_interpolation_x (int i, int j, int k, double x, double y, double z) const;
  double get_interpolation_y(int i, int j, int k, double x, double y, double z) const;
  double get_interpolation_z(int i, int j, int k, double x, double y, double z) const;
  void incompressibility_surface(int i, int j, int k);
  void incompressibility_surface3D(int i, int j, int k);
  void calculate_surface(int i, int j, int k);

  // ==============
  // REPRESENTATION
  ArgParser *args;

  // fluid parameters
  int nx,ny,nz;     // number of grid cells in each dimension
  double dx,dy,dz;  // dimensions of each grid cell
  Cell *cells;      // NOTE: padded with extra cells on each side

  // simulation parameters
  bool xy_free_slip;
  bool yz_free_slip;
  bool zx_free_slip;
  bool compressible;
  double viscosity;
  double density; // average # of particles initialized in each "Full" cell

  Material* surfaceMaterial = nullptr;

  // Helper class to display an isosurface 
  MarchingCubes *marchingCubes; 
};

// ========================================================================

#endif
