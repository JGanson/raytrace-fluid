#include <fstream>
#include <cstring>
#include <algorithm>

#include "fluid.h"
#include "marching_cubes.h"
#include "../argparser.h"
#include "../boundingbox.h"
#include "../utils.h"
#include "../meshdata.h"

extern MeshData *mesh_data;

// ==============================================================
// ==============================================================

// utility functions
void PackBoundingBox(float * &current,
                     const BoundingBox &bbox);

void addEdgeGeometry(float* &current,
                     const Vec3f &a, const Vec3f &b, 
                     const Vec3f &acolor, const Vec3f &bcolor, 
                     float a_th,float b_th);

void addCubeGeometry(float* &current,
                     Vec3f pts[8],
                     const Vec3f &acolor);

int Fluid::triCount() {
   int new_fluid_tri_count = 0;

    if (mesh_data->surface) {
        GenerateMarchingCubesSurface();
        new_fluid_tri_count += marchingCubes->numMCTris();
    }
    if (mesh_data->velocity) {
        if (mesh_data->dense_velocity == 0) {
            new_fluid_tri_count += 12 * (nx) * (ny) * (nz);
        } else if (mesh_data->dense_velocity == 1) {
            new_fluid_tri_count += 12 * (4*nx + 1) * (4*ny + 1);
        } else if (mesh_data->dense_velocity == 2) {
            new_fluid_tri_count += 12 * (4*nx + 1) * (4*nz + 1);
        } else {
            assert (mesh_data->dense_velocity == 3);
            new_fluid_tri_count += 12 * (4*ny + 1) * (4*nz + 1);
        }
    }
    if (mesh_data->face_velocity != 0) {
        new_fluid_tri_count += 12 * (nx) * (ny) * (nz);
    }
    if (mesh_data->pressure) {
        new_fluid_tri_count += 12 * (nx) * (ny) * (nz);
    }
    if (mesh_data->cubes) {
        new_fluid_tri_count += 12 * (nx) * (ny) * (nz);
    }
    if (mesh_data->bounding_box) {
        new_fluid_tri_count += 12 * 12;
    }

    return new_fluid_tri_count;
}

void Fluid::PackMesh(float* &current, float*& current_points) {
    if (mesh_data->particles) {
        PackFluidParticles(current_points);
    }

    if (mesh_data->surface) {
        PackFluidSurface(current);
    }
    if (mesh_data->velocity) {
        PackFluidVelocities(current);
    }
    if (mesh_data->face_velocity != 0) {
        PackFluidFaceVelocities(current);
    }
    if (mesh_data->pressure) {
        PackFluidPressures(current);
    }
    if (mesh_data->cubes) {
        PackFluidCells(current);
    }
    if (mesh_data->bounding_box) {
        PackBoundingBox(current,getBoundingBox());
    }
}

int Fluid::pointCount() {
    if (!mesh_data->particles) {
        return 0;
    }

    int count = 0;
    // count the particles
    for (int x = 0; x < nx; x++) {
        for (int y = 0; y < ny; y++) {
            for (int z = 0; z < nz; z++) {
                Cell *cell = getCell(x,y,z);
                std::vector<FluidParticle*> &particles = cell->getParticles();
                count += particles.size();
            }
        }
    }
    return count;
}

void Fluid::PackFluidParticles(float* &current) {
   for (int x = 0; x < nx; x++) {
        for (int y = 0; y < ny; y++) {
            for (int z = 0; z < nz; z++) {
                Cell *cell = getCell(x,y,z);
                std::vector<FluidParticle*> &particles = cell->getParticles();
                for (unsigned int iter = 0; iter < particles.size(); iter++) {
                    FluidParticle *p = particles[iter];
                    Vec3f v = p->getPosition();
                    float12 t = { float(v.x()),float(v.y()),float(v.z()),1,   0,0,0,0,   0,0,0,1 };
                    memcpy(current, &t, sizeof(float)*12); current += 12; 
                }
            }
        }
    }
}

void Fluid::PackFluidVelocities(float* &current) {
    float thickness = 0.01; // bb_max_dim doesn't exist any more * mesh_data->bb_max_dim;
    float dt = mesh_data->timestep;

    if (mesh_data->dense_velocity == 0) {
        // one velocity vector per cell, at the centroid
        for (int i = 0; i < nx; i++) {
            for (int j = 0; j < ny; j++) {
                for (int k = 0; k < nz; k++) {
                    Vec3f cell_center((i+0.5)*dx,(j+0.5)*dy,(k+0.5)*dz);
                    Vec3f direction(get_u_avg(i,j,k),get_v_avg(i,j,k),get_w_avg(i,j,k));
                    Vec3f pt2 = cell_center+float(100)*dt*direction;
                    addEdgeGeometry(current,
                                    cell_center,pt2,
                                    Vec3f(1,0,0),Vec3f(1,0,0),thickness,thickness*0.1);
                }
            }
        }
    }

    else {
        thickness *= 0.5;
        dt *= 0.1;

        if (mesh_data->dense_velocity == 1) {
            double z = nz*dz / 2.0;
            for (double x = 0; x <= (nx+0.01)*dx; x+=0.25*dx) {
                for (double y = 0; y <= (ny+0.01)*dy; y+=0.25*dy) {
                    Vec3f vel = getInterpolatedVelocity(Vec3f(x,y,z));
                    Vec3f pt1(x,y,z);
                    Vec3f pt2 = pt1 + float(100*dt)*vel;
                    addEdgeGeometry(current,
                                    pt1,pt2,
                                    Vec3f(1,0,0),Vec3f(1,1,0),thickness,thickness*0.1);
                } 
            }
        } else if (mesh_data->dense_velocity == 2) {
            double y = ny*dy / 2.0;
            for (double x = 0; x <= (nx+0.01)*dx; x+=0.25*dx) {
                for (double z = 0; z <= (nz+0.01)*dz; z+=0.25*dz) {
                    Vec3f vel = getInterpolatedVelocity(Vec3f(x,y,z));
                    Vec3f pt1(x,y,z);
                    Vec3f pt2 = pt1 + float(100*dt)*vel;
                    addEdgeGeometry(current,
                                    pt1,pt2,
                                    Vec3f(1,0,0),Vec3f(1,1,0),thickness,thickness*0.1);
                }
            } 
        } else if (mesh_data->dense_velocity == 3) {
            double x = nx*dx / 2.0;
            for (double y = 0; y <= (ny+0.01)*dy; y+=0.25*dy) {
                for (double z = 0; z <= (nz+0.01)*dz; z+=0.25*dz) {
                    Vec3f vel = getInterpolatedVelocity(Vec3f(x,y,z));
                    Vec3f pt1(x,y,z);
                    Vec3f pt2 = pt1 + float(100*dt)*vel;
                    addEdgeGeometry(current,
                                    pt1,pt2,
                                    Vec3f(1,0,0),Vec3f(1,1,0),thickness,thickness*0.1);
                }
            } 
        }
    }

}


void Fluid::PackFluidFaceVelocities(float* &current) {
    float thickness = 0.01; // bb_max_dim doesn't exist anymore * mesh_data->bb_max_dim;
    float dt = mesh_data->timestep;

    // =====================================================================================
    // visualize the face velocity
    // render stubby triangles to visualize the u, v, and w velocities between cell faces
    // =====================================================================================
    for (int i = 0; i < nx; i++) {
        for (int j = 0; j < ny; j++) {
            for (int k = 0; k < nz; k++) {
                double u = get_u_plus(i,j,k)*100*dt;
                double v = get_v_plus(i,j,k)*100*dt;
                double w = get_w_plus(i,j,k)*100*dt;
                double x = i*dx;
                double y = j*dy;
                double z = k*dz;
                if (mesh_data->face_velocity==1) {
                    addEdgeGeometry(current,
                                    Vec3f(x+dx  ,y+0.5*dy,z+0.5*dz),
                                    Vec3f(x+dx+u,y+0.5*dy,z+0.5*dz),
                                    Vec3f(1,0,0),Vec3f(1,0,0),thickness,thickness*0.1);
                } else if (mesh_data->face_velocity==2) {
                    addEdgeGeometry(current,
                                    Vec3f(x+0.5*dx,y+dy,z+0.5*dz),
                                    Vec3f(x+0.5*dx,y+dy+v,z+0.5*dz),
                                    Vec3f(0,1,0),Vec3f(0,1,0),thickness,thickness*0.1);
                } else if (mesh_data->face_velocity==3) {
                    addEdgeGeometry(current,
                        Vec3f(x+0.5*dx,y+0.5*dy,z+dz),
                        Vec3f(x+0.5*dx,y+0.5*dy,z+dz+w),
                        Vec3f(0,0,1),Vec3f(0,0,1),thickness,thickness*0.1);
                }
            }
        }
    }
}

void Fluid::PackFluidPressures(float* &current) {

    // =====================================================================================
    // visualize the cell pressure
    // =====================================================================================
    for (int i = 0; i < nx; i++) {
        for (int j = 0; j < ny; j++) {
            for (int k = 0; k < nz; k++) {
                Vec3f pts[8] = { Vec3f((i+0.1)*dx,(j+0.1)*dy,(k+0.1)*dz),
                    Vec3f((i+0.1)*dx,(j+0.1)*dy,(k+0.9)*dz),
                    Vec3f((i+0.1)*dx,(j+0.9)*dy,(k+0.1)*dz),
                    Vec3f((i+0.1)*dx,(j+0.9)*dy,(k+0.9)*dz),
                    Vec3f((i+0.9)*dx,(j+0.1)*dy,(k+0.1)*dz),
                    Vec3f((i+0.9)*dx,(j+0.1)*dy,(k+0.9)*dz),
                    Vec3f((i+0.9)*dx,(j+0.9)*dy,(k+0.1)*dz),
                    Vec3f((i+0.9)*dx,(j+0.9)*dy,(k+0.9)*dz) };
                double p = getCell(i,j,k)->getPressure();
                // scale the pressure
                p *= 0.01;
                if (p > 1) p = 1;
                if (p < -1) p = -1;
                assert(p >= -1 && p <= 1);
                Vec3f color;
                if (p < 0) {
                    // negative pressure is blue
                    color = Vec3f(1+p,1+p,1);
                } else {
                    // positive pressure is red
                    color = Vec3f(1,1-p,1-p);
                }
                addCubeGeometry(current,pts,color);
            }
}
    }
}

void Fluid::PackFluidCells(float* &current) {
    // =====================================================================================
    // render the MAC cells (FULL, SURFACE, or EMPTY)
    // =====================================================================================
    for (int i = 0; i < nx; i++) {
        for (int j = 0; j < ny; j++) {
            for (int k = 0; k < nz; k++) {
                Vec3f pts[8] = { Vec3f((i+0.1)*dx,(j+0.1)*dy,(k+0.1)*dz),
                    Vec3f((i+0.1)*dx,(j+0.1)*dy,(k+0.9)*dz),
                    Vec3f((i+0.1)*dx,(j+0.9)*dy,(k+0.1)*dz),
                    Vec3f((i+0.1)*dx,(j+0.9)*dy,(k+0.9)*dz),
                    Vec3f((i+0.9)*dx,(j+0.1)*dy,(k+0.1)*dz),
                    Vec3f((i+0.9)*dx,(j+0.1)*dy,(k+0.9)*dz),
                    Vec3f((i+0.9)*dx,(j+0.9)*dy,(k+0.1)*dz),
                    Vec3f((i+0.9)*dx,(j+0.9)*dy,(k+0.9)*dz) };
                Cell *cell = getCell(i,j,k);
                Vec3f color;
                if (cell->getStatus() == CELL_FULL) {
                    color = Vec3f(1,0,0);
                    addCubeGeometry(current,pts,color);
                } else if (cell->getStatus() == CELL_SURFACE) {
                    color=Vec3f(0,0,1);
                    addCubeGeometry(current,pts,color);
                } else {
                    // don't render this cell (render a box with no surface area)
                    Vec3f zero_pts[8] = { Vec3f(0,0,0),
                        Vec3f(0,0,0),
                        Vec3f(0,0,0),
                        Vec3f(0,0,0),
                        Vec3f(0,0,0),
                        Vec3f(0,0,0),
                        Vec3f(0,0,0),
                        Vec3f(0,0,0) };
                    addCubeGeometry(current,zero_pts,color);
                }
            }
        }
    }
}


MarchingCubes* Fluid::GenerateMarchingCubesSurface() {

    // =====================================================================================
    // setup a marching cubes representation of the surface
    // =====================================================================================
    for (int i = 0; i <= nx; i++) {
        for (int j = 0; j <= ny; j++) {
            for (int k = 0; k <= nz; k++) {
                marchingCubes->set(i,j,k,interpolateIsovalue(Vec3f((i-0.5),(j-0.5),(k-0.5))));
            } 
        }
    }

    marchingCubes->GenerateTris();

    return marchingCubes;
}

void Fluid::PackFluidSurface(float* &current) {

    const std::vector<MCTri> &mctris = marchingCubes->getMCTris();

    for (std::vector<MCTri>::const_iterator itr = mctris.begin(); itr < mctris.end(); itr++) {

        Vec3f a = itr->a;
        Vec3f b = itr->b;
        Vec3f c = itr->c;
        Vec3f na = itr->na;
        Vec3f nb = itr->nb;
        Vec3f nc = itr->nc;

        Vec3f color = Vec3f(0.1,0,1);

        float12 ta,tb,tc;

        ta = { float(a.x()),float(a.y()),float(a.z()),1, float(na.x()),float(na.y()),float(na.z()),0, float(color.r()),float(color.g()),float(color.b()),1 };
        tb = { float(b.x()),float(b.y()),float(b.z()),1, float(nb.x()),float(nb.y()),float(nb.z()),0, float(color.r()),float(color.g()),float(color.b()),1 };
        tc = { float(c.x()),float(c.y()),float(c.z()),1, float(nc.x()),float(nc.y()),float(nc.z()),0, float(color.r()),float(color.g()),float(color.b()),1 };

        memcpy(current, &ta, sizeof(float)*12); current += 12; 
        memcpy(current, &tb, sizeof(float)*12); current += 12; 
        memcpy(current, &tc, sizeof(float)*12); current += 12;

    }

}

// ==============================================================

double Fluid::getIsovalue(int i, int j, int k) const {
    i = std::max(0,(std::min(i,nx-1)));
    j = std::max(0,(std::min(j,ny-1)));
    k = std::max(0,(std::min(k,nz-1)));
    Cell *c = getCell(i,j,k);
    if (c->getStatus() == CELL_EMPTY) return 0;
    // note: this is technically not a correct thing to do
    //       the number of particles is not an indication of it's "fullness"
    if (c->getStatus() == CELL_SURFACE) return 0.5 + c->numParticles()/double(density);
    if (c->getStatus() == CELL_FULL) return 2;
    assert(0);
    return 0;
}

// ==============================================================

double Fluid::interpolateIsovalue(const Vec3f &v) const {

    double x = v.x();
    double y = v.y();
    double z = v.z();

    // get the values at the corners
    double a = getIsovalue(int(floor(x)),int(floor(y)),int(floor(z)));
    double b = getIsovalue(int(floor(x)),int(floor(y)),int( ceil(z)));
    double c = getIsovalue(int(floor(x)),int( ceil(y)),int(floor(z)));
    double d = getIsovalue(int(floor(x)),int( ceil(y)),int( ceil(z)));
    double e = getIsovalue(int( ceil(x)),int(floor(y)),int(floor(z)));
    double f = getIsovalue(int( ceil(x)),int(floor(y)),int( ceil(z)));
    double g = getIsovalue(int( ceil(x)),int( ceil(y)),int(floor(z)));
    double h = getIsovalue(int( ceil(x)),int( ceil(y)),int( ceil(z)));

    double x_frac = x - (floor(x));
    double y_frac = y - (floor(y));
    double z_frac = z - (floor(z));

    assert (x_frac >= 0 && x_frac <= 1);
    assert (y_frac >= 0 && y_frac <= 1);
    assert (z_frac >= 0 && z_frac <= 1);

    double answer = triInterpolate(x_frac,y_frac,z_frac,a,b,c,d,e,f,g,h);
    return answer;
}

// ==============================================================


void PackBoundingBox(float * &current,
                     const BoundingBox &bbox) {

  Vec3f min = bbox.getMin();
  Vec3f max = bbox.getMax();
  Vec3f diff = max-min;

  float thickness = diff.Length()*0.002;
  
  Vec3f pts[8] = { min + Vec3f(       0,       0,       0),
                   min + Vec3f(       0,       0,diff.z()),
                   min + Vec3f(       0,diff.y(),       0),
                   min + Vec3f(       0,diff.y(),diff.z()),
                   min + Vec3f(diff.x(),       0,       0),
                   min + Vec3f(diff.x(),       0,diff.z()),
                   min + Vec3f(diff.x(),diff.y(),       0),
                   min + Vec3f(diff.x(),diff.y(),diff.z()) };
 
  addEdgeGeometry(current,pts[0],pts[1],Vec3f(0,0,0),Vec3f(0,0,0),thickness,thickness);
  addEdgeGeometry(current,pts[2],pts[3],Vec3f(0,0,0),Vec3f(0,0,0),thickness,thickness);
  addEdgeGeometry(current,pts[4],pts[5],Vec3f(0,0,0),Vec3f(0,0,0),thickness,thickness);
  addEdgeGeometry(current,pts[6],pts[7],Vec3f(0,0,0),Vec3f(0,0,0),thickness,thickness);

  addEdgeGeometry(current,pts[0],pts[2],Vec3f(0,0,0),Vec3f(0,0,0),thickness,thickness);
  addEdgeGeometry(current,pts[1],pts[3],Vec3f(0,0,0),Vec3f(0,0,0),thickness,thickness);
  addEdgeGeometry(current,pts[4],pts[6],Vec3f(0,0,0),Vec3f(0,0,0),thickness,thickness);
  addEdgeGeometry(current,pts[5],pts[7],Vec3f(0,0,0),Vec3f(0,0,0),thickness,thickness);

  addEdgeGeometry(current,pts[0],pts[4],Vec3f(0,0,0),Vec3f(0,0,0),thickness,thickness);
  addEdgeGeometry(current,pts[1],pts[5],Vec3f(0,0,0),Vec3f(0,0,0),thickness,thickness);
  addEdgeGeometry(current,pts[2],pts[6],Vec3f(0,0,0),Vec3f(0,0,0),thickness,thickness);
  addEdgeGeometry(current,pts[3],pts[7],Vec3f(0,0,0),Vec3f(0,0,0),thickness,thickness);
}
