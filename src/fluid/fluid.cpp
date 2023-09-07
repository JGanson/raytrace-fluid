#include <fstream>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <iostream>

#include "fluid.h"
#include "marching_cubes.h"
#include "../argparser.h"
#include "../boundingbox.h"
#include "../utils.h"
#include "../meshdata.h"

extern MeshData *mesh_data;

#define BETA_0 1.7
#define FLUID_EPSILON 0.0001

// ==============================================================
// ==============================================================
// CONSTRUCTOR
// ==============================================================
// ==============================================================

Fluid::Fluid(ArgParser *_args) {
    args = _args;
    marchingCubes = nullptr;
}

Fluid::~Fluid() { 
    delete [] cells; 
    delete marchingCubes; 
}

double abs_(double v){
    if(v<0){
        return -v;
    }
    return v;
}
// ==============================================================

// load fluid properties from the given file, stopping after reading '}' or EOF
void Fluid::Load(std::ifstream& istr, Material* active_material) {    
    assert (istr.good());

    // fluids must be assigned a material first
    assert(active_material != nullptr);
    surfaceMaterial = active_material;


    // these must be defined
    bool defined_grid     = false;
    bool defined_cell_dim = false;
    bool defined_timestep = false;

    std::string token;

    // load in the grid size & dimensions
    istr >> token >> nx >> ny >> nz;  
    assert (nx > 0 && ny > 0 && nz > 0);
    assert (token=="grid");

    istr >> token >> dx >> dy >> dz;
    assert (token=="cell_dimensions");

    cells = new Cell[(nx+2)*(ny+2)*(nz+2)];

    istr >> token >> mesh_data->timestep;
    assert (token=="timestep");
    assert (mesh_data->timestep > 0.0);

    // simulation parameters
    istr >> token;
    assert (token=="flow");
    {
        std::string flow_kind;
        istr >> flow_kind;
        if (flow_kind == "compressible") {
            compressible = true;
        } else {
            assert (flow_kind == "incompressible");
            compressible = false;
        }
    }

    istr >> token;
    assert (token=="xy_boundary");
    {
        std::string slip_kind;
        istr >> slip_kind;
        if (slip_kind == "free_slip") {
            xy_free_slip = true;
        } else {
            assert (slip_kind == "no_slip");
            xy_free_slip = false;
        }
    }

    istr >> token;
    assert (token=="yz_boundary");
    {
        std::string slip_kind;
        istr >> slip_kind;
        if (slip_kind == "free_slip") {
            yz_free_slip = true;
        } else {
            assert  (slip_kind == "no_slip");
            yz_free_slip = false;
        }
    }

    istr >> token;
    assert (token=="zx_boundary");
    {
        std::string slip_kind;
        istr >> slip_kind;
        if (slip_kind == "free_slip") {
            zx_free_slip = true;
        } else {
            assert  (slip_kind == "no_slip");
            zx_free_slip = false;
        }
    }

    istr >> token;
    assert (token=="viscosity");
    istr >> viscosity;
    
    istr >> token;
    assert (token=="gravity");
    double gravity;
    istr >> gravity;
    mesh_data->gravity.data[0] = 0;
    mesh_data->gravity.data[1] = float(-9.8 * gravity);
    mesh_data->gravity.data[2] = 0;

    // read in custom velocities and particle positions
    density = 8;
    while(istr >> token) {
        if (token == "}") {
            break;
        }
        if (token == "#") {
            getline(istr, token);
            continue;
        }

        if (token == "density") {
            istr >> density;
            continue;
        }
        if (token == "initial_particles") {
            // place marker particles
            std::string shape, placement;
            istr >> shape >> placement;

            GenerateParticles(shape,placement);
            continue;
        }

        if (token == "initial_velocity") {
            // set the velocities everywhere
            std::string distr_kind;
            // initialize velocities
            istr >> distr_kind;  
            if (distr_kind == "zero") {
                // default is zero
            } else if (distr_kind == "random") {
                int i,j,k;
                double max_dim = std::max(dx,std::max(dy,dz));
                for (i = -1; i <= nx; i++) {
                    for (j = -1; j <= ny; j++) {
                        for (k = -1; k <= nz; k++) {

                            if (getCell(i,j,k)->getStatus() != CELL_EMPTY || getCell(i+1,j,k)->getStatus() != CELL_EMPTY) {
                                getCell(i,j,k)->set_u_plus((2*args->rand()-1)*max_dim);
                            }
                            if (getCell(i,j,k)->getStatus() != CELL_EMPTY || getCell(i,j+1,k)->getStatus() != CELL_EMPTY) {
                                getCell(i,j,k)->set_v_plus((2*args->rand()-1)*max_dim);
                            }
                            if (getCell(i,j,k)->getStatus() != CELL_EMPTY || getCell(i,j,k+1)->getStatus() != CELL_EMPTY) {
                                getCell(i,j,k)->set_w_plus((2*args->rand()-1)*max_dim);
                            }
                        }
                    }
                }
            }  else {
                std::cerr << "INVALID TOKEN '" << distr_kind << "' after 'initial_velocity'\n";
                assert(0);
            }
            continue;
        }
        // otherwise, read in a velocity
        int i,j,k;
        double velocity;
        assert (token == "u" || token == "v" || token == "w");
        istr >> i >> j >> k >> velocity;
        assert(i >= 0 && i < nx);
        assert(j >= 0 && j < ny);
        assert(k >= 0 && k < nz);
        if      (token == "u") getCell(i,j,k)->set_u_plus(velocity);
        else if (token == "v") getCell(i,j,k)->set_v_plus(velocity);
        else if (token == "w") getCell(i,j,k)->set_w_plus(velocity);
        else assert(0);
    }

    std::cout << "Fluid loaded " << (nx+2)*(ny+2)*(nz+2) << " cells.\n";

    marchingCubes = new MarchingCubes(surfaceMaterial,nx+1,ny+1,nz+1,dx,dy,dz);
    SetEmptySurfaceFull();

    SetBoundaryVelocities();
}

// ==============================================================

bool Fluid::inShape(Vec3f &pos, const std::string &shape) {
    // return true if this point is inside the "shape"
    // defined procedurally (using an implicit surface)
    if (shape == "everywhere") {
        return true;
    } else if (shape == "left") {
        // a blob of particles on the lower left (for the dam)
        return (pos.x() < 0.2*nx*dx && pos.y() < 0.5*ny*dy);
    } else if (shape == "drop") {
        // a shallow pool of particles on the bottom
        double h = ny*dy/6.0;
        if (pos.y() < 2*h) return true;
        // and a sphere of particles above
        Vec3f center = Vec3f(nx*dx*0.5, 5*h,nz*dz*0.5);
        double length = (center-pos).Length();
        if (length < 0.8*h) return true;
        return false;
    } else if (shape == "bottom") {
        // a blob of particles on the quarter (for the ripples)
        return (pos.y() < 0.25 * ny * dy);
    } else {
        std::cout << "unknown shape: " << shape << std::endl;
        exit(0);
    }
}

// ==============================================================

void Fluid::GenerateParticles(const std::string &shape, const std::string &placement) {
    // create a set of points according to the "placement" token,
    // then check whether they are inside of the "shape"
    if (placement == "uniform") {
        int dens = (int)pow(density,0.334);
        assert (dens*dens*dens == density);
        // the uniform grid spacing
        double spacing = 1/double(dens);
        for (double x = 0.5*spacing*dx; x < nx*dx; x += spacing*dx) {
            for (double y = 0.5*spacing*dy; y < ny*dy; y += spacing*dy) {
                for (double z = 0.5*spacing*dz; z < nz*dz; z += spacing*dz) {
                    Vec3f pos = Vec3f(x,y,z);
                    Cell *cell = getCell(int(x/dx),int(y/dy),int(z/dz));
                    if (inShape(pos,shape)) {
                        FluidParticle *p = new FluidParticle();
                        p->setPosition(pos);
                        cell->addParticle(p);
                    }
                }
            }
        }
    } else {
    assert (placement == "random");
    // note: we don't necessarily have the same number of particles in each cell
    for (int n = 0; n < nx*ny*nz*density; n++) {
        Vec3f pos = Vec3f(args->rand()*nx*dx,
                          args->rand()*ny*dy,
                          args->rand()*nz*dz);
        if (inShape(pos,shape)) {      
            Cell *cell = getCell(int(pos.x()/dx),int(pos.y()/dy),int(pos.z()/dz));
            FluidParticle *p = new FluidParticle();
            p->setPosition(pos);
            cell->addParticle(p);
        }
    }
}
}

// ==============================================================
// ==============================================================
// ANIMATION
// ==============================================================
// ==============================================================


void Fluid::Animate() {

  // the animation manager:  this is what gets done each timestep!
  
  ComputeNewVelocities();
  SetBoundaryVelocities();
  
  // compressible / incompressible flow
  if (compressible == false) {
    for (int iters = 0; iters < 50; iters++) {
      double max_divergence = AdjustForIncompressibility();
      SetBoundaryVelocities();


      for(int i = 0; i<nx;i++){
        for(int j=0; j<ny;j++){
          for(int k=0; k<nz; k++ ){
            
              if(nz<=1){
                incompressibility_surface(i,j,k);
              }
              else{
                incompressibility_surface3D(i,j,k);
              }
            
            
          }
        }
    }
      if (max_divergence < EPSILON) break;
    }
  }


  

  
  UpdatePressures();


  double maxv = 0;
  for (int i=0; i < nx; i++){
    for (int j=0; j<ny; j++){
      for(int k=0; k<nz;k++){
        Cell * cell = getCell(i, j, k);
        if(abs_(cell->get_new_u_plus())>maxv){
          maxv = abs_(cell->get_new_u_plus());
        }
        else if(abs_(cell->get_new_v_plus())>maxv){
          maxv = abs_(cell->get_new_v_plus());
        }
        else if(abs_(cell->get_new_w_plus())>maxv){
          maxv = abs_(cell->get_new_w_plus());
        }
      }
    }
  }
  if(maxv != 0 && dx/(2*maxv) < mesh_data->timestep){
    std::cout<<"timestep change from "<<mesh_data->timestep; 
    mesh_data->timestep = dx/(2.1*maxv);
    std::cout<<" to "<<mesh_data->timestep<<std::endl;
  }

  CopyVelocities();

  // advanced the particles through the fluid
  MoveParticles();
  ReassignParticles();
  SetEmptySurfaceFull();
}

// ==============================================================

void Fluid::ComputeNewVelocities() {
    double dt = mesh_data->timestep;
    int i,j,k;

    // using the formulas from Foster & Metaxas

    for (i = 0; i < nx-1; i++) {
        for (j = 0; j < ny; j++) {
            for (k = 0; k < nz; k++) {
                Cell *cell = getCell(i,j,k);
                double new_u_plus =
                    get_u_plus(i,j,k) +            
                    dt * ((1/dx) * (square(get_u_avg(i,j,k)) - square(get_u_avg(i+1,j,k))) +
                    (1/dy) * (get_uv_plus(i,j-1,k) - get_uv_plus(i,j,k)) + 
                    (1/dz) * (get_uw_plus(i,j,k-1) - get_uw_plus(i,j,k)) +
                    mesh_data->gravity.data[0] +
                    (1/dx) * (getPressure(i,j,k)-getPressure(i+1,j,k)) +
                    (viscosity/square(dx)) * (get_u_plus(i+1,j  ,k  ) - 2*get_u_plus(i,j,k) + get_u_plus(i-1,j  ,k  )) +
                    (viscosity/square(dy)) * (get_u_plus(i  ,j+1,k  ) - 2*get_u_plus(i,j,k) + get_u_plus(i  ,j-1,k  )) +
                    (viscosity/square(dz)) * (get_u_plus(i  ,j  ,k+1) - 2*get_u_plus(i,j,k) + get_u_plus(i  ,j  ,k-1)) );
                cell->set_new_u_plus(new_u_plus);
            }
        }
    }

    for (i = 0; i < nx; i++) {
        for (j = 0; j < ny-1; j++) {
            for (k = 0; k < nz; k++) {	
                Cell *cell = getCell(i,j,k);
                double new_v_plus =
                    get_v_plus(i,j,k) +
                    dt * ((1/dx) * (get_uv_plus(i-1,j,k) - get_uv_plus(i,j,k)) +
                    (1/dy) * (square(get_v_avg(i,j,k)) - square(get_v_avg(i,j+1,k))) +
                    (1/dz) * (get_vw_plus(i,j,k-1) - get_vw_plus(i,j,k)) +
                    mesh_data->gravity.data[1] +
                    (1/dy) * (getPressure(i,j,k)-getPressure(i,j+1,k)) +
                    (viscosity/square(dx)) * (get_v_plus(i+1,j  ,k  ) - 2*get_v_plus(i,j,k) + get_v_plus(i-1,j  ,k  )) +
                    (viscosity/square(dy)) * (get_v_plus(i  ,j+1,k  ) - 2*get_v_plus(i,j,k) + get_v_plus(i  ,j-1,k  )) +
                    (viscosity/square(dz)) * (get_v_plus(i  ,j  ,k+1) - 2*get_v_plus(i,j,k) + get_v_plus(i  ,j  ,k-1)) );
                cell->set_new_v_plus(new_v_plus);
            }
        }
    }

    for (i = 0; i < nx; i++) {
        for (j = 0; j < ny; j++) {
            for (k = 0; k < nz-1; k++) {
                Cell *cell = getCell(i,j,k);
                double new_w_plus =
                    get_w_plus(i,j,k) +
                    dt * ((1/dx) * (get_uw_plus(i-1,j,k) - get_uw_plus(i,j,k)) +
                    (1/dy) * (get_vw_plus(i,j-1,k) - get_vw_plus(i,j,k)) +
                    (1/dz) * (square(get_w_avg(i,j,k)) - square(get_w_avg(i,j,k+1))) +
                    mesh_data->gravity.data[2] +
                    (1/dz) * (getPressure(i,j,k)-getPressure(i,j,k+1)) +
                    (viscosity/square(dx)) * (get_w_plus(i+1,j  ,k  ) - 2*get_w_plus(i,j,k) + get_w_plus(i-1,j  ,k  )) +
                    (viscosity/square(dy)) * (get_w_plus(i  ,j+1,k  ) - 2*get_w_plus(i,j,k) + get_w_plus(i  ,j-1,k  )) +
                    (viscosity/square(dz)) * (get_w_plus(i  ,j  ,k+1) - 2*get_w_plus(i,j,k) + get_w_plus(i  ,j  ,k-1)) );
                cell->set_new_w_plus(new_w_plus);
            }
        }
    }
}


// ==============================================================

void Fluid::SetBoundaryVelocities() {

    // zero out flow perpendicular to the boundaries (no sources or sinks)
    for (int j = -1; j <= ny; j++) {
        for (int k = -1; k <= nz; k++) {
            getCell(-1  ,j,k)->set_u_plus(0);
            getCell(nx-1,j,k)->set_u_plus(0);
            getCell(nx  ,j,k)->set_u_plus(0);
        }
    }
    for (int i = -1; i <= nx; i++) {
        for (int k = -1; k <= nz; k++) {
            getCell(i,-1  ,k)->set_v_plus(0);
            getCell(i,ny-1,k)->set_v_plus(0);
            getCell(i,ny  ,k)->set_v_plus(0);
        }
    }
    for (int i = -1; i <= nx; i++) {
        for (int j = -1; j <= ny; j++) {
            getCell(i,j,-1  )->set_w_plus(0);
            getCell(i,j,nz-1)->set_w_plus(0);
            getCell(i,j,nz  )->set_w_plus(0);
        }
    }

    // free slip or no slip boundaries (friction with boundary)
    double xy_sign = (xy_free_slip) ? 1 : -1;
    double yz_sign = (yz_free_slip) ? 1 : -1;
    double zx_sign = (zx_free_slip) ? 1 : -1;
    for (int i = 0; i < nx; i++) {
        for (int j = -1; j <= ny; j++) {
            getCell(i,j,-1)->set_u_plus(xy_sign*getCell(i,j,0)->get_u_plus());
            getCell(i,j,nz)->set_u_plus(xy_sign*getCell(i,j,nz-1)->get_u_plus());
        }
        for (int k = -1; k <= nz; k++) {
            getCell(i,-1,k)->set_u_plus(zx_sign*getCell(i,0,k)->get_u_plus());
            getCell(i,ny,k)->set_u_plus(zx_sign*getCell(i,ny-1,k)->get_u_plus());
        }
    }
    for (int j = 0; j < ny; j++) {
        for (int i = -1; i <= nx; i++) {
            getCell(i,j,-1)->set_v_plus(xy_sign*getCell(i,j,0)->get_v_plus());
            getCell(i,j,nz)->set_v_plus(xy_sign*getCell(i,j,nz-1)->get_v_plus());
        }
        for (int k = -1; k <= nz; k++) {
            getCell(-1,j,k)->set_v_plus(yz_sign*getCell(0,j,k)->get_v_plus());
            getCell(nx,j,k)->set_v_plus(yz_sign*getCell(nx-1,j,k)->get_v_plus());
        }
    }
    for (int k = 0; k < nz; k++) {
        for (int i = -1; i <= nx; i++) {
            getCell(i,-1,k)->set_w_plus(zx_sign*getCell(i,0,k)->get_w_plus());
            getCell(i,ny,k)->set_w_plus(zx_sign*getCell(i,ny-1,k)->get_w_plus());
        }
        for (int j = -1; j <= ny; j++) {
            getCell(-1,j,k)->set_w_plus(yz_sign*getCell(0,j,k)->get_w_plus());
            getCell(nx,j,k)->set_w_plus(yz_sign*getCell(nx-1,j,k)->get_w_plus());
        }
    }
}

// ==============================================================

void Fluid::EmptyVelocities(int i, int j, int k) {
    Cell *c = getCell(i,j,k);
    if (c->getStatus() != CELL_EMPTY) return;
Cell *ciplus = getCell(i+1,j,k);
    Cell *cjplus = getCell(i,j+1,k);
    Cell *ckplus = getCell(i,j,k+1);
    if (ciplus->getStatus() == CELL_EMPTY)
        c->set_new_u_plus(0);
    if (cjplus->getStatus() == CELL_EMPTY)
        c->set_new_v_plus(0);
    if (ckplus->getStatus() == CELL_EMPTY)
        c->set_new_w_plus(0);
}


// move to new timestep
void Fluid::CopyVelocities() {
    double dt = mesh_data->timestep;
    for (int i = 0; i < nx; i++) {
        for (int j = 0; j < ny; j++) {
            for (int k = 0; k < nz; k++) {
                Cell *c = getCell(i,j,k);

                EmptyVelocities(i,j,k);

                c->copyVelocity();
                if (fabs(c->get_u_plus()) > 0.5*dx/dt ||
                    fabs(c->get_v_plus()) > 0.5*dy/dt ||
                    fabs(c->get_w_plus()) > 0.5*dz/dt) {
                    // velocity has exceeded reasonable threshhold
                    std::cout << "velocity has exceeded reasonable threshhold, stopping animation" << std::endl;
                    mesh_data->animate=false;
                }
            }
        }
    }
}

// ==============================================================

double pow2(double num){
    return num * num;
}
double Fluid::AdjustForIncompressibility() {


    // *********************************************************************  
    // ASSIGNMENT:
    //
    // This is not a complete implementation of the Marker and Cell (MAC) method.
    // Additional boundary velocities should be equalized as described in the references
    // depending on whether the boundaries are free-slip or no-slip.
    //
    // Also play around with compressible flow!
    //
    // *********************************************************************    

    double max = -INFINITY;
    double dt = mesh_data->timestep;
    for(int i = 0; i<nx;i++){
        for(int j = 0; j<ny; j++){
            for(int k = 0; k<nz; k++){
                Cell* c = getCell(i,j,k);
                if(c->getStatus()==CELL_FULL){

                    double divergence = -((1/dx)*(get_new_u_plus(i,j,k)-get_new_u_plus(i-1,j,k))+ 
                        (1/dy)*(get_new_v_plus(i,j,k)-get_new_v_plus(i,j-1,k))+
                        (1/dz)*(get_new_w_plus(i,j,k)-get_new_w_plus(i,j,k-1)));

                    double beta = BETA_0 /(2*dt*(pow2(1/dx) + pow2(1/dy)+pow2(1/dz)));
                    double del_p = beta*divergence;


                    set_new_u_plus(i, j, k, get_new_u_plus(i,j,k)+(dt/dx)*del_p);
                    set_new_u_plus(i-1,j,k, get_new_u_plus(i-1,j,k)-(dt/dx)*del_p);

                    set_new_v_plus(i, j, k, get_new_v_plus(i,j,k)+(dt/dy)*del_p);
                    set_new_v_plus(i,j-1,k, get_new_v_plus(i,j-1,k)-(dt/dy)*del_p);

                    set_new_w_plus(i, j, k, get_new_w_plus(i,j,k)+(dt/dz)*del_p);
                    set_new_w_plus(i,j,k-1, get_new_w_plus(i,j,k-1)-(dt/dz)*del_p);


                    c->setPressure(0);
                    if(divergence >max ){
                        max = divergence;
                    }
                }
            }
}
    }
    //std::cout<<max<<std::endl;
    // return the maximum divergence
    // (will iterate for specified # of iterations or until divergence is near zero)

    // placeholder...
    return max;
}

// ==============================================================

void Fluid::UpdatePressures() {
    for (int i = -1; i <= nx; i++) {
        for (int j = -1; j <= ny; j++) {
            for (int k = -1; k <= nz; k++) {
                Cell *c = getCell(i,j,k);
                if (i >= 0 && i < nx && j >= 0 && j < ny && k >= 0 && k < nz) {
                    // compute divergence and increment/decrement pressure
                    double pressure = c->getPressure();
                    double divergence = 
                        - ( (1/dx) * (get_new_u_plus(i,j,k) - get_new_u_plus(i-1,j,k)) +
                        (1/dy) * (get_new_v_plus(i,j,k) - get_new_v_plus(i,j-1,k)) +
                        (1/dz) * (get_new_w_plus(i,j,k) - get_new_w_plus(i,j,k-1)) );
                    double dt = mesh_data->timestep;
                    double beta = BETA_0/((2*dt) * (1/square(dx) + 1/square(dy) + 1/square(dz)));
                    double dp = beta*divergence;
                    c->setPressure(pressure + dp);
                } else {
                    // zero out boundary cells (just in case)
                    c->setPressure(0);
                }


                // =======================================
                // HACK? From Foster 2001 paper?
                // zero out empty cells
                if (c->getStatus() == CELL_EMPTY) {
                    c->setPressure(0);
                }
                // ========================================

            }
        }
    }
}

// ==============================================================

void Fluid::MoveParticles() {
    double dt = mesh_data->timestep;
    for (int i = 0; i < nx; i++) {
        for (int j = 0; j < ny; j++) {
            for (int k = 0; k < nz; k++) {
                Cell *cell = getCell(i,j,k);
                std::vector<FluidParticle*> &particles = cell->getParticles();
                for (unsigned int iter = 0; iter < particles.size(); iter++) {
                    FluidParticle *p = particles[iter];
                    Vec3f pos = p->getPosition();


                    //Vec3f vel = getInterpolatedVelocity(pos);
                    //Vec3f pos2 = pos + float(dt)*vel;


                    Vec3f vel_a = getInterpolatedVelocity(pos);
                    Vec3f pos_a = pos + vel_a*dt;

                    Vec3f vel_b = getInterpolatedVelocity(pos_a);
                    Vec3f pos_b = pos_a + vel_b*dt;

                    Vec3f final = (pos_b - pos)*0.5 + pos; 

                    // *********************************************************************
                    // OPTIONAL ASSIGNMENT:
                    //
                    // Replacing Euler with Trapezoid or Runge Kutta will have
                    // better visual results for tight spiral flows.
                    //

                    // euler integration
                    p->setPosition(final);

                    // *********************************************************************

                }
            }
        }
    }
}

// ==============================================================

void Fluid::ReassignParticles() {
    for (int i = 0; i < nx; i++) {
        for (int j = 0; j < ny; j++) {
            for (int k = 0; k < nz; k++) {
                Cell *cell = getCell(i,j,k);
                std::vector<FluidParticle*> &particles = cell->getParticles();
                for (unsigned int iter = 0; iter < particles.size(); iter++) {
                    FluidParticle *p = particles[iter];
                    Vec3f pos = p->getPosition();
                    int i2 = (int)std::min(double(nx-1),std::max(0.0,floor(pos.x()/dx)));
                    int j2 = (int)std::min(double(ny-1),std::max(0.0,floor(pos.y()/dy)));
                    int k2 = (int)std::min(double(nz-1),std::max(0.0,floor(pos.z()/dz)));
                    // if the particle has crossed one of the cell faces 
                    // assign it to the new cell
                    if (i != i2 || j != j2 || k != k2) {
                        cell->removeParticle(p);
                        getCell(i2,j2,k2)->addParticle(p);
                    } 
                }
            }
        }
    }
}

// ==============================================================

void Fluid::SetEmptySurfaceFull() {
    int i,j,k;
    for (i = 0; i < nx; i++) {
        for (j = 0; j < ny; j++) {
            for (k = 0; k < nz; k++) {
                Cell *cell = getCell(i,j,k);
                if (cell->numParticles() == 0)
                    cell->setStatus(CELL_EMPTY);
                else 
                    cell->setStatus(CELL_FULL);
            }
        }
    }

    // pick out the boundary cells
    for (i = 0; i < nx; i++) {
        for (j = 0; j < ny; j++) {
            for (k = 0; k < nz; k++) {
                Cell *cell = getCell(i,j,k);
                if (cell->getStatus() == CELL_FULL &&
                    (getCell(i-1,j,k)->getStatus() == CELL_EMPTY ||
                    getCell(i+1,j,k)->getStatus() == CELL_EMPTY ||
                    getCell(i,j-1,k)->getStatus() == CELL_EMPTY ||
                    getCell(i,j+1,k)->getStatus() == CELL_EMPTY ||
                    getCell(i,j,k-1)->getStatus() == CELL_EMPTY ||
                    getCell(i,j,k+1)->getStatus() == CELL_EMPTY)) {
                    cell->setStatus(CELL_SURFACE);
                }
            }
        }
    }
}

//as y goes right, z goes up, x goes out of screen
double Fluid::get_interpolation_x(int i, int j, int k, double x, double y, double z) const{

    double up_right_front, up_right_back ,up_left_front, up_left_back;
    double bot_right_front, bot_right_back ,bot_left_front, bot_left_back; 
    double x_ratio_front, x_ratio_back, y_ratio_left, y_ratio_right, z_ratio_bot, z_ratio_up;
    //take left bot
    if(abs_(y - j*dy) <=(dy/2) && abs_(z - k*dz)<=(dz/2)){
        up_right_front = get_u_plus(i,j,k);up_right_back = get_u_plus(i-1, j, k);
        up_left_front = get_u_plus(i,j-1,k);up_left_back = get_u_plus(i-1, j-1,k);
        bot_right_front = get_u_plus(i,j,k-1);bot_right_back = get_u_plus(i-1, j,k-1);
        bot_left_front = get_u_plus(i,j-1,k-1);bot_left_back= get_u_plus(i-1,j-1,k-1); 

        x_ratio_front = 1- abs_((i+1)*dx-x)/dx; x_ratio_back = abs_(1-x_ratio_front);
        y_ratio_left = 1 - abs_(y-(j-0.5)*dy)/dy; y_ratio_right = abs_(1-y_ratio_left);
        z_ratio_bot = 1 - abs_(z - (k-0.5)*dz)/dz; z_ratio_up = abs_(1-z_ratio_bot);
    }
        //take left top, 
    else if((y - j*dy) <=(dy/2) && (z - k*dz)>(dz/2)){
        up_right_front = get_u_plus(i,j,k+1);up_right_back = get_u_plus(i-1, j, k+1);
        up_left_front = get_u_plus(i,j-1,k+1);up_left_back = get_u_plus(i-1, j-1,k+1);
        bot_right_front = get_u_plus(i,j,k); bot_right_back = get_u_plus(i-1, j,k);
        bot_left_front = get_u_plus(i,j-1,k); bot_left_back= get_u_plus(i-1,j-1,k); 

        x_ratio_front = 1- abs_((i+1)*dx-x)/dx; x_ratio_back = abs_(1-x_ratio_front);
        y_ratio_left = 1 - abs_(y-(j-0.5)*dy)/dy; y_ratio_right = abs_(1-y_ratio_left);
        z_ratio_bot = 1 - abs_(z - (k+0.5)*dz)/dz; z_ratio_up = abs_(1-z_ratio_bot);
    }
    //take right bot
    else if((y - j*dy) >(dy/2) && (z - k*dz)<=(dz/2)){
        up_right_front = get_u_plus(i,j+1,k);up_right_back = get_u_plus(i-1, j+1, k);
        up_left_front = get_u_plus(i,j,k);up_left_back = get_u_plus(i-1, j,k);
        bot_right_front = get_u_plus(i,j+1,k-1);bot_right_back = get_u_plus(i-1, j+1,k-1);
        bot_left_front = get_u_plus(i,j,k-1);bot_left_back= get_u_plus(i-1,j,k-1); 

        x_ratio_front = 1- abs_((i+1)*dx-x)/dx; x_ratio_back = abs_(1-x_ratio_front);
        y_ratio_left = 1 - abs_(y-(j+0.5)*dy)/dy; y_ratio_right = abs_(1-y_ratio_left);
        z_ratio_bot = 1 - abs_(z - (k-0.5)*dz)/dz; z_ratio_up = abs_(1-z_ratio_bot);
    }
    //take right up
    else{
        up_right_front = get_u_plus(i,j+1,k+1);up_right_back = get_u_plus(i-1, j+1, k+1);
        up_left_front = get_u_plus(i,j,k+1);up_left_back = get_u_plus(i-1, j,k+1);
        bot_right_front = get_u_plus(i,j+1,k);bot_right_back = get_u_plus(i-1, j+1,k);
        bot_left_front = get_u_plus(i,j,k);bot_left_back= get_u_plus(i-1,j,k); 

        x_ratio_front = 1- abs_((i+1)*dx-x)/dx; x_ratio_back = abs_(1-x_ratio_front);
        y_ratio_left = 1 - abs_(y-(j+0.5)*dy)/dy; y_ratio_right = abs_(1-y_ratio_left);
        z_ratio_bot = 1 - abs_(z-(k+0.5)*dz)/dz; z_ratio_up = abs_(1-z_ratio_bot);
    }
    return x_ratio_front*(
        z_ratio_up* (y_ratio_left * up_left_front + y_ratio_right *up_right_front) + 
        z_ratio_bot*(y_ratio_right *bot_right_front + y_ratio_left *bot_left_front)
        )+
        x_ratio_back * (
            z_ratio_up* (y_ratio_right *up_right_back + y_ratio_left *up_left_back)+
            z_ratio_bot*(y_ratio_right *bot_right_back + y_ratio_left *bot_left_back)
        );
}

//as x goes right, z goes up, y goes OUT of screen
double Fluid::get_interpolation_y(int i, int j, int k, double x, double y, double z) const{
    double up_right_front, up_right_back ,up_left_front, up_left_back;
    double bot_right_front, bot_right_back ,bot_left_front, bot_left_back; 
    double x_ratio_right, x_ratio_left, y_ratio_front, y_ratio_back, z_ratio_bot, z_ratio_up;
    //take left bot
    if((x - i*dx) <=(dx/2) && (z - k*dz)<=(dz/2)){
        up_right_front = get_v_plus(i,j,k);up_right_back = get_v_plus(i, j-1, k);
        up_left_front = get_v_plus(i-1,j,k);up_left_back = get_v_plus(i-1, j-1,k);
        bot_right_front = get_v_plus(i,j,k-1);bot_right_back = get_v_plus(i, j-1,k-1);
        bot_left_front = get_v_plus(i-1,j,k-1);bot_left_back= get_v_plus(i-1,j-1,k-1); 

        x_ratio_left = 1- abs_(x - (i-0.5)*dx)/dx; x_ratio_right = abs_(1-x_ratio_left);
        y_ratio_front = 1 - abs_((j+1)*dy-y)/dy; y_ratio_back = abs_(1-y_ratio_front);
        z_ratio_bot = 1 - abs_(z - (k-0.5)*dz)/dz; z_ratio_up = abs_(1-z_ratio_bot);
    }
        //take left top, 
    else if((x - i*dx) <=(dx/2) && (z - k*dz)>(dz/2)){
        up_right_front = get_v_plus(i,j,k+1);up_right_back = get_v_plus(i, j-1, k+1);
        up_left_front = get_v_plus(i-1,j,k+1);up_left_back = get_v_plus(i-1, j-1,k+1);
        bot_right_front = get_v_plus(i,j,k);bot_right_back = get_v_plus(i, j-1,k);
        bot_left_front = get_v_plus(i-1,j,k);bot_left_back= get_v_plus(i-1,j-1,k); 

        x_ratio_left = 1- abs_(x - (i-0.5)*dx)/dx; x_ratio_right = abs_(1-x_ratio_left);
        y_ratio_front = 1 - abs_((j+1)*dy-y)/dy; y_ratio_back = abs_(1-y_ratio_front);
        z_ratio_bot = 1 - abs_(z - (k+0.5)*dz)/dz; z_ratio_up = abs_(1-z_ratio_bot);
    }
        //take right bot
    else if((x - i*dx) >(dx/2) && (z - k*dz)<=(dz/2)){
        up_right_front = get_v_plus(i+1,j,k);up_right_back = get_v_plus(i+1, j-1, k);
        up_left_front = get_v_plus(i,j,k);up_left_back = get_v_plus(i, j-1,k);
        bot_right_front = get_v_plus(i+1,j,k-1);bot_right_back = get_v_plus(i+1, j-1,k-1);
        bot_left_front = get_v_plus(i,j,k-1);bot_left_back= get_v_plus(i,j-1,k-1); 

        x_ratio_left = 1- abs_(x - (i+0.5)*dx)/dx; x_ratio_right = abs_(1-x_ratio_left);
        y_ratio_front = 1 - abs_((j+1)*dy-y)/dy; y_ratio_back = abs_(1-y_ratio_front);
        z_ratio_bot = 1 - abs_(z - (k-0.5)*dz)/dz; z_ratio_up = abs_(1-z_ratio_bot);
    }
    //take right up
    else{
    up_right_front = get_v_plus(i+1,j,k+1);up_right_back = get_v_plus(i+1, j-1, k+1);
    up_left_front = get_v_plus(i,j,k+1);up_left_back = get_v_plus(i, j-1,k+1);
    bot_right_front = get_v_plus(i+1,j,k);bot_right_back = get_v_plus(i+1, j-1,k);
    bot_left_front = get_v_plus(i,j,k);bot_left_back= get_v_plus(i,j-1,k); 

    x_ratio_left = 1- abs_(x - (i+0.5)*dx)/dx; x_ratio_right = abs_(1-x_ratio_left);
    y_ratio_front = 1 - abs_((j+1)*dy-y)/dy; y_ratio_back = abs_(1-y_ratio_front);
    z_ratio_bot = 1 - abs_(z-(k+0.5)*dz)/dz; z_ratio_up = abs_(1-z_ratio_bot);
}
    return y_ratio_front*(
            z_ratio_up* (x_ratio_left * up_left_front + x_ratio_right *up_right_front) + 
            z_ratio_bot*(x_ratio_right *bot_right_front + x_ratio_left *bot_left_front)
            )+
                y_ratio_back * (
            z_ratio_up* (x_ratio_right *up_right_back + x_ratio_left *up_left_back)+
                z_ratio_bot*(x_ratio_right *bot_right_back + x_ratio_left *bot_left_back)
            );
}

//z go out of screen, x goes right, y goes up
double Fluid::get_interpolation_z(int i, int j, int k, double x, double y, double z) const{
    double up_right_front, up_right_back ,up_left_front, up_left_back;
    double bot_right_front, bot_right_back ,bot_left_front, bot_left_back; 
    double x_ratio_right, x_ratio_left, y_ratio_up, y_ratio_bot, z_ratio_front, z_ratio_back;
    //take left bot
    if((x - i*dx) <=(dx/2) && (y - j*dy)<=(dy/2)){
up_right_front = get_w_plus(i,j,k);up_right_back = get_w_plus(i, j, k-1);
        up_left_front = get_w_plus(i-1,j,k);up_left_back = get_w_plus(i-1, j,k-1);
        bot_right_front = get_w_plus(i,j-1,k);bot_right_back = get_w_plus(i, j-1,k-1);
bot_left_front = get_w_plus(i-1,j-1,k);bot_left_back= get_w_plus(i-1,j-1,k-1); 

x_ratio_left = 1- abs_(x - (i-0.5)*dx)/dx; x_ratio_right = abs_(1-x_ratio_left);
        y_ratio_bot = 1 - abs_(y- (j-0.5)*dy)/dy; y_ratio_up = abs_(1-y_ratio_bot);
        z_ratio_front = 1 - abs_((k+1)*dz - z)/dz; z_ratio_back = abs_(1-z_ratio_front);
    }
        //take left top, 
    else if((x - i*dx) <=(dx/2) && (y - j*dy)>(dy/2)){
up_right_front = get_w_plus(i,j+1,k);up_right_back = get_w_plus(i, j+1, k-1);
        up_left_front = get_w_plus(i-1,j+1,k);up_left_back = get_w_plus(i-1, j+1,k-1);
        bot_right_front = get_w_plus(i,j,k);bot_right_back = get_w_plus(i, j,k-1);
        bot_left_front = get_w_plus(i-1,j,k);bot_left_back= get_w_plus(i-1,j,k-1); 

        x_ratio_left = 1- abs_(x - (i-0.5)*dx)/dx; x_ratio_right = abs_(1-x_ratio_left);
        y_ratio_bot = 1 - abs_(y- (j+0.5)*dy)/dy; y_ratio_up = abs_(1-y_ratio_bot);
        z_ratio_front = 1 - abs_((k+1)*dz - z)/dz; z_ratio_back = abs_(1-z_ratio_front);
    }
//take right bot
    else if((x - i*dx) >(dx/2) && (y - j*dy)<=(dy/2)){
        up_right_front = get_w_plus(i+1,j,k);up_right_back = get_w_plus(i+1, j, k-1);
        up_left_front = get_w_plus(i,j,k);up_left_back = get_w_plus(i, j,k-1);
        bot_right_front = get_w_plus(i+1,j-1,k);bot_right_back = get_w_plus(i+1, j-1,k-1);
        bot_left_front = get_w_plus(i,j-1,k);bot_left_back= get_w_plus(i,j-1,k-1); 

x_ratio_left = 1- abs_(x - (i+0.5)*dx)/dx; x_ratio_right = abs_(1-x_ratio_left);
y_ratio_bot = 1 - abs_(y- (j-0.5)*dy)/dy; y_ratio_up = abs_(1-y_ratio_bot);
z_ratio_front = 1 - abs_((k+1)*dz - z)/dz; z_ratio_back = abs_(1-z_ratio_front);
    }
        //take right up
    else{
    up_right_front = get_w_plus(i+1,j+1,k);up_right_back = get_w_plus(i+1, j+1, k-1);
    up_left_front = get_w_plus(i,j+1,k);up_left_back = get_w_plus(i, j+1,k-1);
    bot_right_front = get_w_plus(i+1,j,k);bot_right_back = get_w_plus(i+1, j,k-1);
    bot_left_front = get_w_plus(i,j,k);bot_left_back= get_w_plus(i,j,k-1); 

    x_ratio_left = 1- abs_(x - (i+0.5)*dx)/dx; x_ratio_right = abs_(1-x_ratio_left);
    y_ratio_bot = 1 - abs_(y- (j+0.5)*dy)/dy; y_ratio_up = abs_(1-y_ratio_bot);
    z_ratio_front = 1 - abs_((k+1)*dz - z)/dz; z_ratio_back = abs_(1-z_ratio_front);
}
    return z_ratio_front*(
y_ratio_up* (x_ratio_left * up_left_front + x_ratio_right *up_right_front) + 
    y_ratio_bot*(x_ratio_right *bot_right_front + x_ratio_left *bot_left_front)
)+
z_ratio_back * (
    y_ratio_up* (x_ratio_right *up_right_back + x_ratio_left *up_left_back)+
y_ratio_bot*(x_ratio_right *bot_right_back + x_ratio_left *bot_left_back)
);
}
// ==============================================================

Vec3f Fluid::getInterpolatedVelocity(const Vec3f &pos) const {


// *********************************************************************  
    // ASSIGNMENT:
    //
    // Here is the naive velocity interpolation.
    // (use a simple average of the face velocity throughout the cell)
// Do it right, as described in the papers.


    int i = int(floor(pos.x()/dx)); if (i < 0) i = 0; if (i >= nx) i = nx-1;
    int j = int(floor(pos.y()/dy)); if (j < 0) j = 0; if (j >= ny) j = ny-1;
    int k = int(floor(pos.z()/dz)); if (k < 0) k = 0; if (k >= nz) k = nz-1;

    //get_interpolation_x(i,j,k,pos.x(),pos.y(),pos.z());
    //std::cout<<get_interpolation_x(i,j,k,pos.x(),pos.y(),pos.z())<<std::endl;
    return Vec3f(
    //get_u_avg(i,j,k)
        get_interpolation_x(i,j,k,pos.x(),pos.y(),pos.z())
        ,get_interpolation_y(i,j,k,pos.x(),pos.y(),pos.z())
        //,get_v_avg(i,j,k)
,get_interpolation_z(i,j,k,pos.x(),pos.y(),pos.z()));
    //return Vec3f(get_u_avg(i,j,k),get_v_avg(i,j,k),get_w_avg(i,j,k));
    //
// *********************************************************************  

}
