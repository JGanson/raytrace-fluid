#include "marching_cubes.h"
#include "../utils.h"
#include "../raytrace/ray.h"
#include "../raytrace/hit.h"
#include "fluid.h"

// ============================================================================
// ============================================================================


void MarchingCubes::GenerateTris() {
  double isosurface = 0.5;

  mc_tris.clear();

  for (int i = 0; i < nx-1; i++) {
    for (int j = 0; j < ny-1; j++) {
      for (int k = 0; k < nz-1; k++) {
	GridValue v[8];
	double eps = 0;//dx * 0.05;
	v[0] = GridValue(Vec3f(dx*i    +eps,dy*j    +eps,dz*k    +eps),getNormal(i  ,j  ,k  ),get(i  ,j  ,k  ));
	v[1] = GridValue(Vec3f(dx*i    +eps,dy*j    +eps,dz*(k+1)-eps),getNormal(i  ,j  ,k+1),get(i  ,j  ,k+1));
	v[2] = GridValue(Vec3f(dx*i    +eps,dy*(j+1)-eps,dz*k    +eps),getNormal(i  ,j+1,k  ),get(i  ,j+1,k  ));
	v[3] = GridValue(Vec3f(dx*i    +eps,dy*(j+1)-eps,dz*(k+1)-eps),getNormal(i  ,j+1,k+1),get(i  ,j+1,k+1));
	v[4] = GridValue(Vec3f(dx*(i+1)-eps,dy*j    +eps,dz*k    +eps),getNormal(i+1,j  ,k  ),get(i+1,j  ,k  ));
	v[5] = GridValue(Vec3f(dx*(i+1)-eps,dy*j    +eps,dz*(k+1)-eps),getNormal(i+1,j  ,k+1),get(i+1,j  ,k+1));
	v[6] = GridValue(Vec3f(dx*(i+1)-eps,dy*(j+1)-eps,dz*k    +eps),getNormal(i+1,j+1,k  ),get(i+1,j+1,k  ));
	v[7] = GridValue(Vec3f(dx*(i+1)-eps,dy*(j+1)-eps,dz*(k+1)-eps),getNormal(i+1,j+1,k+1),get(i+1,j+1,k+1));
	// need to alternate orientation of central tetrahedron to ensure that the diagonals line up
	if ((i+j+k)%2) {
	  PaintTetra(v[0],v[5],v[3],v[6],isosurface);
	  PaintTetra(v[0],v[1],v[3],v[5],isosurface);
	  PaintTetra(v[0],v[4],v[5],v[6],isosurface);
	  PaintTetra(v[0],v[2],v[6],v[3],isosurface);
	  PaintTetra(v[3],v[7],v[6],v[5],isosurface);
	} else {
	  PaintTetra(v[2],v[4],v[1],v[7],isosurface);
	  PaintTetra(v[5],v[4],v[7],v[1],isosurface);
	  PaintTetra(v[2],v[3],v[7],v[1],isosurface);
	  PaintTetra(v[4],v[2],v[1],v[0],isosurface);
	  PaintTetra(v[2],v[7],v[6],v[4],isosurface);
	}
      }
    }
  }

}


// ============================================================================

void MarchingCubes::PaintTetra(const GridValue &a, const GridValue &b, const GridValue &c, const GridValue &d, double isosurface)  {
  // figure out how many of the vertices are on each side of the isosurface
  int count = 0;
  if (a.value > isosurface) count++;
  if (b.value > isosurface) count++;
  if (c.value > isosurface) count++;
  if (d.value > isosurface) count++;
  // handle each case
  if (count == 4) { PaintTetraHelper4(a,b,c,d,isosurface); } 
  else if (count == 3) {
    if (d.value <= isosurface) PaintTetraHelper3(a,b,c,d,isosurface);
    else if (c.value <= isosurface) PaintTetraHelper3(a,d,b,c,isosurface);
    else if (b.value <= isosurface) PaintTetraHelper3(a,c,d,b,isosurface);
    else if (a.value <= isosurface) PaintTetraHelper3(b,d,c,a,isosurface);
    else assert(0);
  } else if (count == 2) {
    if (a.value > isosurface && b.value > isosurface) PaintTetraHelper2(a,b,c,d,isosurface);
    else if (a.value > isosurface && c.value > isosurface) PaintTetraHelper2(a,c,d,b,isosurface);
    else if (a.value > isosurface && d.value > isosurface) PaintTetraHelper2(a,d,b,c,isosurface);
    else if (b.value > isosurface && c.value > isosurface) PaintTetraHelper2(b,c,a,d,isosurface);
    else if (b.value > isosurface && d.value > isosurface) PaintTetraHelper2(b,d,c,a,isosurface);
    else if (c.value > isosurface && d.value > isosurface) PaintTetraHelper2(c,d,a,b,isosurface);
    else assert(0);
  } else if (count == 1) {
    if (a.value > isosurface) PaintTetraHelper1(a,b,c,d,isosurface);
    else if (b.value > isosurface) PaintTetraHelper1(b,c,a,d,isosurface);
    else if (c.value > isosurface) PaintTetraHelper1(c,a,b,d,isosurface);
    else if (d.value > isosurface) PaintTetraHelper1(d,c,b,a,isosurface);
    else assert(0);
  } else {
    // count == 0, do nothing
    assert (count == 0);
  }
}

// ============================================================================

// when all 4 vertices are within the volume
void MarchingCubes::PaintTetraHelper4(const GridValue &a, const GridValue &b, const GridValue &c, const GridValue &d, double isosurface)  {
  assert (a.value > isosurface && b.value > isosurface && c.value > isosurface && d.value > isosurface);
  drawIfBoundary(a.position,b.position,c.position);
  drawIfBoundary(a.position,d.position,b.position);
  drawIfBoundary(b.position,d.position,c.position);
  drawIfBoundary(c.position,d.position,a.position);
}

// when 3 vertices are within the volume
void MarchingCubes::PaintTetraHelper3(const GridValue &a, const GridValue &b, const GridValue &c, const GridValue &d, double isosurface)  {
  assert (a.value > isosurface && b.value > isosurface && c.value > isosurface && d.value <= isosurface);
  float ad_frac = (isosurface - d.value) / (a.value - d.value);
  assert (ad_frac >= 0 && ad_frac <= 1);
  float bd_frac = (isosurface - d.value) / (b.value - d.value);
  assert (bd_frac >= 0 && bd_frac <= 1);
  float cd_frac = (isosurface - d.value) / (c.value - d.value);
  assert (cd_frac >= 0 && cd_frac <= 1);
  Vec3f ad = ad_frac*a.position + (1-ad_frac)*d.position;
  Vec3f bd = bd_frac*b.position + (1-bd_frac)*d.position;
  Vec3f cd = cd_frac*c.position + (1-cd_frac)*d.position;
  Vec3f ad_norm = ad_frac*a.normal + (1-ad_frac)*d.normal;
  Vec3f bd_norm = bd_frac*b.normal + (1-bd_frac)*d.normal;
  Vec3f cd_norm = cd_frac*c.normal + (1-cd_frac)*d.normal;
  drawTriangleWithNormals(ad_norm,ad,cd_norm,cd,bd_norm,bd);
  drawIfBoundary(a.position,ad,bd);
  drawIfBoundary(a.position,bd,b.position);
  drawIfBoundary(b.position,bd,cd);
  drawIfBoundary(b.position,cd,c.position);
  drawIfBoundary(c.position,cd,ad);
  drawIfBoundary(c.position,ad,a.position);
  drawIfBoundary(a.position,b.position,c.position);
}

// when 2 vertices are within the volume
void MarchingCubes::PaintTetraHelper2(const GridValue &a, const GridValue &b, const GridValue &c, const GridValue &d, double isosurface)  {
  assert (a.value > isosurface && b.value > isosurface && c.value <= isosurface && d.value <= isosurface);
  float ac_frac = (isosurface - c.value) / (a.value - c.value);
  assert (ac_frac >= 0 && ac_frac <= 1);
  float ad_frac = (isosurface - d.value) / (a.value - d.value);
  assert (ad_frac >= 0 && ad_frac <= 1);
  float bc_frac = (isosurface - c.value) / (b.value - c.value);
  assert (bc_frac >= 0 && bc_frac <= 1);
  float bd_frac = (isosurface - d.value) / (b.value - d.value);
  assert (bd_frac >= 0 && bd_frac <= 1);
  Vec3f ac = ac_frac*a.position + (1-ac_frac)*c.position;
  Vec3f ad = ad_frac*a.position + (1-ad_frac)*d.position;
  Vec3f bc = bc_frac*b.position + (1-bc_frac)*c.position;
  Vec3f bd = bd_frac*b.position + (1-bd_frac)*d.position;
  Vec3f ac_norm = ac_frac*a.normal + (1-ac_frac)*c.normal;
  Vec3f ad_norm = ad_frac*a.normal + (1-ad_frac)*d.normal;
  Vec3f bc_norm = bc_frac*b.normal + (1-bc_frac)*c.normal;
  Vec3f bd_norm = bd_frac*b.normal + (1-bd_frac)*d.normal;
  drawTriangleWithNormals(ac_norm,ac,bc_norm,bc,ad_norm,ad);
  drawTriangleWithNormals(bc_norm,bc,bd_norm,bd,ad_norm,ad);
  drawIfBoundary(a.position,ac,ad);
  drawIfBoundary(b.position,bd,bc);
  drawIfBoundary(a.position,ad,bd);
  drawIfBoundary(a.position,bd,b.position);
  drawIfBoundary(a.position,bc,ac);
  drawIfBoundary(a.position,b.position,bc);
}

// when 1 vertex is within the volume
void MarchingCubes::PaintTetraHelper1(const GridValue &a, const GridValue &b, const GridValue &c, const GridValue &d, double isosurface)  {
  assert (a.value > isosurface && b.value <= isosurface && c.value <= isosurface && d.value <= isosurface);
  float ab_frac = (isosurface - b.value) / (a.value - b.value);
  assert (ab_frac >= 0 && ab_frac <= 1);
  float ac_frac = (isosurface - c.value) / (a.value - c.value);
  assert (ac_frac >= 0 && ac_frac <= 1);
  float ad_frac = (isosurface - d.value) / (a.value - d.value);
  assert (ad_frac >= 0 && ad_frac <= 1);
  Vec3f ab = ab_frac*a.position + (1-ab_frac)*b.position;
  Vec3f ac = ac_frac*a.position + (1-ac_frac)*c.position;
  Vec3f ad = ad_frac*a.position + (1-ad_frac)*d.position;
  Vec3f ab_norm = ab_frac*a.normal + (1-ab_frac)*b.normal;
  Vec3f ac_norm = ac_frac*a.normal + (1-ac_frac)*c.normal;
  Vec3f ad_norm = ad_frac*a.normal + (1-ad_frac)*d.normal;
  drawTriangleWithNormals(ab_norm,ab,ad_norm,ad,ac_norm,ac);
  drawIfBoundary(a.position,ab,ac);
  drawIfBoundary(a.position,ac,ad);
  drawIfBoundary(a.position,ad,ab);
}

// ============================================================================


void MarchingCubes::drawTriangleWithNormals(const Vec3f &n1, const Vec3f &p1, 
					    const Vec3f &n2, const Vec3f &p2, 
					    const Vec3f &n3, const Vec3f &p3) {
  MCTri tmp(this);
  tmp.a = p1;
  tmp.b = p2;
  tmp.c = p3;
  tmp.na = n1;
  tmp.nb = n2;
  tmp.nc = n3;
  mc_tris.push_back(tmp);
}

void MarchingCubes::drawIfBoundary(const Vec3f &p1, const Vec3f &p2, const Vec3f &p3) {
  if ((p1.x() <= 0.1*dx && p2.x() <= 0.1*dx && p3.x() <= 0.1*dx) ||
      (p1.y() <= 0.1*dy && p2.y() <= 0.1*dy && p3.y() <= 0.1*dy) ||
      (p1.z() <= 0.1*dz && p2.z() <= 0.1*dz && p3.z() <= 0.1*dz) ||
      (p1.x() >= dx*(nx-1.1) && p2.x() >= dx*(nx-1.1) && p3.x() >= dx*(nx-1.1)) ||
      (p1.y() >= dy*(ny-1.1) && p2.y() >= dy*(ny-1.1) && p3.y() >= dy*(ny-1.1)) ||
      (p1.z() >= dz*(nz-1.1) && p2.z() >= dz*(nz-1.1) && p3.z() >= dz*(nz-1.1))) {
    Vec3f normal = ComputeNormal(p1,p2,p3);
    MCTri tmp(this);
    tmp.a = p1;
    tmp.b = p2;
    tmp.c = p3;
    tmp.na = tmp.nb = tmp.nc = normal;
    mc_tris.push_back(tmp);
  }
}

bool MCTri::intersect(const Ray& r, Hit& h) const {
    
    Vec3f normal = ComputeNormal(a, b, c);

    Vec3f origin = r.getOrigin();
    Vec3f dir = r.getDirection();
    
    
    // test for intersection against the plane
    float t;
    {
        float d = normal.Dot3(a);
        float numer = d - origin.Dot3(normal);
        float denom = dir.Dot3(normal);

        if (denom == 0) return 0;

        // we do intersect the plane!
        t = numer/denom;

        if (t < EPSILON) {
            // the intersection is too close
            return false;
        }

        
        if (t > h.getT()) {
        
            // not an improvement on the previous hit 
            return false;
        }
    }
    
    // figure out the barycentric coordinates:
    //  we must have
    //    a * alpha + b * beta + c * gamma  =  dir * t + origin
    //        alpha   +   beta   +   gamma  =  1
    //
    // Solve this linear system:
    // [ ax-bx   ax-cx  dir.x ][ beta  ]     [ a.x - origin.x ] 
    // [ ay-by   ay-cy  dir.x ][ gamma ]  =  [ a.y - origin.y ] 
    // [ az-bz   az-cz  dir.z ][ t     ]     [ a.z - origin.z ] 
    // solve for beta, gamma using Cramer's rule

    
    float det = Matrix::det3x3(a.x() - b.x(), a.x() - c.x(), dir.x(),
                               a.y() - b.y(), a.y() - c.y(), dir.y(),
                               a.z() - b.z(), a.z() - c.z(), dir.z());

    if (fabs(det) <= 0.000001) {
     
      return 0;
    }
    assert (fabs(det) >= 0.000001);

    float beta = Matrix::det3x3(a.x() - origin.x(), a.x() - c.x(), dir.x(),
                                a.y() - origin.y(), a.y() - c.y(), dir.y(),
                                a.z() - origin.z(), a.z() - c.z(), dir.z()) / det;

    float gamma = Matrix::det3x3(a.x() - b.x(), a.x() - origin.x(), dir.x(),
                                 a.y() - b.y(), a.y() - origin.y(), dir.y(),
                                 a.z() - b.z(), a.z() - origin.z(), dir.z()) / det;

    float alpha = 1 - beta - gamma;

    // if any of the barycentric coordinates are outside the range [0,1] then we missed the triangle
    if (beta < -0.00001 || 1.00001 < beta) {
     
        return false;
    }
    if (gamma < -0.00001 || 1.00001 < gamma) {
     
        return false;
    }
    if (alpha < -0.00001 || 1.00001 < alpha) {
  
        return false;
    }

    // interpolate between the normals on the corners
    Vec3f faked_normal = na * alpha + nb * beta + nc * gamma;
    faked_normal.Normalize();
    Material* mat = this->marchingCubes->getMaterial();

    h.set(t, mat, faked_normal, (Shape*)nullptr);
    return true;
}

// ============================================================================

