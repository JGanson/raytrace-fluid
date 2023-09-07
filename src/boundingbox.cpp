#include "boundingbox.h"

#include <limits>

#include "utils.h"
#include "raytrace/ray.h"
#include "raytrace/hit.h"

bool BoundingBox::intersect(const Ray &r, Hit &h) const {
    const Vec3f& dir = r.getDirection();
    const Vec3f& origin = r.getOrigin();

    if (contains(origin)) {
        h.set(0.0, nullptr, Vec3f(0,0,0), (Shape*)nullptr);
        return true;
    }

    // intersect with each of six faces

    // solve the equation
    // origin.x + dir.x * t = face.x
    //  =>   t = (face.x - origin.x) / dir.x
    //  If dir.x is zero, then set t to infinity
    float t_xmin = origin.x()? (minimum.x() - origin.x()) / dir.x() : FLOAT_INFINITY;
    float t_xmax = origin.x()? (maximum.x() - origin.x()) / dir.x() : FLOAT_INFINITY;
    float t_ymin = origin.y()? (minimum.y() - origin.y()) / dir.y() : FLOAT_INFINITY;
    float t_ymax = origin.y()? (maximum.y() - origin.y()) / dir.y() : FLOAT_INFINITY;
    float t_zmin = origin.z()? (minimum.z() - origin.z()) / dir.z() : FLOAT_INFINITY;
    float t_zmax = origin.z()? (maximum.z() - origin.z()) / dir.z() : FLOAT_INFINITY;

    float epsilon = 0.1;

    bool is_hit = false;
    // to be valid, the point at each t value must be in the boundary of the box.
    if (t_xmin > EPSILON && t_xmin < h.getT() && contains(r.pointAt(t_xmin), epsilon)) {
        h.set(t_xmin, nullptr, Vec3f(-1,  0,  0), (Shape*)nullptr);
        is_hit = true;
    }
    if (t_xmax > EPSILON && t_xmax < h.getT() && contains(r.pointAt(t_xmax), epsilon)) {
        h.set(t_xmax, nullptr, Vec3f( 1,  0,  0), (Shape*)nullptr);
        is_hit = true;
    }
    if (t_ymin > EPSILON && t_ymin < h.getT() && contains(r.pointAt(t_ymin), epsilon)) {
        h.set(t_ymin, nullptr, Vec3f( 0, -1,  0), (Shape*)nullptr);
        is_hit = true;
    }
    if (t_ymax > EPSILON && t_ymax < h.getT() && contains(r.pointAt(t_ymax), epsilon)) {
        h.set(t_ymax, nullptr, Vec3f( 0,  1,  0), (Shape*)nullptr);
        is_hit = true;
    }
    if (t_zmin > EPSILON && t_zmin < h.getT() && contains(r.pointAt(t_zmin), epsilon)) {
        h.set(t_zmin, nullptr, Vec3f( 0,  0, -1), (Shape*)nullptr);
        is_hit = true;
    }
    if (t_zmax > EPSILON && t_zmax < h.getT() && contains(r.pointAt(t_zmax), epsilon)) {
        h.set(t_zmax, nullptr, Vec3f( 0,  0,  1), (Shape*)nullptr);
        is_hit = true;
    }
    return is_hit;
}

// Uses 12 edges (each with 12 triangles)
void BoundingBox::packMesh(float* &current) const {
  Vec3f min = minimum;
  Vec3f max = maximum;
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
