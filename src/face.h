#ifndef _FACE_H_
#define _FACE_H_

#include "edge.h"
#include "vertex.h"
#include "primitive.h"
#include "boundingbox.h"
#include "raytrace/ray.h"
#include "raytrace/hit.h"

class Material;

// ==============================================================
// Simple class to store quads for use in raytracing.

class Face : Primitive {

public:

  // ========================
  // CONSTRUCTOR & DESTRUCTOR
  Face(Material *m) {
    edge = NULL;
    material = m; }

  // =========
  // ACCESSORS
  Vertex* operator[](int i) const { 
    assert (edge != NULL);
    if (i==0) return edge->getStartVertex();
    if (i==1) return edge->getNext()->getStartVertex();
    if (i==2) return edge->getNext()->getNext()->getStartVertex();
    if (i==3) return edge->getNext()->getNext()->getNext()->getStartVertex();
    assert(0);
    exit(0);
  }
  Edge* getEdge() const { 
    assert (edge != NULL);
    return edge; 
  }
  Vec3f computeCentroid() const {
    return 0.25f * ((*this)[0]->get() +
                    (*this)[1]->get() +
                    (*this)[2]->get() +
                    (*this)[3]->get());
  }
  Material* getMaterial() const { return material; }
  float getArea() const;
  Vec3f RandomPoint() const;
  Vec3f computeNormal() const;

  // =========
  // MODIFIERS
  void setEdge(Edge *e) {
    assert (edge == NULL);
    assert (e != NULL);
    edge = e;
  }

  // ==========
  // RAYTRACING
  bool intersect(const Ray &r, Hit &h, bool intersect_backfacing) const;

  virtual bool intersect(const Ray& r, Hit &h) const { return intersect(r, h, true); }

  virtual BoundingBox getBoundingBox() const {
    BoundingBox bbox;
    bbox.Extend((*this)[0]->get());
    bbox.Extend((*this)[1]->get());
    bbox.Extend((*this)[2]->get());
    bbox.Extend((*this)[3]->get());
    return bbox;
  }

  virtual void addRasterizedFaces(Mesh*, ArgParser*) { return; }

protected:

  // helper functions
  bool triangle_intersect(const Ray &r, Hit &h, Vertex *a, Vertex *b, Vertex *c, bool intersect_backfacing) const;
  bool plane_intersect(const Ray &r, Hit &h, bool intersect_backfacing) const;

  // don't use this constructor
  Face& operator= (const Face&) { assert(0); exit(0); }
  
  // ==============
  // REPRESENTATION
  Edge *edge;
  // NOTE: If you want to modify a face, remove it from the mesh,
  // delete it, create a new copy with the changes, and re-add it.
  // This will ensure the edges get updated appropriately.
  
  Material *material;
};

// ===========================================================

#endif
