#ifndef _SPHERE_H_
#define _SPHERE_H_

#include "primitive.h"
#include "vectors.h"

// ====================================================================
// ====================================================================
// Simple implicit repesentation of a cuboid, 
// rasterized for use in radiosity.

class Cuboid : public Primitive {

public:
    // CONSTRUCTOR & DESTRUCTOR
    Cuboid(const Vec3f &min, Vec3f& max, Material *m) {
        min_corner = min;
        max_corner = max;
    }

    // for ray tracing
    virtual bool intersect(const Ray &r, Hit &h) const;

    // for OpenGL rendering & radiosity
    void addRasterizedFaces(Mesh *m, ArgParser *args);

private:

    // REPRESENTATION
    Vec3f min_corner;
    Vec3f max_corner;
};

// ====================================================================
// ====================================================================

#endif
