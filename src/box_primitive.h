#ifndef __BOX_PRIMITIVE_H__
#define __BOX_PRIMITIVE_H__

#include "boundingbox.h"
#include "primitive.h"
#include "argparser.h"

// An axis aligned bounding box with a material applied to it.
class BoxPrimitive : public Primitive {
public:
    BoxPrimitive(Vec3f min, Vec3f max, Material* mat) : bbox(min, max), material(mat) { }

    virtual BoundingBox getBoundingBox() const { return bbox; }

    virtual bool intersect(const Ray& ray, Hit& hit) const;

    virtual void addRasterizedFaces(Mesh*, ArgParser*);

private:
    // Representation
    BoundingBox bbox;
    Material* material;
};

#endif
