#include "box_primitive.h"
#include "mesh.h"
#include "raytrace/hit.h"
#include "argparser.h"

bool BoxPrimitive::intersect(const Ray& ray, Hit& hit) const {
    bool found_hit = bbox.intersect(ray, hit);
    if (found_hit) {
        hit.setMaterial(material);
    }
    return found_hit;
}

void BoxPrimitive::addRasterizedFaces(Mesh* m, ArgParser*) {
    Vec3f center = bbox.getCenter();
    Vec3f extents = 0.5 * bbox.getMax() - 0.5 * bbox.getMin();

    // the actual points
    Vec3f a = center + Vec3f( 1, 1, 1) * extents;
    Vec3f b = center + Vec3f( 1, 1,-1) * extents;
    Vec3f c = center + Vec3f( 1,-1, 1) * extents;
    Vec3f d = center + Vec3f( 1,-1,-1) * extents;
    Vec3f e = center + Vec3f(-1, 1, 1) * extents;
    Vec3f f = center + Vec3f(-1, 1,-1) * extents;
    Vec3f g = center + Vec3f(-1,-1, 1) * extents;
    Vec3f h = center + Vec3f(-1,-1,-1) * extents;

    // each face will create overlapping vertices, for convience
    // (the mesh class does not let us add faces after the fact)
    auto make_face = [m](Vec3f p1, Vec3f p2, Vec3f p3, Vec3f p4, Material* material) {
        Vertex* v1 = m->addVertex(p1);
        Vertex* v2 = m->addVertex(p2);
        Vertex* v3 = m->addVertex(p3);
        Vertex* v4 = m->addVertex(p4);
        m->addRasterizedPrimitiveFace(v1, v2, v3, v4, material);
    };

    make_face(b,a,c,d, material); // x=1
    make_face(a,b,f,e, material); // y=1
    make_face(c,a,e,g, material); // z=1
    make_face(e,f,h,g, material); // x=-1
    make_face(d,c,g,h, material); // y=-1
    make_face(b,d,h,f, material); // z=-1
}
