#include "utils.h"
#include "material.h"
#include "argparser.h"
#include "sphere.h"
#include "vertex.h"
#include "mesh.h"
#include "boundingbox.h"
#include "raytrace/ray.h"
#include "raytrace/hit.h"

// ====================================================================
// ====================================================================

bool Sphere::intersect(const Ray &ray, Hit &hit) const {

    Vec3f dir = ray.getDirection();
    Vec3f offset = ray.getOrigin() - center;
    
    // plug the explicit ray equation into the implict sphere equation and solve
    // p = r.getOrigin() + t * r.getDirection()
    // and we must have |p - center| < radius
    // thus |origin + t * dir - center|^2 = radius^2
    //      <offset + t*dir, offset + t*dir> = radius^2
    //      <offset,offset> + 2 * t * <offset, dir> + t*t*<dir,dir> = radius^2
    //      (<offset,offset> - radius^2) + t * (2<offset, dir>) + t^2 * <dir,dir> = 0
    //       ^^^^^^^^^^^^^^^^^^^^^^^^^^         ^^^^^^^^^^^^^^          ^^^^^^^^^
    //                   c              +          b*t           +         at^2 = 0
    //
    float c = offset.Dot3(offset) - radius*radius;
    float b = 2 * offset.Dot3(dir);
    float a = dir.Dot3(dir); // should always be 1 !!
    
    // at this point, we are solving the quadratic
    //  0 = at^2 + bt + c


    // solve the quadratic at^2 + bt + c = 0
    float discr = b*b - 4.0 * a * c;

    // no solutions
    if (discr < 0) {
        return false; 
    }

    float t_plus  = (-b + std::sqrt(discr)) / (2.0 * a);
    float t_minus = (-b - std::sqrt(discr)) / (2.0 * a);

    // two candidate hits.
    // we take the minimum, as long as its greater than zero
      
    float t = INFINITY;

    if (t_plus > EPSILON) {
        t = std::min(t, t_plus);
    }
    if (t_minus > EPSILON) {
        t = std::min(t, t_minus);
    }

    if (std::isinf(t)) {
        return false; // never intersects
    }

    auto pos = ray.pointAtParameter(t);
    
    // normals on the sphere point out from the center
    auto normal = (pos - center).Normalized();

    hit.set(t, getMaterial(), normal, (Shape*)this);
    return true;

} 

BoundingBox Sphere::getBoundingBox() const {
    Vec3f extents(radius, radius, radius);
    return BoundingBox(center - extents, center + extents);
}

// ====================================================================
// ====================================================================

// helper function to place a grid of points on the sphere
Vec3f ComputeSpherePoint(float s, float t, const Vec3f center, float radius) {
    float angle = 2*M_PI*s;
    float y = -cos(M_PI*t);
    float factor = sqrt(1-y*y);
    float x = factor*cos(angle);
    float z = factor*-sin(angle);
    Vec3f answer = Vec3f(x,y,z);
    answer *= radius;
    answer += center;
    return answer;
}

void Sphere::addRasterizedFaces(Mesh *m, ArgParser * /*args*/) {

    // and convert it into quad patches 
    int h = Sphere::horizontal_rasterization;
    int v = Sphere::vertical_rasterization;

    assert (h % 2 == 0);
    int i,j;
    int va,vb,vc,vd;
    Vertex *a,*b,*c,*d;
    int offset = m->numVertices(); //vertices.size();

    // place vertices
    m->addVertex(center+radius*Vec3f(0,-1,0));  // bottom
    for (j = 1; j < v; j++) {  // middle
        for (i = 0; i < h; i++) {
            float s = i / float(h);
            float t = j / float(v);
            m->addVertex(ComputeSpherePoint(s,t,center,radius));
        }
    }
    m->addVertex(center+radius*Vec3f(0,1,0));  // top

    // the middle patches
    for (j = 1; j < v-1; j++) {
        for (i = 0; i < h; i++) {
            va = 1 +  i      + h*(j-1);
            vb = 1 + (i+1)%h + h*(j-1);
            vc = 1 +  i      + h*(j);
            vd = 1 + (i+1)%h + h*(j);
            a = m->getVertex(offset + va);
            b = m->getVertex(offset + vb);
            c = m->getVertex(offset + vc);
            d = m->getVertex(offset + vd);
            m->addRasterizedPrimitiveFace(a,b,d,c,material);
        }
    }

    for (i = 0; i < h; i+=2) {
        // the bottom patches
        va = 0;
        vb = 1 +  i;
        vc = 1 + (i+1)%h;
        vd = 1 + (i+2)%h;
        a = m->getVertex(offset + va);
        b = m->getVertex(offset + vb);
        c = m->getVertex(offset + vc);
        d = m->getVertex(offset + vd);
        m->addRasterizedPrimitiveFace(d,c,b,a,material);
        // the top patches
        va = 1 + h*(v-1);
        vb = 1 +  i      + h*(v-2);
        vc = 1 + (i+1)%h + h*(v-2);
        vd = 1 + (i+2)%h + h*(v-2);
        a = m->getVertex(offset + va);
        b = m->getVertex(offset + vb);
        c = m->getVertex(offset + vc);
        d = m->getVertex(offset + vd);
        m->addRasterizedPrimitiveFace(b,c,d,a,material);
    }
}
