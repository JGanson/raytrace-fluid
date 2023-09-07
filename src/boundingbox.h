#ifndef _BOUNDING_BOX_H_
#define _BOUNDING_BOX_H_

#include <cassert>
#include <algorithm>

#include "vectors.h"
#include "primitive.h"
#include "utils.h"

// Because getting std::max & std::min to work on all platforms is annoying
inline double mymax(double x, double y) { if (x > y) return x; return y; }
inline double mymin(double x, double y) { if (x < y) return x; return y; }

// ====================================================================
// ====================================================================

// An axis aligned bounding box
class BoundingBox {

public:

    // ========================
    // CONSTRUCTOR & DESTRUCTOR
    BoundingBox() { 
        minimum = Vec3f(FLOAT_INFINITY, FLOAT_INFINITY, FLOAT_INFINITY);
        maximum = -minimum;
    }
    BoundingBox(const Vec3f &pt) {
        Set(pt,pt);
    }
    BoundingBox(const Vec3f &_minimum, const Vec3f &_maximum) { 
        Set(_minimum,_maximum);
    }
    ~BoundingBox() { }

    // =========
    // ACCESSORS
    void Get(Vec3f &_minimum, Vec3f &_maximum) const {
        _minimum = minimum;
        _maximum = maximum;
    }
    const Vec3f& getMin() const { return minimum; }
    const Vec3f& getMax() const { return maximum; }

    void computeCenter(Vec3f &c) const {
        c = maximum; 
        c -= minimum;
        c *= 0.5f;
        c += minimum;
    }

    Vec3f getCenter() const {
        return 0.5f * minimum + 0.5f * maximum;
    }

    Vec3f Size() const {
        return maximum - minimum;
    }

    double SurfaceArea() const {
        auto size = Size();
        return 2.0 * (size.y() * size.z() + size.x() * size.z() + size.x() * size.y());
    }
    


    double maxDim() const {
        double x = maximum.x() - minimum.x();
        double y = maximum.y() - minimum.y();
        double z = maximum.z() - minimum.z();
        return mymax(x,mymax(y,z));
    }

    // Is the given point contained within the box?
    bool contains(const Vec3f& p) const {
        return (minimum.x() <= p.x() && p.x() <= maximum.x()
             && minimum.y() <= p.y() && p.y() <= maximum.y()
             && minimum.z() <= p.z() && p.z() <= maximum.z());
    }
    // Is the given point contained within the box, to some tolerance eps?
    bool contains(const Vec3f& p, float eps) const {
        return (minimum.x() - eps <= p.x() && p.x() < maximum.x() + eps
            &&  minimum.y() - eps <= p.y() && p.y() < maximum.y() + eps
            &&  minimum.z() - eps <= p.z() && p.z() < maximum.z() + eps);
    }


    // Is this bounding box a subset of another?
    bool isSubset(const BoundingBox& other) const {
        return other.contains(minimum) && other.contains(maximum);
    }

    // =========
    // MODIFIERS
    void Set(const BoundingBox &bb) {
        minimum = bb.minimum;
        maximum = bb.maximum;
    }
    void Set(const Vec3f &_minimum, const Vec3f &_maximum) {
        assert (minimum.x() <= maximum.x() &&
                minimum.y() <= maximum.y() &&
                minimum.z() <= maximum.z());
        minimum = _minimum;
        maximum = _maximum; }
    void Extend(const Vec3f v) {
        minimum = Vec3f(mymin(minimum.x(),v.x()),
                        mymin(minimum.y(),v.y()),
                        mymin(minimum.z(),v.z()));
        maximum = Vec3f(mymax(maximum.x(),v.x()),
                        mymax(maximum.y(),v.y()),
                        mymax(maximum.z(),v.z())); 
    }
    void Extend(const BoundingBox &bb) {
        Extend(bb.minimum);
        Extend(bb.maximum); 
    }


    // ==========
    // primitive operations
    bool intersect(const Ray &r, Hit &h) const;


    static int triCount() { return 12 * 36; }

    void packMesh(float* &current) const;

private:

    // ==============
    // REPRESENTATION
    Vec3f minimum;
    Vec3f maximum;
};

// ====================================================================
// ====================================================================

#endif
