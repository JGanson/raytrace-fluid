#ifndef __BVH_H__
#define __BVH_H__

#include <cassert>
#include <string>
#include <vector>

#include "../primitive.h"

class ArgParser;
class BoundingBox;

enum BVH_Heuristic {
    BVH_NaiveFlat, // the default: this reduces the BVH to a linear search
    BVH_LongestAxis_Median,
    BVH_AlternateAxis_Median,
    BVH_BestAxis_SurfaceArea,
};

#define BVH_DEFAULT_HEURISTIC BVH_LongestAxis_Median
#define BVH_NUM_HEURISTICS 4

BVH_Heuristic bvh_parse_heuristic(const std::string& heuristic_name);

std::string bvh_heuristic_to_string(BVH_Heuristic heuristic);

/// A Build heuristic returns the index to split the array primitives[start..end] at, or -1 if it should be made a leaf.
typedef int (*BVHBuilder)(std::vector<Primitive*>& primitives, uint start, uint end, const BoundingBox& bbox, int depth);

/// Bounding Volume Hierarchy.
/// This is like a KD tree, but the data we store in it
/// are primitives that have a volume, not single points.
/// And theoretically we split on objects rather than space, but eh.
class BVH {
public:
    
    // CONSTRUCTOR & DESTRUCTOR
    static BVH* Build(ArgParser* args, std::vector<Primitive*>& primitives);
    friend BVH* BuildHelper(BVHBuilder b, std::vector<Primitive*>& primitives, uint start, uint end, int height);

    ~BVH();

    // ACCESSORS
    const BoundingBox* getBoundingBox() const { return bbox; }


    bool isLeaf() const {
        if (child1 == NULL && child2 == NULL) return true;
        assert (child1 != NULL && child2);
        return false;
    }

    const BVH* getChild1() const { return child1; }
    const BVH* getChild2() const { return child2; }
    BVH* getChild1() { return child1; }
    BVH* getChild2() { return child2; }

    const std::vector<Primitive*>& getPrimitives() const {
        return primitives;
    }

    // count the number of primitives we store
    uint primitiveCount() const;

    // count the height of the tree
    uint computeHeight() const;

    // Computes the closest intersection.
    bool CastRay(const Ray& ray, Hit& hit) const;

    // for OpenGL rendering
    int triCount() const;


    void packMesh(float*& current) const;


    // Do some invariant checking.
    // Return the number of violations.
    int checkRepr() const;


private:

    BVH(const BoundingBox &_bbox, int _depth=0);
    
    // We own our bounding box
    BoundingBox *bbox;
    int depth;

    // We own our children
    BVH* child1;
    BVH* child2;

    // We don't own the primitives, and in general they might not be valid
    // After the scene geometry changes
    std::vector<Primitive*> primitives;
};

#endif
