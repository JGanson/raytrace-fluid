#include <algorithm>
#include <chrono>

#include "bvh.h"
#include "hit.h"
#include "../boundingbox.h"
#include "../primitive.h"

BVH::BVH(const BoundingBox& _bbox, int _depth) 
  : bbox(new BoundingBox(_bbox)),
    depth(_depth),
    child1(nullptr),
    child2(nullptr),
    primitives()
{
}

BVH::~BVH() {
    delete bbox;
    delete child1;
    delete child2;
    // we do not own the primitives !
}

BVH_Heuristic bvh_parse_heuristic(const std::string& name) {
    if (name == "") {
        return BVH_LongestAxis_Median; // default
    }
    if (name == "naive") {
        return BVH_NaiveFlat;
    }
    if (name == "longest") {
        return BVH_LongestAxis_Median;
    }
    if (name == "alternate") {
        return BVH_AlternateAxis_Median;
    }
    if (name == "surface_area") {
        return BVH_BestAxis_SurfaceArea;
    }
    std::cerr << "Invalid BVH heuristic: '" << name << "', expected one of:\n";
    std::cerr << "    'naive', 'longest', 'largest', 'surface_area'\n";
    assert(0);
}

std::string bvh_heuristic_to_string(BVH_Heuristic heuristic) {
    switch (heuristic) {
    case BVH_NaiveFlat: return "naive";
    case BVH_LongestAxis_Median: return "longest";
    case BVH_AlternateAxis_Median: return "alternate";
    case BVH_BestAxis_SurfaceArea: return "surface_area";
    default: return "<unknown>";
    }
}

// =====================================================================================
//  Build Helpers

void sort_by_axis(std::vector<Primitive*>& primitives, uint start, uint end, int axis) {
    assert (0 <= start);
    assert (end <= primitives.size());
    assert (start < end);
    std::sort(primitives.begin() + start, primitives.begin() + end, [axis](Primitive* a, Primitive* b) {
        float a_coord = a->getBoundingBox().getCenter()[axis];
        float b_coord = b->getBoundingBox().getCenter()[axis];
        return a_coord < b_coord;
    });
}

BoundingBox make_bbox(std::vector<Primitive*>& primitives, uint start, uint end) {
    BoundingBox bbox;
    for (uint i = start; i < end; i++) {
        bbox.Extend(primitives[i]->getBoundingBox());
    }
    return bbox;
}

// =====================================================================================
//  Heuristic Implementations
// recall that heurstics return the split index to use (after modify the subarray)
// or -1 if a leaf should be used.

int Build_Naive(std::vector<Primitive*>& /*primitives*/, uint /*start*/, uint /*end*/, const BoundingBox& /*bbox*/, int /*depth*/) {
    return -1; // always make it a leaf
}
int Build_Alternate(std::vector<Primitive*>& primitives, uint start, uint end, const BoundingBox& /*bbox*/, int depth) {
    // cycle through available axes
    int split_axis = depth % 3;
  
    // sort by desired axis
    sort_by_axis(primitives, start, end, split_axis);

    // half of the primitives go to one child, half to the other
    uint mid = (start + end) / 2;

    return mid;
}
int Build_Longest(std::vector<Primitive*>& primitives, uint start, uint end, const BoundingBox& bbox, int /*depth*/) {
    if (end - start <= 2) {
        return -1; // make it a leaf
    }
    int split_axis;
    Vec3f size = bbox.getMax() - bbox.getMin();
    if (size.x() >= size.y() && size.x() >= size.z()) {
        split_axis = 0;
    } else if (size.y() >= size.x() && size.y() >= size.z()) {
        split_axis = 1;
    } else {
        split_axis = 2;
    }

    // sort by desired axis
    sort_by_axis(primitives, start, end, split_axis);

    // half of the primitives go to one child, half to the other
    uint mid = (start + end) / 2;

    return mid;
}

int Build_SurfaceArea(std::vector<Primitive*>& primitives, uint start, uint end, const BoundingBox& bbox, int /*depth*/) {
    if (end - start <= 2) {
        return -1; // make it a leaf
    }


    float best_score = FLOAT_INFINITY;
    int best_axis = -1;
    int best_split = -1;

    for (int split_axis = 0; split_axis < 3; split_axis++) {
        // test each split axis
        sort_by_axis(primitives, start, end, split_axis);

        // at each index to be evaluated, we split the array into a left and right:
        //  primitives[start..mid] | primitives[mid..end]
        //  +------- left -------+   +------ right -----+
        //  And we wish to know 'left' and 'right' bounding boxes at each step.

        // iterate in the reverse direction to calculate each 'right' bbox,
        // and store them in an array
        std::vector<BoundingBox> right_bboxes(end - start);
        assert (right_bboxes.size() == end-start);
        {
            BoundingBox right;
            assert (end != 0);
            assert (start < end);
            for (int mid = end - 1; int(start) <= mid; mid--) {
                right.Extend(primitives[mid]->getBoundingBox());
                right_bboxes[mid - start] = right;
            }
        }

        float surfaceArea = bbox.SurfaceArea();

        // the left bounding box will be built up as we go
        BoundingBox left;
        left.Extend(primitives[start]->getBoundingBox());

        for (uint mid = start + 1; mid < end; mid++) {

            // update the left side (as we are moving forward in the array)
            left.Extend(primitives[mid]->getBoundingBox());

            // fetch the corresponding right side
            const BoundingBox& right = right_bboxes[mid - start];

            // this is a score that incentives a large number of objects per unit surface area.
            float score = (left.SurfaceArea() * (mid - start) + right.SurfaceArea() * (end - mid)) / surfaceArea;

            if (score < best_score) {
                best_score = score;
                best_axis = split_axis;
                best_split = mid;
            }


        }



    }

    if (best_axis == -1) {
        return -1;
    }

    // sort by the desired axis
    sort_by_axis(primitives, start, end, best_axis);

    return best_split;
}


BVH* BuildHelper(BVHBuilder b, std::vector<Primitive*>& primitives, uint start, uint end, int depth) {
    BoundingBox bbox = make_bbox(primitives, start, end);
    BVH* node = new BVH(bbox, depth);


    // get the split position from the heuristic
    int mid = -1;
    if (end - start > 2) {
        // only use the heuristic on non-trivial bins
        mid = (*b)(primitives, start, end, bbox, depth);
    }

    if (mid == -1) {
        // make a leaf node
        for (uint i = start; i < end; i++) {
            node->primitives.push_back(primitives[i]);
        }
        return node;
    }

    node->child1 = BuildHelper(b, primitives, start, mid, depth + 1);
    node->child2 = BuildHelper(b, primitives, mid, end, depth + 1);

    return node;
}

bool BVH::CastRay(const Ray& ray, Hit& hit) const {
    Hit hit1;

    if (!bbox->intersect(ray, hit1)) {
        // no intersection, we can prune this node
        return false; 
    }
    
    if (isLeaf()) {
        bool ans = false;
        // simply check each primitive in our possession
        // std::cout << "Leaf node: intersecting " << primitives.size() << " primitives\n";
        //std::cout<<"1"<<std::endl;
        for (Primitive* p : primitives) {

            if (p->intersect(ray, hit)) {
                ans = true;
            }
        }
        
        // std::cout << "result: " << (ans? "found hit" : "nothing") << "\n";
        return ans; 
    }
    
    assert (child1 != nullptr);
    assert (child2 != nullptr);

    bool ans = false;

    if (child1->CastRay(ray, hit)) {
        ans = true;
    }
    if (child2->CastRay(ray, hit)) {
        ans = true;
    }
    
    return ans;
}

BVH* BVH::Build(ArgParser* args, std::vector<Primitive*>& primitives) {
    uint actual_count = primitives.size();

    BVHBuilder b;
    switch (args->mesh_data->bvh_heuristic) {
        case BVH_NaiveFlat: {
            b = &Build_Naive;
            break;
        }
        case BVH_LongestAxis_Median: {
            b = &Build_Longest;
            break;
        }
        case BVH_AlternateAxis_Median: {
            b = &Build_Alternate;
            break;
        }
        case BVH_BestAxis_SurfaceArea: {
            b = &Build_SurfaceArea;
            break;
        }
    }
    std::cout << "Building BVH with heuristic: '" << bvh_heuristic_to_string(args->mesh_data->bvh_heuristic) << "'\n";

    auto start_time = std::chrono::system_clock::now();

    BVH* bvh = BuildHelper(b, primitives, 0, primitives.size(), 0);

    auto elapsed = std::chrono::system_clock::now() - start_time;
    auto minutes = std::chrono::duration_cast<std::chrono::minutes>(elapsed);
    std::chrono::duration<double> seconds = elapsed - minutes;

    std::cout << "Built in " << minutes.count() << "m " << seconds.count() << "s.\n";

    std::cout << "Checking Repr...\n";
    assert(primitives.size() == actual_count);
    uint count = bvh->primitiveCount();
    assert(count == actual_count);

    int errs = bvh->checkRepr();
    assert(errs == 0);


    std::cout << "BVH Tree built with height: " << bvh->computeHeight() << "\n";

    return bvh;
}

uint BVH::primitiveCount() const {
    if (isLeaf()) {
        return primitives.size();
    }
    assert (child1);
    assert (child2);
    return child1->primitiveCount() + child2->primitiveCount();
}

uint BVH::computeHeight()  const {
    if (isLeaf()) {
        return 0;
    }
    assert (child1);
    assert (child2);

    uint h1 = child1->computeHeight();
    uint h2 = child2->computeHeight();

    return 1 + std::max(h1, h2);
} 

int BVH::triCount() const {
    int count = BoundingBox::triCount();
    if (child1) {
        count += child1->triCount();
    }
    if (child2) {
        count += child2->triCount();
    }
    return count;
}

void BVH::packMesh(float*& current) const {
    bbox->packMesh(current);
    if (child1) {
        child1->packMesh(current);
    }
    if (child2) {
        child2->packMesh(current);
    }
}

int BVH::checkRepr() const {
    int count = 0;
    if (isLeaf()) {
        for (Primitive* p : primitives) {
            if (!p->getBoundingBox().isSubset(*bbox)) {
                std::cerr << "WARNING: primitive is not contained within leaf bbox\n";
                count += 1;
            }
        }
        return count;
    }
    if (!primitives.empty()) {
        std::cerr << "WARNING: inner node contains " << primitives.size() << "primitives\n";
        count += 1;
    }
    if (!child1->bbox->isSubset(*bbox)) {
        std::cerr << "WARNING: child bounding box is not a subset of parent\n";
        count += 1;
    }
    if (!child2->bbox->isSubset(*bbox)) {
        std::cerr << "WARNING: cihld bounding box is not a subset of parent\n";
        count += 1;
    }
    count += child1->checkRepr();
    count += child2->checkRepr();
    return count;
}
