#include <iostream>
#include <algorithm>
#include <cstring>

#include "../argparser.h"
#include "../mesh.h"
#include "../face.h"
#include "../primitive.h"
#include "../utils.h"
#include "photon_mapping.h"
#include "kdtree.h"
#include "raytracer.h"
#include "bvh.h"


// ==========
// Clear/reset
void PhotonMapping::Clear() {
    // cleanup all the photons
    delete kdtree;
    kdtree = NULL;
}


// ========================================================================
// Recursively trace a single photon

void PhotonMapping::TracePhoton(const Vec3f &position, const Vec3f &dir,
                                const Vec3f &energy, int iter, Material* origin_mat) {



    // =====================================================
    // ASSIGNMENT: IMPLEMENT RECURSIVE PHOTON TRACING (done)
    // =====================================================

    // Trace the photon through the scene.  At each diffuse or
    // reflective bounce, store the photon in the kd tree.

    // One optimization is to *not* store the first bounce, since that
    // direct light can be efficiently computed using classic ray
    // tracing.
    

    if (iter > args->mesh_data->num_bounces) {
        return;
    }

    // std::cout << "Tracing Photon with energy " << energy << "\n";

    Ray ray = Ray(position, dir);
    Hit hit;
    
    bool found_hit = raytracer->getBVH()->CastRay(ray, hit);
    
    if (!found_hit) {
        return; // no photon to store !
    }
    
    Material* hit_mat = hit.getMaterial();
    Vec3f hit_point = ray.pointAtParameter(hit.getT());
    Vec3f normal = hit.getNormal();

    // compute the relative probabilities of diffuse / specular relfection
    // (proportionaly to their diffuse / reflective colors)
    float chance_diffuse = hit_mat->getDiffuseColor().SumOfComponents();
    float chance_specular = hit_mat->getReflectiveColor().SumOfComponents();
    float chance_transmit = hit_mat->getRefractiveColor().SumOfComponents();
    {
        float total = chance_diffuse + chance_specular + chance_transmit;
        chance_diffuse /= total;
        chance_specular /= total;
        chance_transmit /= total;
    }

    
    bool bounce_next = true;
    Vec3f next_dir, next_color;

    float r = GLOBAL_args->rand();
    
    if (r < chance_diffuse) {
        // we have decided to bounce diffusely
        next_dir = RandomDiffuseDirection(normal);
        next_color = hit_mat->getDiffuseColor();
    } else if (r < chance_diffuse + chance_specular) {
        // we have decided to bounce specularly
        next_dir = MirrorDirection(normal, dir);
        next_color = hit_mat->getReflectiveColor();
    } else if (r < chance_diffuse + chance_specular + chance_transmit) {
        // we are being transmitted, yay!
        next_dir = RefractedDirection(normal, dir, origin_mat, hit_mat);
        next_color = hit_mat->getRefractiveColor();
    } else {
        // it is absorbed
        bounce_next = false;
    }

    

    // do not store the initial location and the second bounce
    if (iter >= 2) {
        Vec3f absorbed_energy = energy;
        Photon photon = Photon(hit_point, dir, absorbed_energy, iter);
        
        kdtree->AddPhoton(photon);
    }

    if (bounce_next) {
        // perform the next trace
        Vec3f next_energy = next_color * energy;
        // scale up next energy to have the same norm as the previous energy
        float scale = energy.SumOfComponents() / next_energy.SumOfComponents();
        next_energy *= scale;
        TracePhoton(hit_point, next_dir, next_energy, iter + 1);
    }
    
}


// ========================================================================
// Trace the specified number of photons through the scene

void PhotonMapping::TracePhotons() {
    std::cout << "trace photons! \n";

    // first, throw away any existing photons
    delete kdtree;

    // consruct a kdtree to store the photons
    BoundingBox *bb = mesh->getBoundingBox();
    Vec3f min = bb->getMin();
    Vec3f max = bb->getMax();
    Vec3f diff = max-min;
    min -= 0.001f*diff;
    max += 0.001f*diff;
    kdtree = new KDTree(BoundingBox(min,max));

    // photons emanate from the light sources
    const std::vector<Face*>& lights = mesh->getLights();

    // compute the total area of the lights
    float total_lights_area = 0;
    for (unsigned int i = 0; i < lights.size(); i++) {
        total_lights_area += lights[i]->getArea();
    }

    

    // shoot a constant number of photons per unit area of light source
    // (alternatively, this could be based on the total energy of each light)
    for (unsigned int i = 0; i < lights.size(); i++) {  
        float my_area = lights[i]->getArea();
        int num = args->mesh_data->num_photons_to_shoot * my_area / total_lights_area;
        // the initial energy for this photon
        Vec3f energy = my_area / float(num) * lights[i]->getMaterial()->getEmittedColor();
        std::cout << "emitting photons with energy: " << energy << "\n";
        Vec3f normal = lights[i]->computeNormal();
        for (int j = 0; j < num; j++) {
            Vec3f start = lights[i]->RandomPoint();
            // the initial direction for this photon (for diffuse light sources)
            Vec3f direction = RandomDiffuseDirection(normal);
            TracePhoton(start, direction, energy, 0);
        }
    }

}


// ======================================================================

// helper function
bool closest_photon(const std::pair<Photon,float> &a, const std::pair<Photon,float> &b) {
    return (a.second < b.second);
}

/// This function removals all elements of the vector that do not satisfy the filter.
void FilterInPlace(std::vector<Photon>& photons, std::function<bool(const Photon&)> filter) {
    uint end = photons.size();
    for (uint i = 0; i < end;) {
        if (filter(photons[i])) {
            // we're good here, go to next element
            i++;
            continue;
        }
        // we put this element on the end
        end -= 1;
        std::swap(photons[i], photons[end]);
    }
    // everything after end must be erased
    photons.erase(photons.begin() + end, photons.end());
}

float PhotonMapping::CollectPhotons(std::vector<Photon>& result, Vec3f point, Vec3f normal) {
    float min_radius = 0;
    float max_radius = FLOAT_INFINITY;

    float radius = GLOBAL_args->mesh_data->initial_gather_radius;
    int num_photons = GLOBAL_args->mesh_data->num_photons_to_collect;

    for (int t = 0; t < max_gathering_iters; t++) {

        Vec3f extents(radius, radius, radius);
        BoundingBox bbox(point - extents, point + extents);
        
        result.clear();
        kdtree->CollectPhotonsInBox(bbox, result);
        FilterInPlace(result, [&](const Photon& p) {
            float dist = p.getPosition().Distance(point);
            if (dist >= radius) {
                return false;
            }
            float alignment = -p.getDirectionFrom().Dot3(normal);
            return alignment > photon_alignment_threshold;
        });

        int n = result.size();

        if (n == num_photons) {
            break;
        }

        if (n > num_photons) {
            max_radius = radius;
        }
        if (n < num_photons) {
            min_radius = radius;
        }

        if (std::isinf(max_radius)) {
            radius *= 2.0;
        } else {
            radius = 0.5 * (min_radius + max_radius);
        }

    }

    return radius;
}
// ======================================================================
Vec3f PhotonMapping::GatherIndirect(const Vec3f &point, const Vec3f &normal,
                                    const Vec3f &incoming_direction) {


    if (kdtree == NULL) { 
        std::cout << "WARNING: Photons have not been traced throughout the scene." << std::endl;
        return Vec3f(0,0,0); 
    }

    // =====================================================================
    // ASSIGNMENT: GATHER THE INDIRECT ILLUMINATION FROM THE PHOTON MAP
    // =====================================================================

    // collect the closest args->num_photons_to_collect photons
    // determine the radius that was necessary to collect that many photons
    // average the energy of those photons over that radius

    // ... it's much quicker to choose a fixed radius

    std::vector<Photon> photons;
    float radius = this->CollectPhotons(photons, point, normal);


    if (photons.size() == 0) {
        // probably we have too few photons, but we don't care 
        return Vec3f(0,0,0);
    }

    // average energy over area

    Vec3f total_energy(0,0,0);

    for (uint i = 0; i < photons.size(); i++) {
        total_energy += photons[i].getEnergy();
    }

    Vec3f energy = total_energy / (M_PI * radius * radius);

    if (false && photons.size() != 0) {
        std::cout << "total energy: " << total_energy << "\n";
        std::cout << "num photons: " << photons.size() << "\n";
        std::cout << "point: " << point << "\n";
        std::cout << "average energy: " << energy << "\n";
    }

    return energy;


}

// ======================================================================
// ======================================================================
// Helper functions to render the photons & kdtree

int PhotonMapping::triCount() const {
    int tri_count = 0;
    if (GLOBAL_args->mesh_data->render_kdtree == true && kdtree != NULL) {
        tri_count += kdtree->numBoxes()*12*12;
    }
    if (GLOBAL_args->mesh_data->render_photon_directions == true && kdtree != NULL) {
        tri_count += kdtree->numPhotons()*12;
    }
    return tri_count;
}

int PhotonMapping::pointCount() const {
    if (GLOBAL_args->mesh_data->render_photons == false || kdtree == NULL) return 0;
    return kdtree->numPhotons();
}

// defined in raytree.cpp
void addBox(float* &current, Vec3f start, Vec3f end, Vec3f color, float width);

// ======================================================================

void packKDTree(const KDTree *kdtree, float* &current, int &count) {
    if (!kdtree->isLeaf()) {
        if (kdtree->getChild1() != NULL) { packKDTree(kdtree->getChild1(),current,count); }
        if (kdtree->getChild2() != NULL) { packKDTree(kdtree->getChild2(),current,count); }
    } else {

        Vec3f a = kdtree->getMin();
        Vec3f b = kdtree->getMax();

        Vec3f corners[8] = {
            Vec3f(a.x(),a.y(),a.z()),
            Vec3f(a.x(),a.y(),b.z()),
            Vec3f(a.x(),b.y(),a.z()),
            Vec3f(a.x(),b.y(),b.z()),
            Vec3f(b.x(),a.y(),a.z()),
            Vec3f(b.x(),a.y(),b.z()),
            Vec3f(b.x(),b.y(),a.z()),
            Vec3f(b.x(),b.y(),b.z())
        };

        float width = 0.01 * (a-b).Length();

        addBox(current,corners[0],corners[1],Vec3f(1,1,0),width);
        addBox(current,corners[1],corners[3],Vec3f(1,1,0),width);
        addBox(current,corners[3],corners[2],Vec3f(1,1,0),width);
        addBox(current,corners[2],corners[0],Vec3f(1,1,0),width);

        addBox(current,corners[4],corners[5],Vec3f(1,1,0),width);
        addBox(current,corners[5],corners[7],Vec3f(1,1,0),width);
        addBox(current,corners[7],corners[6],Vec3f(1,1,0),width);
        addBox(current,corners[6],corners[4],Vec3f(1,1,0),width);

        addBox(current,corners[0],corners[4],Vec3f(1,1,0),width);
        addBox(current,corners[1],corners[5],Vec3f(1,1,0),width);
        addBox(current,corners[2],corners[6],Vec3f(1,1,0),width);
        addBox(current,corners[3],corners[7],Vec3f(1,1,0),width);

        count++;
    }
}

// ======================================================================

void packPhotons(const KDTree *kdtree, float* &current_points, int &count) {
    if (!kdtree->isLeaf()) {
        if (kdtree->getChild1() != NULL) { packPhotons(kdtree->getChild1(),current_points,count); }
        if (kdtree->getChild2() != NULL) { packPhotons(kdtree->getChild2(),current_points,count); }
    } else {
        for (unsigned int i = 0; i < kdtree->getPhotons().size(); i++) {
            const Photon &p = kdtree->getPhotons()[i];
            Vec3f v = p.getPosition();
            Vec3f color = p.getEnergy()*float(GLOBAL_args->mesh_data->num_photons_to_shoot);
            float12 t = { float(v.x()),float(v.y()),float(v.z()),1,   0,0,0,0,   float(color.r()),float(color.g()),float(color.b()),1 };
            memcpy(current_points, &t, sizeof(float)*12); current_points += 12; 
            count++;
        }
    }
}


void packPhotonDirections(const KDTree *kdtree, float* &current, int &count) {
    if (!kdtree->isLeaf()) {
        if (kdtree->getChild1() != NULL) { packPhotonDirections(kdtree->getChild1(),current,count); }
        if (kdtree->getChild2() != NULL) { packPhotonDirections(kdtree->getChild2(),current,count); }
    } else {
        for (unsigned int i = 0; i < kdtree->getPhotons().size(); i++) {
            const Photon &p = kdtree->getPhotons()[i];
            Vec3f v = p.getPosition();
            Vec3f v2 = p.getPosition() - p.getDirectionFrom() * 0.5;
            Vec3f color = p.getEnergy()*float(GLOBAL_args->mesh_data->num_photons_to_shoot);
            float width = 0.01;
            addBox(current,v,v2,color,width);
            count++;
        }
    }
}

// ======================================================================

void PhotonMapping::packMesh(float* &current, float* &current_points) {

    // the photons
    if (GLOBAL_args->mesh_data->render_photons && kdtree != NULL) {
        int count = 0;
        packPhotons(kdtree,current_points,count);
        assert (count == kdtree->numPhotons());
    }
    // photon directions
    if (GLOBAL_args->mesh_data->render_photon_directions && kdtree != NULL) {
        int count = 0;
        packPhotonDirections(kdtree,current,count);
        assert (count == kdtree->numPhotons());
    }

    // the wireframe kdtree
    if (GLOBAL_args->mesh_data->render_kdtree && kdtree != NULL) {
        int count = 0;
        packKDTree(kdtree,current,count);
        assert (count == kdtree->numBoxes());
    }

}

// ======================================================================
