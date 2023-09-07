#include <chrono>
#include <ctime>

#include "../material.h"
#include "../argparser.h"
#include "../utils.h"
#include "../mesh.h"
#include "../face.h"
#include "../primitive.h"
#include "../boundingbox.h"
#include "../camera.h"
#include "../fluid/fluid.h"
#include "../fluid/marching_cubes.h"
#include "raytracer.h"
#include "raytree.h"
#include "photon_mapping.h"
#include "bvh.h"


void RayTracer::SetupRaytracedGeometry() {
    std::cout << "Rebuilding BVH...\n";
    std::vector<Primitive*> primitives;

    // each quad in the mesh
    std::cout << "Collecting original quads...\n";
    for (int i = 0; i < mesh->numOriginalQuads(); i++) {
        Face *f = mesh->getOriginalQuad(i);
        primitives.push_back((Primitive*) f);
    }

    // each primitive object
    std::cout << "Collecting primitives quads...\n";
    for (int i = 0; i < mesh->numPrimitives(); i++) {
        Primitive *p = mesh->getPrimitive(i);
        primitives.push_back(p);
    }

    if (args->fluid) {
        std::cout << "Creating fluid surface...\n";
        MarchingCubes* mc = args->fluid->GenerateMarchingCubesSurface();

        std::cout << "Collecting surface triangles...\n";
        for (int i = 0; i < mc->numMCTris(); i++) {
            // This is fine because we don't change the fluid surface 
            // while the raytracing is happening.
            Primitive *p = mc->getTri(i);
            primitives.push_back(p);
        }

    }

    // each fluid face
    // TODO:
    //   args->fluid->getSurfaceGeometry()
    
    std::cout << "Balancing tree (" << primitives.size() << " primitives)...\n";

    delete bvh;
    bvh = BVH::Build(args, primitives);
}

// ===========================================================================
// does the recursive (shadow rays & recursive rays) work
Vec3f RayTracer::TraceRay(Ray &ray, Hit &hit, int bounce_count, Material* origin_material) const {

    // First cast a ray and see if we hit anything.
    hit = Hit();
    bool intersect = bvh->CastRay(ray, hit);

    // if there is no intersection, simply return the background color
    if (intersect == false) {
        return Vec3f(srgb_to_linear(args->background_color.r()),
                     srgb_to_linear(args->background_color.g()),
                     srgb_to_linear(args->background_color.b()));
    }


    // otherwise decide what to do based on the material
    Material *hit_mat = hit.getMaterial();
    assert (hit_mat != NULL);

    // rays coming from the light source are set to white, don't bother to ray trace further.
    if (hit_mat->getEmittedColor().Length() > 0.001) {
        return Vec3f(1,1,1);
    } 

    Vec3f normal = hit.getNormal();
    Vec3f hit_point = ray.pointAtParameter(hit.getT());
    
    // we will wish to obtain points that are infinitessimally above/below the surface,
    // with respect to the ray direction.
    // Project the ray direction onto the normal vector

    Vec3f answer;

    Vec3f ambient_light = Vec3f(args->mesh_data->ambient_light.data[0],
                                args->mesh_data->ambient_light.data[1],
                                args->mesh_data->ambient_light.data[2]);

    // ===================================================
    //      DIFFUSE AMBIENT LIGHT 
    // ===================================================

    Vec3f diffuse_color = hit_mat->getDiffuseColor(hit.get_s(), hit.get_t());
    // TODO: actually check this !
    if (args->mesh_data->gather_indirect) {
        // photon mapping for more accurate indirect light
        answer = diffuse_color * (photon_mapping->GatherIndirect(hit_point, normal, ray.getDirection()) + ambient_light);
    } else {
        // the usual ray tracing hack for indirect light
        answer = diffuse_color * ambient_light;
    }      

    // ===================================================
    // ASSIGNMENT:  ADD SHADOW & SOFT SHADOW LOGIC (DONE)
    // ===================================================

    int num_lights = mesh->getLights().size();
    for (int i = 0; i < num_lights; i++) {
        // add contributions from each light that is not in shadow

        // the light source
        Face *source = mesh->getLights()[i];

        // the color emitted from that light
        Vec3f emitted_color = source->getMaterial()->getEmittedColor() * source->getArea();

        int num_samples = GLOBAL_args->mesh_data->num_shadow_samples;

        bool do_occlusions = num_samples > 0;
        if (num_samples == 0) {
            num_samples = 1;
        }


        for (int t = 0; t < num_samples; t++) {
            Vec3f point_source;
            if (num_samples == 1) {
                point_source = source->computeCentroid();
            } else {
                point_source = source->RandomPoint();
            }
            // direction from out point to the light's point_source
            Vec3f dir = (point_source - hit_point).Normalized();

            if (do_occlusions) {
                Ray to_light_source = Ray(hit_point, dir);

                // the ray needs to hit something
                Hit light_hit;
                if (!bvh->CastRay(to_light_source, light_hit)) {
                    continue; // no intersection = no light
                }
                RayTree::AddShadowSegment(to_light_source, 0, light_hit.getT());
                Vec3f p = to_light_source.pointAtParameter(light_hit.getT());
                if (p.Distance(point_source) > EPSILON) { 
                    // we consider that the light ray was likely obstructed by something
                    continue;
                }
            }

            // and it needs to hit our object !
            // distance to the light point_source
            float dist = hit_point.Distance(point_source);
            Vec3f my_light_color = 1.0 / float (M_PI * dist * dist) * emitted_color;

            // add the lighting contribution from this particular light at this point
            // (fix this to check for blockers between the light & this surface)
            answer += hit_mat->Shade(ray, hit, dir, my_light_color) * (1.0 / num_samples);
        }
    }


    // ==============================================
    //   REFLECTIONS
    // ==============================================

    // add contribution from reflection, if the surface is shiny
    Vec3f reflectiveColor = hit_mat->getReflectiveColor();
    if (bounce_count > 0 && reflectiveColor.SumOfComponents() > 0.0) {

        Vec3f normal_comp = ray.getDirection().Dot3(normal) * normal;
        Vec3f reflected_dir = ray.getDirection() - 2 * normal_comp;
        
        // the origin is added a little bit in order to fix the problem where reflected rays bounce around inside the object
        auto reflected_ray = Ray(hit_point, reflected_dir);
        
        Hit reflected_hit = Hit();
        answer += reflectiveColor * TraceRay(reflected_ray, reflected_hit, bounce_count - 1, hit_mat);

        RayTree::AddReflectedSegment(reflected_ray, 0.0, reflected_hit.getT());
    }

    // ==============================================
    //    REFRACTION
    // ==============================================

    if (hit_mat->getRefractiveColor().SumOfComponents() > 0.0) {
        Vec3f dir = RefractedDirection(normal, ray.getDirection(), origin_material, hit_mat);

        // the ray transmitted through the surface
        auto trans_ray = Ray(hit_point, dir); 

        Hit trans_hit = Hit();
        Vec3f trans_color = TraceRay(trans_ray, trans_hit, bounce_count - 1, hit_mat);

        // the segment of the ray transmitted through the material
        RayTree::AddTransmittedSegment(trans_ray, 0.0, trans_hit.getT());

        Vec3f t = hit_mat->getRefractiveColor();
        answer = t * trans_color + (Vec3f(1,1,1) - t) * answer;
    }

    
    return answer; 
}

// trace a ray through pixel (i,j) of the image an return the color
Vec3f VisualizeTraceRay(double i, double j) {
    int num_bounces = GLOBAL_args->mesh_data->num_bounces;
    Camera *camera = GLOBAL_args->camera;
    RayTracer *raytracer = GLOBAL_args->raytracer;
    // compute and set the pixel color
    int width = GLOBAL_args->mesh_data->width;
    int height = GLOBAL_args->mesh_data->height;
    double max_dim = mymax(width, height);

    Vec3f color;

    // ===========================================
    // ASSIGNMENT: IMPLEMENT ANTIALIASING (DONE)
    // ===========================================

    int num_samples = GLOBAL_args->mesh_data->num_antialias_samples;
    for (int t = 0; t < num_samples; t++) {
        // the position (in pixels) of the center of the pixel
        double x = i;
        double y = j;
        if (num_samples != 1) {
            // if there is more than one sample, offset it by a random number in [-0.5, 0.5]
            x += GLOBAL_args->rand() - 0.5;
            y += GLOBAL_args->rand() - 0.5;
        }
        // convert to percentile screen coordinates
        x = (x - 0.5 * width) / max_dim + 0.5;
        y = (y - 0.5 * height) / max_dim + 0.5;
        Ray r = camera->generateRay(x,y);
        Hit hit;
        color += raytracer->TraceRay(r, hit, num_bounces);

        // add that ray for visualization
        RayTree::AddMainSegment(r,0,hit.getT());
    }

    color /= num_samples;

    // return the color
    return color;
}




// for visualization: find the "corners" of a pixel on an image plane
// 1/2 way between the camera & point of interest
Vec3f PixelGetPos(double i, double j) {
    int max_d = mymax(GLOBAL_args->mesh_data->width,GLOBAL_args->mesh_data->height);
    double x = (i-GLOBAL_args->mesh_data->width/2.0)/double(max_d)+0.5;
    double y = (j-GLOBAL_args->mesh_data->height/2.0)/double(max_d)+0.5;
    Camera *camera = GLOBAL_args->camera;
    Ray r = camera->generateRay(x,y); 
    Vec3f cp = camera->camera_position;
    Vec3f poi = camera->point_of_interest;
    float distance = (cp-poi).Length()/2.0f;
    return r.getOrigin()+distance*r.getDirection();
}


// Scan through the image from the lower left corner across each row
// and then up to the top right.  Initially the image is sampled very
// coarsely.  Increment the static variables that track the progress
// through the scans
int RayTraceDrawPixel() {
    if (GLOBAL_args->mesh_data->raytracing_x >= GLOBAL_args->mesh_data->raytracing_divs_x) {
        // end of row
        GLOBAL_args->mesh_data->raytracing_x = 0; 
        GLOBAL_args->mesh_data->raytracing_y += 1;
    }
    if (GLOBAL_args->mesh_data->raytracing_y >= GLOBAL_args->mesh_data->raytracing_divs_y) {
        // last row
        if (GLOBAL_args->mesh_data->raytracing_divs_x >= GLOBAL_args->mesh_data->width ||
            GLOBAL_args->mesh_data->raytracing_divs_y >= GLOBAL_args->mesh_data->height) {
            // stop rendering, matches resolution of current camera
            return 0; 
        }
        // else decrease pixel size & start over again in the bottom left corner
        GLOBAL_args->mesh_data->raytracing_divs_x *= 3;
        GLOBAL_args->mesh_data->raytracing_divs_y *= 3;
        if (GLOBAL_args->mesh_data->raytracing_divs_x > GLOBAL_args->mesh_data->width * 0.51 ||
            GLOBAL_args->mesh_data->raytracing_divs_x > GLOBAL_args->mesh_data->height * 0.51) {
            GLOBAL_args->mesh_data->raytracing_divs_x = GLOBAL_args->mesh_data->width;
            GLOBAL_args->mesh_data->raytracing_divs_y = GLOBAL_args->mesh_data->height;
        }
        GLOBAL_args->mesh_data->raytracing_x = 0;
        GLOBAL_args->mesh_data->raytracing_y = 0;

        if (GLOBAL_args->raytracer->render_to_a) {
            GLOBAL_args->raytracer->pixels_b.clear();
            GLOBAL_args->raytracer->render_to_a = false;
        } else {
            GLOBAL_args->raytracer->pixels_a.clear();
            GLOBAL_args->raytracer->render_to_a = true;
        }
    }

    double x_spacing = GLOBAL_args->mesh_data->width / double (GLOBAL_args->mesh_data->raytracing_divs_x);
    double y_spacing = GLOBAL_args->mesh_data->height / double (GLOBAL_args->mesh_data->raytracing_divs_y);

    // compute the color and position of intersection
    Vec3f pos1 =  PixelGetPos((GLOBAL_args->mesh_data->raytracing_x  )*x_spacing, (GLOBAL_args->mesh_data->raytracing_y  )*y_spacing);
    Vec3f pos2 =  PixelGetPos((GLOBAL_args->mesh_data->raytracing_x+1)*x_spacing, (GLOBAL_args->mesh_data->raytracing_y  )*y_spacing);
    Vec3f pos3 =  PixelGetPos((GLOBAL_args->mesh_data->raytracing_x+1)*x_spacing, (GLOBAL_args->mesh_data->raytracing_y+1)*y_spacing);
    Vec3f pos4 =  PixelGetPos((GLOBAL_args->mesh_data->raytracing_x  )*x_spacing, (GLOBAL_args->mesh_data->raytracing_y+1)*y_spacing);

    Vec3f color = VisualizeTraceRay(
            (GLOBAL_args->mesh_data->raytracing_x + 0.5) * x_spacing,
            (GLOBAL_args->mesh_data->raytracing_y + 0.5) * y_spacing
    );

    double r = linear_to_srgb(color.r());
    double g = linear_to_srgb(color.g());
    double b = linear_to_srgb(color.b());

    Pixel p;
    p.v1 = pos1;
    p.v2 = pos2;
    p.v3 = pos3;
    p.v4 = pos4;
    p.color = Vec3f(r,g,b);

    if (GLOBAL_args->raytracer->render_to_a) {
        GLOBAL_args->raytracer->pixels_a.push_back(p);
    } else {
        GLOBAL_args->raytracer->pixels_b.push_back(p);
    }  

    GLOBAL_args->mesh_data->raytracing_x += 1;
    return 1;
}

Image DrawRayTracedImage() {
    GLOBAL_args->raytracer->SetupRaytracedGeometry();

    std::cout << "Beginning render...\n";


    Image img;
    img.Allocate(GLOBAL_args->mesh_data->width, GLOBAL_args->mesh_data->height);

    auto start_time = std::chrono::system_clock::now();

    int count = 0;

    for (int y = 0; y < img.Height(); y++) {
        for (int x = 0; x < img.Width(); x++) {

            Vec3f color = VisualizeTraceRay((float)x + 0.5, (float)y + 0.5);

            double r = linear_to_srgb(color.r());
            double g = linear_to_srgb(color.g());
            double b = linear_to_srgb(color.b());

            img.SetPixel(x, y, Color::From01(r,g,b));

        }
        count += img.Width();
        if (count > 0.1 * img.Height() * img.Width()) {
            count = 0;
            float pct = y / float(img.Height());
            std::cout << "Progress: " << 100.0 * pct << "%\n";
        }
    }

    auto elapsed = std::chrono::system_clock::now() - start_time;
    auto minutes = std::chrono::duration_cast<std::chrono::minutes>(elapsed);
    std::chrono::duration<double> seconds = elapsed - minutes;

    std::cout << "Finished rendering in " << minutes.count() << "m " << seconds.count() << "s.\n";

    std::string output_path = GLOBAL_args->path + '/' + GLOBAL_args->output_filename;

    std::cout << "Writing image data to " << output_path << "\n";
    img.Save(output_path);

    std::cout << "done.\n";

    return img;
}

// ===========================================================================

int RayTracer::triCount() {
    int count = 0;
    // the pixels
    count += (pixels_a.size() + pixels_b.size()) * 2;
    // the BVH (i.e. just a coupla bounding boxes for now)
    if (args->mesh_data->render_bvh) {
        count += bvh->triCount();
    }
    return count;
}

void RayTracer::packMesh(float* &current) {
    packPixels(current);
    if (args->mesh_data->render_bvh) {
        bvh->packMesh(current);
    }
}

void RayTracer::packPixels(float* &current) {
    for (unsigned int i = 0; i < pixels_a.size(); i++) {
        Pixel &p = pixels_a[i];
        Vec3f v1 = p.v1;
        Vec3f v2 = p.v2;
        Vec3f v3 = p.v3;
        Vec3f v4 = p.v4;
        Vec3f normal = ComputeNormal(v1,v2,v3) + ComputeNormal(v1,v3,v4);
        normal.Normalize();
        if (render_to_a) {
            v1 += 0.02*normal;
            v2 += 0.02*normal;
            v3 += 0.02*normal;
            v4 += 0.02*normal;
        }
        normal = Vec3f(0,0,0);
        AddQuad(current,v1,v2,v3,v4,normal,p.color);
    }

    for (unsigned int i = 0; i < pixels_b.size(); i++) {
        Pixel &p = pixels_b[i];
        Vec3f v1 = p.v1;
        Vec3f v2 = p.v2;
        Vec3f v3 = p.v3;
        Vec3f v4 = p.v4;
        Vec3f normal = ComputeNormal(v1,v2,v3) + ComputeNormal(v1,v3,v4);
        normal.Normalize();
        if (!render_to_a) {
            v1 += 0.02*normal;
            v2 += 0.02*normal;
            v3 += 0.02*normal;
            v4 += 0.02*normal;
        }
        normal = Vec3f(0,0,0);
        AddQuad(current,v1,v2,v3,v4,normal,p.color);
    }
}

// ===========================================================================
