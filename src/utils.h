#ifndef _UTILS_H
#define _UTILS_H

#include <vector>
#include "vectors.h"
#include "meshdata.h"
#include "material.h"

#include "argparser.h"

// ======================================================================

#define square(x) ((x)*(x))
//// helper for VBOs
//#define BUFFER_OFFSET(i) ((char *)NULL + (i))

// needed by Windows
// allows us to use std::min & std::max
#define NOMINMAX


// =========================================================================
// EPSILON is a necessary evil for raytracing implementations
// The appropriate value for epsilon depends on the precision of
// the floating point calculations on your hardware **AND** on the
// overall dimensions of the scene and your camera projection matrix.
#define EPSILON 0.01


#define FLOAT_INFINITY std::numeric_limits<float>::infinity()


// =========================================================================
// These two functions convert between linear intensity values
// (approximate range 0->1) to an sRGB value (approximate range 0->1).
// The sRGB values make best use of 8 bit storage and are the best
// input for most displays and darkened viewing environments.

#define SRGB_ALPHA 0.055

inline double clamp01(double x) {
    if (x < 0.0) {
        return 0.0;
    }
    if (1.0 < x) {
        return 1.0;
    }
    return x;
};

inline float linear_to_srgb(float x) {
  float answer;
  if (x <= 0.0031308)
    answer = 12.92*x;
  else 
    answer = (1+SRGB_ALPHA)*(pow(x,1/2.4)-SRGB_ALPHA);
  return answer;
}

inline float srgb_to_linear(float x) {
  float answer;
  if (x <= 0.04045)
    answer = x/12.92;
  else 
    answer = pow((x+SRGB_ALPHA)/(1+SRGB_ALPHA),2.4);
  return answer;
}

// =========================================================================
// utility functions 
inline float DistanceBetweenTwoPoints(const Vec3f &p1, const Vec3f &p2) {
  Vec3f v = p1-p2;
  return v.Length();
}

inline float AreaOfTriangle(float a, float b, float c) {
  // from the lengths of the 3 edges, compute the area
  // Area of Triangle = (using Heron's Formula)
  //  sqrt[s*(s-a)*(s-b)*(s-c)]
  //    where s = (a+b+c)/2
  // also... Area of Triangle = 0.5 * x * c
  float s = (a+b+c) / (float)2;
  return sqrt(s*(s-a)*(s-b)*(s-c));
}

inline float AreaOfTriangle(const Vec3f &a, const Vec3f &b, const Vec3f &c) {
  float aside = DistanceBetweenTwoPoints(a,b);
  float bside = DistanceBetweenTwoPoints(b,c);
  float cside = DistanceBetweenTwoPoints(c,a);
  return AreaOfTriangle(aside,bside,cside);
}

inline Vec3f ComputeNormal(const Vec3f &p1, const Vec3f &p2, const Vec3f &p3) {
  Vec3f v12 = p2;
  v12 -= p1;
  Vec3f v23 = p3;
  v23 -= p2;
  Vec3f normal;
  Vec3f::Cross3(normal,v12,v23);
  normal.Normalize();
  return normal;
}

inline double triInterpolate(double x_frac, double y_frac, double z_frac,
			     double a, double b, double c, double d, double e, double f, double g, double h) {
  
  assert (x_frac >= 0 && x_frac <= 1);
  assert (y_frac >= 0 && y_frac <= 1);
  assert (z_frac >= 0 && z_frac <= 1);

  // trilinear interpolation
  double ab = (1-z_frac)*a + z_frac*b;
  double cd = (1-z_frac)*c + z_frac*d;
  double ef = (1-z_frac)*e + z_frac*f;
  double gh = (1-z_frac)*g + z_frac*h;
  double abcd = (1-y_frac)*ab + y_frac*cd;
  double efgh = (1-y_frac)*ef + y_frac*gh;
  double abcdefgh = (1-x_frac)*abcd + x_frac*efgh;
  return abcdefgh;
}

// utility function to generate random numbers used for sampling
inline Vec3f RandomUnitVector() {
  Vec3f tmp;
  while (true) {
    tmp = Vec3f(2*GLOBAL_args->rand()-1,  // random real in [-1,1]
                2*GLOBAL_args->rand()-1,  // random real in [-1,1]
                2*GLOBAL_args->rand()-1); // random real in [-1,1]
    if (tmp.Length() < 1) break;
  }
  tmp.Normalize();
  return tmp;
}

// compute the perfect mirror direction
inline Vec3f MirrorDirection(const Vec3f &normal, const Vec3f &incoming) {
  float dot = incoming.Dot3(normal);
  Vec3f r = (incoming*-1.0f) + normal * (2 * dot);
  return r*-1.0f;
}

// compute the refracted ray direction
// normal - the vector normal to the tangent plane of the surface
// incoming - a unit vector in the direction of the incoming light ray
// eta - the ratio of refractive indices
inline Vec3f RefractedDirection(const Vec3f &normal, const Vec3f &incoming, Material* incoming_mat, Material* outgoing_mat) {
    // snell's law refraction

    // compute refractive indices (nullptr is assumed to be air)
    float incoming_index = incoming_mat? incoming_mat->getRefractiveIndex() : 1.0;
    if (incoming_mat == outgoing_mat) {
        // NOTE: we assume that the second time we hit a material we are exiting the shape
        outgoing_mat = nullptr; 
    }
    float outgoing_index = outgoing_mat? outgoing_mat->getRefractiveIndex() : 1.0;

    float eta = incoming_index / outgoing_index;

    // std::cout << "refracting, origin = " << origin_material << ", dest = " << dest_mat << " eta = " << eta << "\n";

    // lies in the tangent plane
    auto tangent0 = normal.Cross(incoming); // what to do if incoming and normal are parallel?
    // a vector that lies in the tangent plane, in the same direction as the incoming ray
    // together with tangent0, it forms a basis for the tangent plane
    auto tangent = tangent0.Cross(normal);
    tangent.Normalize();

    // {tangent0, tangent, normal} form a frame oriented in the direction of the incoming light ray

    // the normal component of the incoming ray (= cos(theta_I))
    float normal_comp = incoming.Dot3(normal);

    // the tangent component of the incoming ray (= sin(theta_I))
    float tangent_comp = incoming.Dot3(tangent);


    // the transmitted direction has the negative normal component (it goes into the surface)
    // and its tangent component is scaled by the refractive ratio, eta
    Vec3f transmitted = normal_comp * normal + eta * tangent_comp * tangent;

    /*
    std::cout << "in refracted direction, "
              << "\n eta = " << eta 
              << "\n incoming = " << incoming 
              << "\n tangent = " << tangent << " (" << tangent_comp << ")"
              << "\n normal = " << normal << " (" << normal_comp << ")"
              << "\n transmitted = " << transmitted 
        << "\n";
        */

    return transmitted;
}

// compute a random diffuse direction
// (not the same as a uniform random direction on the hemisphere)
inline Vec3f RandomDiffuseDirection(const Vec3f &normal) {
  Vec3f answer = normal+RandomUnitVector();
  answer.Normalize();
  return answer;
}


// ===================================================
// Rendering utilities


void AddWireFrameTriangle(float* &current,
                          const Vec3f &apos, const Vec3f &bpos, const Vec3f &cpos,
                          const Vec3f &anormal, const Vec3f &bnormal, const Vec3f &cnormal,
                          const Vec3f &color,
                          const Vec3f &abcolor, const Vec3f &bccolor, const Vec3f &cacolor);

void AddQuad(float* &current,
             const Vec3f &apos, const Vec3f &bpos, const Vec3f &cpos, const Vec3f &dpos,
             const Vec3f &normal,
             const Vec3f &color);


void AddBox(float* &current,
            const Vec3f pos[8],
            const Vec3f &color);

// Creates an edge with 12 triangles.
void addEdgeGeometry(float* &current,
                     const Vec3f &a, const Vec3f &b, 
                     const Vec3f &acolor, const Vec3f &bcolor, 
                     float a_th,float b_th);

// Creates a cube with 12 triangles.
void addCubeGeometry(float* &current,
                     Vec3f pts[8],
                     const Vec3f &acolor);



#endif
