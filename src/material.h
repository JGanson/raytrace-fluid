#ifndef _MATERIAL_H_
#define _MATERIAL_H_

#include <cassert>
#include <string>

#include "vectors.h"
#include "image.h"

class Ray;
class Hit;

// ====================================================================
// ====================================================================
// A simple Phong-like material 

class Material {

public:

    Material(const std::string& path, std::ifstream& ifstr);

    Material(const std::string &texture_file, const Vec3f &d_color,
             const Vec3f &r_color, const Vec3f &e_color, float roughness_, Vec3f refraction_, float refractive_index) {
        textureFile = texture_file;
        reflectiveColor = r_color;
        emittedColor = e_color;
        roughness = roughness_;
        refractiveColor = refraction_;
        refractiveIndex = refractive_index;

        Initialize();
        // need to initialize texture_id after glut has started
        //texture_id = 0;
    }

    ~Material();

    // ACCESSORS
    const Vec3f& getDiffuseColor() const { return diffuseColor; }
    const Vec3f getDiffuseColor(float s, float t) const;
    const Vec3f& getReflectiveColor() const { return reflectiveColor; }
    const Vec3f& getEmittedColor() const { return emittedColor; }  
    float getRoughness() const { return roughness; } 
    const Vec3f& getRefractiveColor() const { return refractiveColor; }
    float getRefractiveIndex() const { return refractiveIndex; }
    bool hasTextureMap() const { return (textureFile != ""); } 
    //GLuint getTextureID();

    const std::string& getFileName() const { return filename; }

    // SHADE
    // compute the contribution to local illumination at this point for
    // a particular light source
    Vec3f Shade(const Ray &ray, const Hit &hit, const Vec3f &dirToLight, 
     const Vec3f &lightColor) const;


    // OUTPUT
    friend std::ostream& operator<< (std::ostream &ostr, const Material &m) {
        ostr << "Material \"" << m.filename << "\" {"
            << " diffuseColor = "    << m.diffuseColor
            << " reflectiveColor = " << m.reflectiveColor
            << " emittedColor = "    << m.emittedColor
            << " roughness = "       << m.roughness
            << " transparency = "    << m.refractiveColor
            << " refractiveIndex = " << m.refractiveIndex;

        if (m.hasTextureMap()) {
            ostr << " textureFile = \"" << m.textureFile << "\"";
        } else {
            ostr << " no textureFile";
        }
        ostr << "}";
        return ostr;
    }

protected:

    Material() { exit(0); }
    Material(const Material&) { exit(0); }
    const Material& operator=(const Material&) { exit(0); }


    void Initialize(); 

    void ComputeAverageTextureColor();

    // REPRESENTATION
    Vec3f diffuseColor;
    Vec3f reflectiveColor;
    float specularExponent;
    Vec3f emittedColor;
    float roughness;
    Vec3f refractiveColor; // fraction of light (from 0.0 to 1.0) that passes through this material
    float refractiveIndex = 1.0;
    bool isTransparent = false;


    // a unique identifier
    std::string filename;

    std::string textureFile;
    //GLuint texture_id;
    Image *image;
};

// ====================================================================
// ====================================================================

#endif

