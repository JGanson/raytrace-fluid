#include "material.h"
#include "utils.h"
#include "raytrace/ray.h"
#include "raytrace/hit.h"

// CONSTRUCTOR
Material::Material(const std::string& path, std::ifstream& ifstr) {
    std::string token;

    textureFile = "";
    refractiveIndex = 1.0;
    specularExponent = 100;

    while (ifstr >> token) {
        if (token == "#") {
            // this line is a comment and is ignored
            getline(ifstr, token);
            continue;
        }
        if (token == "diffuse") {
            float r, g, b;
            ifstr >> r >> g >> b;
            diffuseColor = Vec3f(r,g,b);
        }
        else if (token == "texture_file") {
            ifstr >> textureFile;
            // prepend the directory name
            textureFile = path + '/' + textureFile;
        }
        else if (token == "reflective") {
            float r, g, b;
            ifstr >> r >> g >> b;
            reflectiveColor = Vec3f(r,g,b);
        }
        else if (token == "roughness") {
            ifstr >> roughness;
        } 
        else if (token == "refractive") {
            float r, g, b;
            ifstr >> r >> b >> g;
            refractiveColor = Vec3f(r,g,b);
        }
        else if (token == "index") {
            ifstr >> refractiveIndex;
        }
        else if (token == "specularExponent") {
            ifstr >> specularExponent;
        }
        else if (token == "emitted") {
            float r, g, b;
            ifstr >> r >> g >> b;
            emittedColor = Vec3f(r,g,b);
        } else if (token == "transparent") {
            ifstr >> isTransparent;
        } else {
            std::cerr << "Error: unrecognized material parameter '" << token << "', skipping...\n";
        }
    }


    Initialize();
}

void Material::Initialize() {
    if (textureFile != "") {
        image = new Image(textureFile);
        ComputeAverageTextureColor();
    } else {
        image = NULL;
    }
}

// ==================================================================
// DESTRUCTOR
// ==================================================================
Material::~Material() {
  if (hasTextureMap()) {
    //glDeleteTextures(1,&texture_id);
    assert (image != NULL);
    delete image;
  }
}

// ==================================================================
// TEXTURE LOOKUP FOR DIFFUSE COLOR
// ==================================================================
const Vec3f Material::getDiffuseColor(float s, float t) const {
  if (!hasTextureMap()) return diffuseColor; 

  assert (image != NULL);

  // this is just using nearest neighbor and could be improved to
  // bilinear interpolation, etc.
  int i = int(s * image->Width()) % image->Width();
  int j = int(t * image->Height()) % image->Height();
  if (i < 0) i += image->Width();
  if (j < 0) j += image->Height();
  assert (i >= 0 && i < image->Width());
  assert (j >= 0 && j < image->Height());
  Color c = image->GetPixel(i,j);

  // we assume the texture is stored in sRGB and convert to linear for
  // computation.  It will be converted back to sRGB before display.
  float r = srgb_to_linear(c.r/255.0);
  float g = srgb_to_linear(c.g/255.0);
  float b = srgb_to_linear(c.b/255.0);

  return Vec3f(r,g,b);
}

/*
// ==================================================================
// OpenGL setup for textures
// ==================================================================
GLuint Material::getTextureID() { 
  assert (hasTextureMap()); 

  // if this is the first time the texture is being used, we must
  // initialize it
  if (texture_id == 0)  {
    glGenTextures(1,&texture_id);
    assert (texture_id != 0);
    glBindTexture(GL_TEXTURE_2D, texture_id);
    // select modulate to mix texture with color for shading
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    // or decal to not mix local shading
    //glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
    // when texture area is small, bilinear filter the closest mipmap
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
		     GL_LINEAR_MIPMAP_NEAREST );
    // when texture area is large, bilinear filter the original
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    // the texture wraps over at the edges (repeat)
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );
    // to be most compatible, textures should be square and a power of 2
    assert (image->Width() == image->Height());
    assert (image->Width() == 256);
    // build our texture mipmaps
    //gluBuild2DMipmaps( GL_TEXTURE_2D, 3, image->Width(), image->Height(),
    //		       GL_RGB, GL_UNSIGNED_BYTE, image->getGLPixelData());
  }
  
  return texture_id;
}
*/

// ==================================================================
// An average texture color, a hack for use in radiosity
// ==================================================================
void Material::ComputeAverageTextureColor() {
  assert (hasTextureMap());
  float r = 0;
  float g = 0;
  float b = 0;
  for (int i = 0; i < image->Width(); i++) {
    for (int j = 0; j < image->Height(); j++) {
      Color c = image->GetPixel(i,j);
       r += srgb_to_linear(c.r/255.0);
       g += srgb_to_linear(c.g/255.0);
       b += srgb_to_linear(c.b/255.0);
    }
  }
  int count = image->Width() * image->Height();
  r /= float(count);
  g /= float(count);
  b /= float(count);
  diffuseColor = Vec3f(r,g,b);
}

// ==================================================================
// PHONG LOCAL ILLUMINATION

// this function should be called to compute the light contributed by
// a particular light source to the intersection point.  Note that
// this function does not calculate any global effects (e.g., shadows). 

Vec3f Material::Shade(const Ray &ray, const Hit &hit, 
                      const Vec3f &dirToLight, 
                      const Vec3f &lightColor) const {
  
  Vec3f n = hit.getNormal();
  Vec3f e = ray.getDirection()*-1.0f;
  Vec3f l = dirToLight;
  
  Vec3f answer = Vec3f(0,0,0);

  // emitted component
  // -----------------
  answer += getEmittedColor();

  // diffuse component
  // -----------------
  float dot_nl = n.Dot3(l);
  if (dot_nl < 0) dot_nl = 0;
  answer += lightColor * getDiffuseColor(hit.get_s(),hit.get_t()) * dot_nl;

  // specular component (Phong)
  // ------------------
  // make up reasonable values for other Phong parameters
  Vec3f specularColor = reflectiveColor;

  // compute ideal reflection angle
  Vec3f r = (l*-1.0f) + n * (2 * dot_nl);
  r.Normalize();
  float dot_er = e.Dot3(r);
  if (dot_er < 0) dot_er = 0;
  answer += lightColor*specularColor*float(pow(dot_er,specularExponent))* dot_nl;

  return answer;
}

// ==================================================================
