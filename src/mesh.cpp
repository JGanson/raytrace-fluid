#include <iostream>
#include <fstream>
#include <assert.h>
#include <string>
#include <utility>
#include <sstream>
#include <stdlib.h>

#include "argparser.h"
#include "vertex.h"
#include "boundingbox.h"
#include "box_primitive.h"
#include "mesh.h"
#include "edge.h"
#include "face.h"
#include "primitive.h"
#include "sphere.h"
#include "cylinder_ring.h"
#include "utils.h"
#include "raytrace/ray.h"
#include "raytrace/hit.h"
#include "fluid/fluid.h"


// =======================================================================
// DESTRUCTOR
// =======================================================================

Mesh::~Mesh() {
    unsigned int i;
    for (i = 0; i < rasterized_primitive_faces.size(); i++) {
        Face *f = rasterized_primitive_faces[i];
        removeFaceEdges(f);
        delete f;
    }
    if (subdivided_quads.size() != original_quads.size()) {
        for (i = 0; i < subdivided_quads.size(); i++) {
            Face *f = subdivided_quads[i];
            removeFaceEdges(f);
            delete f;
        }
    }
    for (i = 0; i < original_quads.size(); i++) {
        Face *f = original_quads[i];
        removeFaceEdges(f);
        delete f;
    }
    for (i = 0; i < primitives.size(); i++) { delete primitives[i]; }
    for (i = 0; i < materials.size(); i++) { delete materials[i]; }
    for (i = 0; i < vertices.size(); i++) { delete vertices[i]; }
    delete bbox;
}

// =======================================================================
// MODIFIERS:   ADD & REMOVE
// =======================================================================

Vertex* Mesh::addVertex(const Vec3f &position) {
    int index = numVertices();
    vertices.push_back(new Vertex(index,position));
    // extend the bounding box to include this point
    if (bbox == NULL) 
        bbox = new BoundingBox(position,position);
    else 
        bbox->Extend(position);
    return vertices[index];
}

void Mesh::addPrimitive(Primitive* p) {
    primitives.push_back(p);
    p->addRasterizedFaces(this,args);
}

Face* Mesh::addFace(Vertex *a, Vertex *b, Vertex *c, Vertex *d, Material *material, enum FACE_TYPE face_type) {
    // create the face
    Face *f = new Face(material);
    // create the edges
    Edge *ea = new Edge(a,b,f);
    Edge *eb = new Edge(b,c,f);
    Edge *ec = new Edge(c,d,f);
    Edge *ed = new Edge(d,a,f);
    // point the face to one of its edges
    f->setEdge(ea);
    // connect the edges to each other
    ea->setNext(eb);
    eb->setNext(ec);
    ec->setNext(ed);
    ed->setNext(ea);
    // verify these edges aren't already in the mesh 
    // (which would be a bug, or a non-manifold mesh)
    assert (edges.find(std::make_pair(a,b)) == edges.end());
    assert (edges.find(std::make_pair(b,c)) == edges.end());
    assert (edges.find(std::make_pair(c,d)) == edges.end());
    assert (edges.find(std::make_pair(d,a)) == edges.end());
    // add the edges to the master list
    edges[std::make_pair(a,b)] = ea;
    edges[std::make_pair(b,c)] = eb;
    edges[std::make_pair(c,d)] = ec;
    edges[std::make_pair(d,a)] = ed;
    // connect up with opposite edges (if they exist)
    edgeshashtype::iterator ea_op = edges.find(std::make_pair(b,a)); 
    edgeshashtype::iterator eb_op = edges.find(std::make_pair(c,b)); 
    edgeshashtype::iterator ec_op = edges.find(std::make_pair(d,c)); 
    edgeshashtype::iterator ed_op = edges.find(std::make_pair(a,d)); 
    if (ea_op != edges.end()) { ea_op->second->setOpposite(ea); }
    if (eb_op != edges.end()) { eb_op->second->setOpposite(eb); }
    if (ec_op != edges.end()) { ec_op->second->setOpposite(ec); }
    if (ed_op != edges.end()) { ed_op->second->setOpposite(ed); }
    // add the face to the appropriate master list
    if (face_type == FACE_TYPE_ORIGINAL) {
        original_quads.push_back(f);
        subdivided_quads.push_back(f);
    } else if (face_type == FACE_TYPE_RASTERIZED) {
        rasterized_primitive_faces.push_back(f); 
    } else {
        assert (face_type == FACE_TYPE_SUBDIVIDED);
        subdivided_quads.push_back(f);
    }
    // if it's a light, add it to that list too
    if ((material->getEmittedColor()).Length() > 0 && face_type == FACE_TYPE_ORIGINAL) {
        original_lights.push_back(f);
    }
    return f;
}

void Mesh::removeFaceEdges(Face *f) {
    // helper function for face deletion
    Edge *ea = f->getEdge();
    Edge *eb = ea->getNext();
    Edge *ec = eb->getNext();
    Edge *ed = ec->getNext();
    assert (ed->getNext() == ea);
    Vertex *a = ea->getStartVertex();
    Vertex *b = eb->getStartVertex();
    Vertex *c = ec->getStartVertex();
    Vertex *d = ed->getStartVertex();
    // remove elements from master lists
    edges.erase(std::make_pair(a,b)); 
    edges.erase(std::make_pair(b,c)); 
    edges.erase(std::make_pair(c,d)); 
    edges.erase(std::make_pair(d,a)); 
    // clean up memory
    delete ea;
    delete eb;
    delete ec;
    delete ed;
}

// ==============================================================================
// EDGE HELPER FUNCTIONS

Edge* Mesh::getEdge(Vertex *a, Vertex *b) const {
    edgeshashtype::const_iterator iter = edges.find(std::make_pair(a,b));
    if (iter == edges.end()) return NULL;
    return iter->second;
}

Vertex* Mesh::getChildVertex(Vertex *p1, Vertex *p2) const {
    vphashtype::const_iterator iter = vertex_parents.find(std::make_pair(p1,p2)); 
    if (iter == vertex_parents.end()) return NULL;
    return iter->second; 
}

void Mesh::setParentsChild(Vertex *p1, Vertex *p2, Vertex *child) {
    assert (vertex_parents.find(std::make_pair(p1,p2)) == vertex_parents.end());
    vertex_parents[std::make_pair(p1,p2)] = child; 
}

//
// ===============================================================================
// the load function parses our (non-standard) extension of very simple .obj files
// ===============================================================================

/// Parses a word (possibly with leading whitespace) and returns true,
/// or else consumes the whole input and returns false.
bool parseWord(const char* &input, std::string& word) {
    // consume all white space
    while (*input && isspace(*input)) {
        ++input;
    }
    // reached EOF
    if (!*input) {
        return false;
    }
    word = std::string();
    while (*input && !isspace(*input)) {
        word.push_back(*input);
        ++input;
    }
    return true;
}

bool parseFloat(const char* &input, float& num) {
    // consume all white space
    while (*input && isspace(*input)) {
        ++input;
    }
    if (!*input) {
        std::cerr << "ERROR: expected <float>, found EOF\n";
        return false;
    }
    // consume all of the (hopefully digits) non-space characters
    std::string word;
    while (*input && !isspace(*input)) {
        word.push_back(*input);
        ++input;
    }
    char* endptr;
    num = strtof(word.c_str(), &endptr);
    if (*endptr != 0) {
        std::cerr << "ERROR: expected <float>, but '" << word << "' contains the non-digit '" << *endptr << "'\n";
        return false;
    }
    return true;
}
bool parseVertexSpec(const char* &input, int& vertIdx, int& textIdx, int& normIdx) {
    // consume all white space
    while (*input && isspace(*input)) {
        ++input;
    }
    // oops, we consumed everything
    if (*input == 0) {
        std::cerr << "ERROR: expected <int>, but found EOF\n";
        return false;
    }
    char* endptr;

    // consume the first number
    vertIdx = strtol(input, &endptr, 10);
    if (input == endptr) {
        std::cerr << "ERROR: expected at least one integer, found '" << endptr << "'\n";
        return false;
    }
    input = endptr;

    // set other indices to defaults
    textIdx = 0;
    normIdx = 0;

    if (*endptr == 0 || isspace(*endptr)) {
        return true; // return early, no more indices specified
    }
    if (*endptr != '/') {
        std::cerr << "ERROR: expected integer(s) separated by '\\', found '" << *endptr << "'\n";
        return false;
    }
    // consume the slash
    input += 1;

    // consume the next number
    textIdx = strtol(input, &endptr, 10);
    input = endptr;

    if (*endptr == 0 || isspace(*endptr)) {
        return true; // return early
    }
    if (*endptr != '/') {
        std::cerr << "ERROR: expected integer(s) separated by '\\', found '" << *endptr << "'\n";
        return false;
    }
    // consume the slash
    input += 1;

    normIdx = strtol(input, &endptr, 10);
    input = endptr;
    return true;
}

void Mesh::Load(std::ifstream& file, Material* mat, const Matrix& transform) {
    std::string line_buf;

    bbox = new BoundingBox();
    Material *active_material = mat;
    int base = numVertices(); // the number of vertices to start out is our base offset to work with
    int line_num = 0;

    while (std::getline(file, line_buf)) {
        const char* line = line_buf.c_str();
        line_num += 1;

        // eat all leading spaces
        while (*line && isspace(*line)) {
            line++;
        }
        // if we reached the end of the line, skip it
        if (!*line) {
            continue;
        }

        std::string token;
        parseWord(line, token);

        if (token == "#") {
            // ignore this line
            continue;
        }
        if (token == "}") {
            // done parsing
            break;
        }

        if (token == "material") {
            // load a material file
            parseWord(line, token);
            active_material = LoadMaterial(token);
        } else if (token == "v") {
            float x,y,z;
            if (!parseFloat(line, x)) {
                std::cerr << "   note: in line " << line_num << ", '" << line_buf << "'\n";
                std::cerr << "   expected 'v <float> <float> <float>'\n";
                std::cerr << "               ^^^^^^^ error\n";
                continue;
            }
            if (!parseFloat(line, y)) {
                std::cerr << "   note: in line " << line_num << ", '" << line_buf << "'\n";
                std::cerr << "   expected 'v <float> <float> <float>'\n";
                std::cerr << "                       ^^^^^^^ error\n";
                continue;
            }
            if (!parseFloat(line, z)) {
                std::cerr << "   note: in line " << line_num << ", '" << line_buf << "'\n";
                std::cerr << "   expected 'v <float> <float> <float>'\n";
                std::cerr << "                               ^^^^^^^ error\n";
                continue;
            }
            Vec3f pos(x,y,z);
            transform.Transform(pos);
            addVertex(pos);
        } else if (token == "vt") {
            assert (numVertices() >= 1);
            float s,t;
            if (!parseFloat(line, s)) {
                std::cerr << "   note: in line " << line_num << ", '" << line_buf << "'\n";
                std::cerr << "   expected 'vt <float> <float>'\n";
                std::cerr << "                ^^^^^^^ error\n";
                continue;
            }
            if (!parseFloat(line, t)) {
                std::cerr << "   note: in line " << line_num << ", '" << line_buf << "'\n";
                std::cerr << "   expected 'vt <float> <float>'\n";
                std::cerr << "                       ^^^^^^^ error\n";
                continue;
            }

            getVertex(numVertices()-1)->setTextureCoordinates(s,t);
        } else if (token == "vn") {
            // specifying the normal
            float x,y,z;
            if (!parseFloat(line, x)) {
                std::cerr << "   note: in line " << line_num << ", '" << line_buf << "'\n";
                std::cerr << "   expected 'vn <float> <float> <float>'\n";
                std::cerr << "                ^^^^^^^ error\n";
                continue;
            }
            if (!parseFloat(line, y)) {
                std::cerr << "   note: in line " << line_num << ", '" << line_buf << "'\n";
                std::cerr << "   expected 'vn <float> <float> <float>'\n";
                std::cerr << "                        ^^^^^^^ error\n";
                continue;
            }
            if (!parseFloat(line, z)) {
                std::cerr << "   note: in line " << line_num << ", '" << line_buf << "'\n";
                std::cerr << "   expected 'vn <float> <float> <float>'\n";
                std::cerr << "                                ^^^^^^^ error\n";
                continue;
            }
            Vec3f n = Vec3f(x,y,z);
            transform.TransformDirection(n);
            // TODO: make this work with relative & absolute indexing
            // getVertex(numVertices()-1)->setNormal(Vec3f(x,y,z))
        } else if (token == "f") {
            // we define point, (and optionally, texture and normal) indices for each vertex

            int vert[4]; // array of vertex indices
            int text[4]; // the texture indices
            int norm[4]; // the normal vector indices

            bool bad = false;
            int i;
            for (i = 0; i < 4; i++) {
                // eat leading whitespace
                while (*line && isspace(*line)) {
                    line += 1;
                }
                if (*line == 0) {
                    // reached EOF, stop.
                    break;
                }
                if (!parseVertexSpec(line, vert[i], text[i], norm[i])) {
                    std::cerr << "   note: in line " << line_num << ", '" << line_buf << "'\n";
                    std::cerr << "   expected 'f <vert> <vert> <vert> <vert>'\n";
                    std::cerr << "   in the " << i << " position\n";
                    bad = true;
                }
            }
            if (bad) {
                continue;
            }
            if (i < 3) {
                std::cerr << "Not enough gabagool vertices, eh?\n";
                continue;
            }

            bad = false;

            int vert_a = vert[0] - 1 + base;
            if (vert_a < base || numVertices() <= vert_a) {
                std::cerr << "ERROR: The vertex 'a' has index " << vert_a << ", but this is not allowed\n";
                std::cerr << "   note: in line " << line_num << ", '" << line_buf << "'\n";
                std::cerr << "   note: the base is " << base << " and there are " << numVertices() << " so far.\n";
                bad = true;
            }
            int vert_b = vert[1] - 1 + base;
            if (vert_b < base || numVertices() <= vert_b) {
                std::cerr << "ERROR: The vertex 'b' has index " << vert_b << ", but this is not allowed\n";
                std::cerr << "   note: in line " << line_num << ", '" << line_buf << "'\n";
                std::cerr << "   note: the base is " << base << " and there are " << numVertices() << " so far.\n";
                bad = true;
            }
            int vert_c = vert[2] - 1 + base;
            if (vert_c < base || numVertices() <= vert_c) {
                std::cerr << "ERROR: The vertex 'c' has index " << vert_c << ", but this is not allowed\n";
                std::cerr << "   note: in line " << line_num << ", '" << line_buf << "'\n";
                std::cerr << "   note: the base is " << base << " and there are " << numVertices() << " so far.\n";
                bad = true;
            }
            int vert_d;
            if (i == 3) {
                Vec3f pos = 0.5 * getVertex(vert_a)->get() + 0.5 * getVertex(vert_c)->get();
                vert_d = numVertices();
                addVertex(pos);
            } else {
                vert_d = vert[3] - 1 + base;
            }
            if (vert_d < base || numVertices() <= vert_d) {
                std::cerr << "ERROR: The vertex 'd' has index " << vert_d << ", but this is not allowed\n";
                std::cerr << "   note: in line " << line_num << ", '" << line_buf << "'\n";
                std::cerr << "   note: the base is " << base << " and there are " << numVertices() << " so far.\n";
                bad = true;
            }
            if (bad) {
                continue;
            }

            
            assert (active_material != NULL);
            addOriginalQuad(getVertex(vert_a),
                            getVertex(vert_b),
                            getVertex(vert_c),
                            getVertex(vert_d),
                            active_material);
        } else if (token == "sphere") {
            float x,y,z,r;
            assert( parseFloat(line, x) );
            assert( parseFloat(line, y) );
            assert( parseFloat(line, z) );
            assert( parseFloat(line, r) );
            assert (active_material != NULL);
            Vec3f pos(x,y,z);
            transform.Transform(pos);
            addPrimitive(new Sphere(pos,r,active_material));
        } else if (token == "ring") {
            float x,y,z,h,r,r2;
            assert( parseFloat(line, x) );
            assert( parseFloat(line, y) );
            assert( parseFloat(line, z) );
            assert( parseFloat(line, h) );
            assert( parseFloat(line, r) );
            assert( parseFloat(line, r2) );
            assert (active_material != NULL);
            Vec3f pos(x,y,z);
            transform.Transform(pos);
            addPrimitive(new CylinderRing(pos,h,r,r2,active_material));
        } else if (token == "box") {
            float minx, miny, minz, maxx, maxy, maxz;
            assert( parseFloat(line, minx) );
            assert( parseFloat(line, miny) );
            assert( parseFloat(line, minz) );
            assert( parseFloat(line, maxx) );
            assert( parseFloat(line, maxy) );
            assert( parseFloat(line, maxz) );
            assert (active_material != NULL);
            Vec3f min(minx,miny,minz);
            Vec3f max(maxx,maxy,maxz);
            transform.Transform(min);
            transform.Transform(max);
            addPrimitive(new BoxPrimitive(min, max, active_material));
        }
        else {
            std::cerr << "Unknown token '" << token << "' on line " << line_num << " while parsing scene file, ignoring...\n";
        }
    }
    std::cout << "Mesh loaded: " << numFaces() << " faces and " << numEdges() << " edges." << std::endl;
}

Material* Mesh::LoadMaterial(const std::string& filename) {
    std::string fullpath = args->path + '/' + filename;
    std::ifstream file(fullpath.c_str());
    if (!file.good()) {
        std::cerr << "Error loading object: can not find '" << filename << "' in path '" << args->path << "'" << std::endl;
        exit(1);
    }
    // avoid duplicate materials
    for (Material* mat : materials) {
        if (mat->getFileName() == filename) {
             return mat;
        }
    }
    Material *mat = new Material(args->path, file);
    materials.push_back(mat);
    return mat;
}
// =================================================================
// SUBDIVISION
// =================================================================

Vertex* Mesh::AddEdgeVertex(Vertex *a, Vertex *b) {
    Vertex *v = getChildVertex(a,b);
    if (v != NULL) return v;
    Vec3f pos = 0.5f*a->get() + 0.5f*b->get();
    float s = 0.5f*a->get_s() + 0.5f*b->get_s();
    float t = 0.5f*a->get_t() + 0.5f*b->get_t();
    v = addVertex(pos);
    v->setTextureCoordinates(s,t);
    setParentsChild(a,b,v);
    return v;
}

Vertex* Mesh::AddMidVertex(Vertex *a, Vertex *b, Vertex *c, Vertex *d) {
    Vec3f pos = 0.25f*a->get() + 0.25f*b->get() + 0.25f*c->get() + 0.25f*d->get();
    float s = 0.25f*a->get_s() + 0.25f*b->get_s() + 0.25f*c->get_s() + 0.25f*d->get_s();
    float t = 0.25f*a->get_t() + 0.25f*b->get_t() + 0.25f*c->get_t() + 0.25f*d->get_t();
    Vertex *v = addVertex(pos);
    v->setTextureCoordinates(s,t);
    return v;
}

void Mesh::Subdivision() {

    bool first_subdivision = false;
    if (original_quads.size() == subdivided_quads.size()) {
        first_subdivision = true;
    }

    std::vector<Face*> tmp = subdivided_quads;
    subdivided_quads.clear();

    for (unsigned int i = 0; i < tmp.size(); i++) {
        Face *f = tmp[i];

        Vertex *a = (*f)[0];
        Vertex *b = (*f)[1];
        Vertex *c = (*f)[2];
        Vertex *d = (*f)[3];
        // add new vertices on the edges
        Vertex *ab = AddEdgeVertex(a,b);
        Vertex *bc = AddEdgeVertex(b,c);
        Vertex *cd = AddEdgeVertex(c,d);
        Vertex *da = AddEdgeVertex(d,a);
        // add new point in the middle of the patch
        Vertex *mid = AddMidVertex(a,b,c,d);

        assert (getEdge(a,b) != NULL);
        assert (getEdge(b,c) != NULL);
        assert (getEdge(c,d) != NULL);
        assert (getEdge(d,a) != NULL);

        // copy the color and emission from the old patch to the new
        Material *material = f->getMaterial();
        if (!first_subdivision) {
            removeFaceEdges(f);
            delete f;
        }

        // create the new faces
        addSubdividedQuad(a,ab,mid,da,material);
        addSubdividedQuad(b,bc,mid,ab,material);
        addSubdividedQuad(c,cd,mid,bc,material);
        addSubdividedQuad(d,da,mid,cd,material);

        assert (getEdge(a,ab) != NULL);
        assert (getEdge(ab,b) != NULL);
        assert (getEdge(b,bc) != NULL);
        assert (getEdge(bc,c) != NULL);
        assert (getEdge(c,cd) != NULL);
        assert (getEdge(cd,d) != NULL);
        assert (getEdge(d,da) != NULL);
        assert (getEdge(da,a) != NULL);
    }
}

// ==================================================================

int Mesh::triCount() const { return 12 * numFaces(); } 

void Mesh::packMesh(float*& current) {

    RENDER_MODE render_mode = GLOBAL_args->mesh_data->render_mode;
    auto color_of_face = [render_mode](Face* f, int _i, int vertex_index) {
        // until we want to change the render modes, just render the diffuse color
        Vec3f col = f->getMaterial()->getDiffuseColor();
        return Vec3f(
            linear_to_srgb(col.r()),
            linear_to_srgb(col.b()),
            linear_to_srgb(col.g())
        );
    };

    for (int i = 0; i < numFaces(); i++) {
        Face *f = getFace(i);
        Vec3f normal = f->computeNormal();

        //double avg_s = 0;
        //double avg_t = 0;

        // wireframe is normally black, except when it's the special
        // patch, then the wireframe is red
        Vec3f wireframe_color(0,0,0);

        // 4 corner vertices
        Vec3f a_pos = ((*f)[0])->get();
        Vec3f b_pos = ((*f)[1])->get();
        Vec3f c_pos = ((*f)[2])->get();
        Vec3f d_pos = ((*f)[3])->get();

        // and their colors
        Vec3f a_color = color_of_face(f, i, 0);
        Vec3f b_color = color_of_face(f, i, 1);
        Vec3f c_color = color_of_face(f, i, 2);
        Vec3f d_color = color_of_face(f, i, 3);

        Vec3f avg_color = 0.25f * (a_color+b_color+c_color+d_color);

        // the centroid (for wireframe rendering)
        Vec3f centroid = f->computeCentroid();

        AddWireFrameTriangle(current,
                             a_pos,b_pos,centroid,
                             normal,normal,normal,
                             wireframe_color,
                             a_color,b_color,avg_color);
        AddWireFrameTriangle(current,
                             b_pos,c_pos,centroid,
                             normal,normal,normal,
                             wireframe_color,
                             b_color,c_color,avg_color);
        AddWireFrameTriangle(current,
                             c_pos,d_pos,centroid,
                             normal,normal,normal,
                             wireframe_color,
                             c_color,d_color,avg_color);
        AddWireFrameTriangle(current,
                             d_pos,a_pos,centroid,
                             normal,normal,normal,
                             wireframe_color,
                             d_color,a_color,avg_color);

    }
}


