#include <iostream>
#include <cstring>
#include "utils.h"

// ============================================================================================
// ============================================================================================
// Helper function that adds 3 triangles to render wireframe

void AddWireFrameTriangle(float* &current,
                          const Vec3f &apos, const Vec3f &bpos, const Vec3f &cpos,
                          const Vec3f &anormal, const Vec3f &bnormal, const Vec3f &cnormal,
                          const Vec3f &edge_color,
                          const Vec3f &acolor_, const Vec3f &bcolor_, const Vec3f &ccolor) {

  // will be edited if wireframe is visible
  Vec3f acolor=acolor_;
  Vec3f bcolor=bcolor_;
  
  /*     a-------------b
   *      \           /
   *       x---------y
   *        \       /
   *         \     /
   *          \   /
   *           \ /
   *            c     
   */

  // interpolate the positions, colors, and normals
  float frac = 0.05;
  Vec3f xpos = (1-frac)*apos + frac*cpos;
  Vec3f ypos = (1-frac)*bpos + frac*cpos;
  Vec3f xcolor = (1-frac)*acolor + frac*ccolor;
  Vec3f ycolor = (1-frac)*bcolor + frac*ccolor;
  Vec3f xnormal = (1-frac)*anormal + frac*cnormal; xnormal.Normalize();
  Vec3f ynormal = (1-frac)*bnormal + frac*cnormal; ynormal.Normalize();

  float12 ta;
  float12 tb;
  float12 tc;

  // the main triangle
  ta = { float(xpos.x()),float(xpos.y()),float(xpos.z()),1, float(xnormal.x()),float(xnormal.y()),float(xnormal.z()),0, float(xcolor.r()),float(xcolor.g()),float(xcolor.b()),1 };
  tb = { float(ypos.x()),float(ypos.y()),float(ypos.z()),1, float(ynormal.x()),float(ynormal.y()),float(ynormal.z()),0, float(ycolor.r()),float(ycolor.g()),float(ycolor.b()),1 };
  tc = { float(cpos.x()),float(cpos.y()),float(cpos.z()),1, float(cnormal.x()),float(cnormal.y()),float(cnormal.z()),0, float(ccolor.r()),float(ccolor.g()),float(ccolor.b()),1 };
  memcpy(current, &ta, sizeof(float)*12); current += 12; 
  memcpy(current, &tb, sizeof(float)*12); current += 12; 
  memcpy(current, &tc, sizeof(float)*12); current += 12;

  // the two triangles that make the wireframe
  if (!GLOBAL_args->mesh_data->wireframe) {
    acolor = edge_color;
    bcolor = edge_color;
    xcolor = edge_color;
    ycolor = edge_color;    
  }
  ta = { float(apos.x()),float(apos.y()),float(apos.z()),1, float(anormal.x()),float(anormal.y()),float(anormal.z()),0, float(acolor.r()),float(acolor.g()),float(acolor.b()),1 };
  tb = { float(bpos.x()),float(bpos.y()),float(bpos.z()),1, float(bnormal.x()),float(bnormal.y()),float(bnormal.z()),0, float(bcolor.r()),float(bcolor.g()),float(bcolor.b()),1 };
  tc = { float(xpos.x()),float(xpos.y()),float(xpos.z()),1, float(xnormal.x()),float(xnormal.y()),float(xnormal.z()),0, float(xcolor.r()),float(xcolor.g()),float(xcolor.b()),1 };
  memcpy(current, &ta, sizeof(float)*12); current += 12; 
  memcpy(current, &tb, sizeof(float)*12); current += 12; 
  memcpy(current, &tc, sizeof(float)*12); current += 12;
  ta = { float(xpos.x()),float(xpos.y()),float(xpos.z()),1, float(xnormal.x()),float(xnormal.y()),float(xnormal.z()),0, float(xcolor.r()),float(xcolor.g()),float(xcolor.b()),1 };
  tb = { float(bpos.x()),float(bpos.y()),float(bpos.z()),1, float(bnormal.x()),float(bnormal.y()),float(bnormal.z()),0, float(bcolor.r()),float(bcolor.g()),float(bcolor.b()),1 };
  tc = { float(ypos.x()),float(ypos.y()),float(ypos.z()),1, float(ynormal.x()),float(ynormal.y()),float(ynormal.z()),0, float(ycolor.r()),float(ycolor.g()),float(ycolor.b()),1 };
  memcpy(current, &ta, sizeof(float)*12); current += 12; 
  memcpy(current, &tb, sizeof(float)*12); current += 12; 
  memcpy(current, &tc, sizeof(float)*12); current += 12;
}

// ============================================================================================
// ============================================================================================
// Adds 2 triangles to make a simple quad (no wireframe)

void AddQuad(float* &current,
             const Vec3f &apos, const Vec3f &bpos, const Vec3f &cpos, const Vec3f &dpos,
             const Vec3f &normal,
             const Vec3f &color) {

  float12 ta;
  float12 tb;
  float12 tc;

  ta = { float(apos.x()),float(apos.y()),float(apos.z()),1, float(normal.x()),float(normal.y()),float(normal.z()),0, float(color.r()),float(color.g()),float(color.b()),1 };
  tb = { float(bpos.x()),float(bpos.y()),float(bpos.z()),1, float(normal.x()),float(normal.y()),float(normal.z()),0, float(color.r()),float(color.g()),float(color.b()),1 };
  tc = { float(cpos.x()),float(cpos.y()),float(cpos.z()),1, float(normal.x()),float(normal.y()),float(normal.z()),0, float(color.r()),float(color.g()),float(color.b()),1 };
  memcpy(current, &ta, sizeof(float)*12); current += 12; 
  memcpy(current, &tb, sizeof(float)*12); current += 12; 
  memcpy(current, &tc, sizeof(float)*12); current += 12;

  ta = { float(apos.x()),float(apos.y()),float(apos.z()),1, float(normal.x()),float(normal.y()),float(normal.z()),0, float(color.r()),float(color.g()),float(color.b()),1 };
  tb = { float(cpos.x()),float(cpos.y()),float(cpos.z()),1, float(normal.x()),float(normal.y()),float(normal.z()),0, float(color.r()),float(color.g()),float(color.b()),1 };
  tc = { float(dpos.x()),float(dpos.y()),float(dpos.z()),1, float(normal.x()),float(normal.y()),float(normal.z()),0, float(color.r()),float(color.g()),float(color.b()),1 };
  memcpy(current, &ta, sizeof(float)*12); current += 12; 
  memcpy(current, &tb, sizeof(float)*12); current += 12; 
  memcpy(current, &tc, sizeof(float)*12); current += 12;
}

// ============================================================================================
// ============================================================================================
// Adds 12 triangles to make a simple rectangular box

void AddBox(float* &current,
            const Vec3f pos[8],
            const Vec3f &color) {

  Vec3f normal1 = ComputeNormal(pos[0],pos[1],pos[2]);
  normal1 = Vec3f(0,0,0);
  AddQuad(current,pos[0],pos[1],pos[3],pos[2],normal1,color);
  AddQuad(current,pos[4],pos[6],pos[7],pos[5],-normal1,color);

  Vec3f normal2 = ComputeNormal(pos[0],pos[4],pos[5]);
  normal2 = Vec3f(0,0,0);
  AddQuad(current,pos[0],pos[4],pos[5],pos[1],normal2,color);
  AddQuad(current,pos[2],pos[3],pos[7],pos[6],-normal2,color);

  Vec3f normal3 = ComputeNormal(pos[0],pos[2],pos[6]);
  normal3 = Vec3f(0,0,0);
  AddQuad(current,pos[0],pos[2],pos[6],pos[4],normal3,color);
  AddQuad(current,pos[1],pos[5],pos[7],pos[3],normal3,color);
}

// since glLineWidth is gone...  
// instead we'll draw a rectangular box with 6 sides, 6 triangles each
// (should probably use a geometry shader instead)
void addEdgeGeometry(float* &current,
                     const Vec3f &a, const Vec3f &b, 
                     const Vec3f &acolor, const Vec3f &bcolor, 
                     float a_th,float b_th) {

  
  // find perpendicular axes
  Vec3f dir = (b-a);
  Vec3f one;
  Vec3f two;
  if (std::min(a_th,b_th) < 0.0000001 ||
      dir.Length() < 0.01*std::min(a_th,b_th)) {
    dir = one = two = Vec3f(0,0,0);
  } else {
    dir.Normalize(); 
    Vec3f tmp; Vec3f::Cross3(tmp,dir,Vec3f(1,0,0));
    if (tmp.Length() < 0.1) {
      Vec3f::Cross3(tmp,dir,Vec3f(0,0,1));
    }
    tmp.Normalize();
    Vec3f::Cross3(one,dir,tmp);
    assert (fabs(one.Length()-1.0) < 0.001);
    Vec3f::Cross3(two,dir,one);
    assert (fabs(two.Length()-1.0) < 0.001);
  }
  
  Vec3f a1 = a-one*a_th-two*a_th;
  Vec3f a2 = a-one*a_th+two*a_th;
  Vec3f a3 = a+one*a_th+two*a_th;
  Vec3f a4 = a+one*a_th-two*a_th;

  Vec3f b1 = b-one*b_th-two*b_th;
  Vec3f b2 = b-one*b_th+two*b_th;
  Vec3f b3 = b+one*b_th+two*b_th;
  Vec3f b4 = b+one*b_th-two*b_th;
  
  float12 ta,tb,tc;

  // draw 6 sides of the box  
  ta = { float(a1.x()),float(a1.y()),float(a1.z()),1, float(-one.x()),float(-one.y()),float(-one.z()),0, float(acolor.r()),float(acolor.g()),float(acolor.b()),1 };
  tb = { float(b2.x()),float(b2.y()),float(b2.z()),1, float(-one.x()),float(-one.y()),float(-one.z()),0, float(bcolor.r()),float(bcolor.g()),float(bcolor.b()),1 };
  tc = { float(a2.x()),float(a2.y()),float(a2.z()),1, float(-one.x()),float(-one.y()),float(-one.z()),0, float(acolor.r()),float(acolor.g()),float(acolor.b()),1 };
  memcpy(current, &ta, sizeof(float)*12); current += 12; 
  memcpy(current, &tb, sizeof(float)*12); current += 12; 
  memcpy(current, &tc, sizeof(float)*12); current += 12;
  ta = { float(a1.x()),float(a1.y()),float(a1.z()),1, float(-one.x()),float(-one.y()),float(-one.z()),0, float(acolor.r()),float(acolor.g()),float(acolor.b()),1 };
  tb = { float(b1.x()),float(b1.y()),float(b1.z()),1, float(-one.x()),float(-one.y()),float(-one.z()),0, float(bcolor.r()),float(bcolor.g()),float(bcolor.b()),1 };
  tc = { float(b2.x()),float(b2.y()),float(b2.z()),1, float(-one.x()),float(-one.y()),float(-one.z()),0, float(bcolor.r()),float(bcolor.g()),float(bcolor.b()),1 };
  memcpy(current, &ta, sizeof(float)*12); current += 12; 
  memcpy(current, &tb, sizeof(float)*12); current += 12; 
  memcpy(current, &tc, sizeof(float)*12); current += 12;

  ta = { float(a2.x()),float(a2.y()),float(a2.z()),1, float(two.x()),float(two.y()),float(two.z()),0, float(acolor.r()),float(acolor.g()),float(acolor.b()),1 };
  tb = { float(b3.x()),float(b3.y()),float(b3.z()),1, float(two.x()),float(two.y()),float(two.z()),0, float(bcolor.r()),float(bcolor.g()),float(bcolor.b()),1 };
  tc = { float(a3.x()),float(a3.y()),float(a3.z()),1, float(two.x()),float(two.y()),float(two.z()),0, float(acolor.r()),float(acolor.g()),float(acolor.b()),1 };
  memcpy(current, &ta, sizeof(float)*12); current += 12; 
  memcpy(current, &tb, sizeof(float)*12); current += 12; 
  memcpy(current, &tc, sizeof(float)*12); current += 12;
  ta = { float(a2.x()),float(a2.y()),float(a2.z()),1, float(two.x()),float(two.y()),float(two.z()),0, float(acolor.r()),float(acolor.g()),float(acolor.b()),1 };
  tb = { float(b2.x()),float(b2.y()),float(b2.z()),1, float(two.x()),float(two.y()),float(two.z()),0, float(bcolor.r()),float(bcolor.g()),float(bcolor.b()),1 };
  tc = { float(b3.x()),float(b3.y()),float(b3.z()),1, float(two.x()),float(two.y()),float(two.z()),0, float(bcolor.r()),float(bcolor.g()),float(bcolor.b()),1 };
  memcpy(current, &ta, sizeof(float)*12); current += 12; 
  memcpy(current, &tb, sizeof(float)*12); current += 12; 
  memcpy(current, &tc, sizeof(float)*12); current += 12;

  ta = { float(a3.x()),float(a3.y()),float(a3.z()),1, float(one.x()),float(one.y()),float(one.z()),0, float(acolor.r()),float(acolor.g()),float(acolor.b()),1 };
  tb = { float(b4.x()),float(b4.y()),float(b4.z()),1, float(one.x()),float(one.y()),float(one.z()),0, float(bcolor.r()),float(bcolor.g()),float(bcolor.b()),1 };
  tc = { float(a4.x()),float(a4.y()),float(a4.z()),1, float(one.x()),float(one.y()),float(one.z()),0, float(acolor.r()),float(acolor.g()),float(acolor.b()),1 };
  memcpy(current, &ta, sizeof(float)*12); current += 12; 
  memcpy(current, &tb, sizeof(float)*12); current += 12; 
  memcpy(current, &tc, sizeof(float)*12); current += 12;
  ta = { float(a3.x()),float(a3.y()),float(a3.z()),1, float(one.x()),float(one.y()),float(one.z()),0, float(acolor.r()),float(acolor.g()),float(acolor.b()),1 };
  tb = { float(b3.x()),float(b3.y()),float(b3.z()),1, float(one.x()),float(one.y()),float(one.z()),0, float(bcolor.r()),float(bcolor.g()),float(bcolor.b()),1 };
  tc = { float(b4.x()),float(b4.y()),float(b4.z()),1, float(one.x()),float(one.y()),float(one.z()),0, float(bcolor.r()),float(bcolor.g()),float(bcolor.b()),1 };
  memcpy(current, &ta, sizeof(float)*12); current += 12; 
  memcpy(current, &tb, sizeof(float)*12); current += 12; 
  memcpy(current, &tc, sizeof(float)*12); current += 12;

  ta = { float(a4.x()),float(a4.y()),float(a4.z()),1, float(-two.x()),float(-two.y()),float(-two.z()),0, float(acolor.r()),float(acolor.g()),float(acolor.b()),1 };
  tb = { float(b1.x()),float(b1.y()),float(b1.z()),1, float(-two.x()),float(-two.y()),float(-two.z()),0, float(bcolor.r()),float(bcolor.g()),float(bcolor.b()),1 };
  tc = { float(a1.x()),float(a1.y()),float(a1.z()),1, float(-two.x()),float(-two.y()),float(-two.z()),0, float(acolor.r()),float(acolor.g()),float(acolor.b()),1 };
  memcpy(current, &ta, sizeof(float)*12); current += 12; 
  memcpy(current, &tb, sizeof(float)*12); current += 12; 
  memcpy(current, &tc, sizeof(float)*12); current += 12;
  ta = { float(a4.x()),float(a4.y()),float(a4.z()),1, float(-two.x()),float(-two.y()),float(-two.z()),0, float(acolor.r()),float(acolor.g()),float(acolor.b()),1 };
  tb = { float(b4.x()),float(b4.y()),float(b4.z()),1, float(-two.x()),float(-two.y()),float(-two.z()),0, float(bcolor.r()),float(bcolor.g()),float(bcolor.b()),1 };
  tc = { float(b1.x()),float(b1.y()),float(b1.z()),1, float(-two.x()),float(-two.y()),float(-two.z()),0, float(bcolor.r()),float(bcolor.g()),float(bcolor.b()),1 };
  memcpy(current, &ta, sizeof(float)*12); current += 12; 
  memcpy(current, &tb, sizeof(float)*12); current += 12; 
  memcpy(current, &tc, sizeof(float)*12); current += 12;
  
  //top
  ta = { float(a1.x()),float(a1.y()),float(a1.z()),1, float(dir.x()),float(dir.y()),float(dir.z()),0, float(acolor.r()),float(acolor.g()),float(acolor.b()),1 };
  tb = { float(a2.x()),float(a2.y()),float(a2.z()),1, float(dir.x()),float(dir.y()),float(dir.z()),0, float(acolor.r()),float(acolor.g()),float(acolor.b()),1 };
  tc = { float(a3.x()),float(a3.y()),float(a3.z()),1, float(dir.x()),float(dir.y()),float(dir.z()),0, float(acolor.r()),float(acolor.g()),float(acolor.b()),1 };
  memcpy(current, &ta, sizeof(float)*12); current += 12; 
  memcpy(current, &tb, sizeof(float)*12); current += 12; 
  memcpy(current, &tc, sizeof(float)*12); current += 12;
  ta = { float(a1.x()),float(a1.y()),float(a1.z()),1, float(dir.x()),float(dir.y()),float(dir.z()),0, float(acolor.r()),float(acolor.g()),float(acolor.b()),1 };
  tb = { float(a3.x()),float(a3.y()),float(a3.z()),1, float(dir.x()),float(dir.y()),float(dir.z()),0, float(acolor.r()),float(acolor.g()),float(acolor.b()),1 };
  tc = { float(a4.x()),float(a4.y()),float(a4.z()),1, float(dir.x()),float(dir.y()),float(dir.z()),0, float(acolor.r()),float(acolor.g()),float(acolor.b()),1 };
  memcpy(current, &ta, sizeof(float)*12); current += 12; 
  memcpy(current, &tb, sizeof(float)*12); current += 12; 
  memcpy(current, &tc, sizeof(float)*12); current += 12;

  //bottom
  ta = { float(b1.x()),float(b1.y()),float(b1.z()),1, float(-dir.x()),float(-dir.y()),float(-dir.z()),0, float(bcolor.r()),float(bcolor.g()),float(bcolor.b()),1 };
  tb = { float(b3.x()),float(b3.y()),float(b3.z()),1, float(-dir.x()),float(-dir.y()),float(-dir.z()),0, float(bcolor.r()),float(bcolor.g()),float(bcolor.b()),1 };
  tc = { float(b2.x()),float(b2.y()),float(b2.z()),1, float(-dir.x()),float(-dir.y()),float(-dir.z()),0, float(bcolor.r()),float(bcolor.g()),float(bcolor.b()),1 };
  memcpy(current, &ta, sizeof(float)*12); current += 12; 
  memcpy(current, &tb, sizeof(float)*12); current += 12; 
  memcpy(current, &tc, sizeof(float)*12); current += 12;
  ta = { float(b1.x()),float(b1.y()),float(b1.z()),1, float(-dir.x()),float(-dir.y()),float(-dir.z()),0, float(acolor.r()),float(bcolor.g()),float(bcolor.b()),1 };
  tb = { float(b4.x()),float(b4.y()),float(b4.z()),1, float(-dir.x()),float(-dir.y()),float(-dir.z()),0, float(acolor.r()),float(bcolor.g()),float(bcolor.b()),1 };
  tc = { float(b3.x()),float(b3.y()),float(b3.z()),1, float(-dir.x()),float(-dir.y()),float(-dir.z()),0, float(acolor.r()),float(bcolor.g()),float(bcolor.b()),1 };
  memcpy(current, &ta, sizeof(float)*12); current += 12; 
  memcpy(current, &tb, sizeof(float)*12); current += 12; 
  memcpy(current, &tc, sizeof(float)*12); current += 12;
}

void addCubeGeometry(float* &current,
                     Vec3f pt[8],
                     const Vec3f &color) {
  
  float12 ta,tb,tc;

  // left
  ta = { float(pt[0].x()),float(pt[0].y()),float(pt[0].z()),1,  -1,0,0,0,  float(color.r()),float(color.g()),float(color.b()),1 };
  tb = { float(pt[1].x()),float(pt[1].y()),float(pt[1].z()),1,  -1,0,0,0,  float(color.r()),float(color.g()),float(color.b()),1 };
  tc = { float(pt[2].x()),float(pt[2].y()),float(pt[2].z()),1,  -1,0,0,0,  float(color.r()),float(color.g()),float(color.b()),1 };
  memcpy(current, &ta, sizeof(float)*12); current += 12; 
  memcpy(current, &tb, sizeof(float)*12); current += 12; 
  memcpy(current, &tc, sizeof(float)*12); current += 12;
  ta = { float(pt[1].x()),float(pt[1].y()),float(pt[1].z()),1,  -1,0,0,0,  float(color.r()),float(color.g()),float(color.b()),1 };
  tb = { float(pt[3].x()),float(pt[3].y()),float(pt[3].z()),1,  -1,0,0,0,  float(color.r()),float(color.g()),float(color.b()),1 };
  tc = { float(pt[2].x()),float(pt[2].y()),float(pt[2].z()),1,  -1,0,0,0,  float(color.r()),float(color.g()),float(color.b()),1 };
  memcpy(current, &ta, sizeof(float)*12); current += 12; 
  memcpy(current, &tb, sizeof(float)*12); current += 12; 
  memcpy(current, &tc, sizeof(float)*12); current += 12;

  // right
  ta = { float(pt[4].x()),float(pt[4].y()),float(pt[4].z()),1,  0,1,0,0,  float(color.r()),float(color.g()),float(color.b()),1 };
  tb = { float(pt[6].x()),float(pt[6].y()),float(pt[6].z()),1,  0,1,0,0,  float(color.r()),float(color.g()),float(color.b()),1 };
  tc = { float(pt[5].x()),float(pt[5].y()),float(pt[5].z()),1,  0,1,0,0,  float(color.r()),float(color.g()),float(color.b()),1 };
  memcpy(current, &ta, sizeof(float)*12); current += 12; 
  memcpy(current, &tb, sizeof(float)*12); current += 12; 
  memcpy(current, &tc, sizeof(float)*12); current += 12;
  ta = { float(pt[5].x()),float(pt[5].y()),float(pt[5].z()),1,  0,1,0,0,  float(color.r()),float(color.g()),float(color.b()),1 };
  tb = { float(pt[6].x()),float(pt[6].y()),float(pt[6].z()),1,  0,1,0,0,  float(color.r()),float(color.g()),float(color.b()),1 };
  tc = { float(pt[7].x()),float(pt[7].y()),float(pt[7].z()),1,  0,1,0,0,  float(color.r()),float(color.g()),float(color.b()),1 };
  memcpy(current, &ta, sizeof(float)*12); current += 12; 
  memcpy(current, &tb, sizeof(float)*12); current += 12; 
  memcpy(current, &tc, sizeof(float)*12); current += 12;

  // bottom
  ta = { float(pt[0].x()),float(pt[0].y()),float(pt[0].z()),1,  0,-1,0,0,  float(color.r()),float(color.g()),float(color.b()),1 };
  tb = { float(pt[4].x()),float(pt[4].y()),float(pt[4].z()),1,  0,-1,0,0,  float(color.r()),float(color.g()),float(color.b()),1 };
  tc = { float(pt[1].x()),float(pt[1].y()),float(pt[1].z()),1,  0,-1,0,0,  float(color.r()),float(color.g()),float(color.b()),1 };
  memcpy(current, &ta, sizeof(float)*12); current += 12; 
  memcpy(current, &tb, sizeof(float)*12); current += 12; 
  memcpy(current, &tc, sizeof(float)*12); current += 12;
  ta = { float(pt[1].x()),float(pt[1].y()),float(pt[1].z()),1,  0,-1,0,0,  float(color.r()),float(color.g()),float(color.b()),1 };
  tb = { float(pt[4].x()),float(pt[4].y()),float(pt[4].z()),1,  0,-1,0,0,  float(color.r()),float(color.g()),float(color.b()),1 };
  tc = { float(pt[5].x()),float(pt[5].y()),float(pt[5].z()),1,  0,-1,0,0,  float(color.r()),float(color.g()),float(color.b()),1 };
  memcpy(current, &ta, sizeof(float)*12); current += 12; 
  memcpy(current, &tb, sizeof(float)*12); current += 12; 
  memcpy(current, &tc, sizeof(float)*12); current += 12;

  // top
  ta = { float(pt[2].x()),float(pt[2].y()),float(pt[2].z()),1,  0,1,0,0,  float(color.r()),float(color.g()),float(color.b()),1 };
  tb = { float(pt[3].x()),float(pt[3].y()),float(pt[3].z()),1,  0,1,0,0,  float(color.r()),float(color.g()),float(color.b()),1 };
  tc = { float(pt[6].x()),float(pt[6].y()),float(pt[6].z()),1,  0,1,0,0,  float(color.r()),float(color.g()),float(color.b()),1 };
  memcpy(current, &ta, sizeof(float)*12); current += 12; 
  memcpy(current, &tb, sizeof(float)*12); current += 12; 
  memcpy(current, &tc, sizeof(float)*12); current += 12;
  ta = { float(pt[3].x()),float(pt[3].y()),float(pt[3].z()),1,  0,1,0,0,  float(color.r()),float(color.g()),float(color.b()),1 };
  tb = { float(pt[7].x()),float(pt[7].y()),float(pt[7].z()),1,  0,1,0,0,  float(color.r()),float(color.g()),float(color.b()),1 };
  tc = { float(pt[6].x()),float(pt[6].y()),float(pt[6].z()),1,  0,1,0,0,  float(color.r()),float(color.g()),float(color.b()),1 };
  memcpy(current, &ta, sizeof(float)*12); current += 12; 
  memcpy(current, &tb, sizeof(float)*12); current += 12; 
  memcpy(current, &tc, sizeof(float)*12); current += 12;

  // front
  ta = { float(pt[1].x()),float(pt[1].y()),float(pt[1].z()),1,  0,0,1,0,  float(color.r()),float(color.g()),float(color.b()),1 };
  tb = { float(pt[5].x()),float(pt[5].y()),float(pt[5].z()),1,  0,0,1,0,  float(color.r()),float(color.g()),float(color.b()),1 };
  tc = { float(pt[3].x()),float(pt[3].y()),float(pt[3].z()),1,  0,0,1,0,  float(color.r()),float(color.g()),float(color.b()),1 };
  memcpy(current, &ta, sizeof(float)*12); current += 12; 
  memcpy(current, &tb, sizeof(float)*12); current += 12; 
  memcpy(current, &tc, sizeof(float)*12); current += 12;
  ta = { float(pt[5].x()),float(pt[5].y()),float(pt[5].z()),1,  0,0,1,0,  float(color.r()),float(color.g()),float(color.b()),1 };
  tb = { float(pt[7].x()),float(pt[7].y()),float(pt[7].z()),1,  0,0,1,0,  float(color.r()),float(color.g()),float(color.b()),1 };
  tc = { float(pt[3].x()),float(pt[3].y()),float(pt[3].z()),1,  0,0,1,0,  float(color.r()),float(color.g()),float(color.b()),1 };
  memcpy(current, &ta, sizeof(float)*12); current += 12; 
  memcpy(current, &tb, sizeof(float)*12); current += 12; 
  memcpy(current, &tc, sizeof(float)*12); current += 12;

  // back
  ta = { float(pt[0].x()),float(pt[0].y()),float(pt[0].z()),1,  0,0,-1,0,  float(color.r()),float(color.g()),float(color.b()),1 };
  tb = { float(pt[2].x()),float(pt[2].y()),float(pt[2].z()),1,  0,0,-1,0,  float(color.r()),float(color.g()),float(color.b()),1 };
  tc = { float(pt[4].x()),float(pt[4].y()),float(pt[4].z()),1,  0,0,-1,0,  float(color.r()),float(color.g()),float(color.b()),1 };
  memcpy(current, &ta, sizeof(float)*12); current += 12; 
  memcpy(current, &tb, sizeof(float)*12); current += 12; 
  memcpy(current, &tc, sizeof(float)*12); current += 12;
  ta = { float(pt[4].x()),float(pt[4].y()),float(pt[4].z()),1,  0,0,-1,0,  float(color.r()),float(color.g()),float(color.b()),1 };
  tb = { float(pt[2].x()),float(pt[2].y()),float(pt[2].z()),1,  0,0,-1,0,  float(color.r()),float(color.g()),float(color.b()),1 };
  tc = { float(pt[6].x()),float(pt[6].y()),float(pt[6].z()),1,  0,0,-1,0,  float(color.r()),float(color.g()),float(color.b()),1 };
  memcpy(current, &ta, sizeof(float)*12); current += 12; 
  memcpy(current, &tb, sizeof(float)*12); current += 12; 
  memcpy(current, &tc, sizeof(float)*12); current += 12;
}

// ============================================================================================
// ============================================================================================
