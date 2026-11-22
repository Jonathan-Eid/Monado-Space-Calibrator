// SPDX-License-Identifier: MIT
// Portable math coverage for the Linux OpenGL scene helpers (no GL context).
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>
#include "calibration_scene.h"
#include <cmath>
#include <cassert>
#include <iostream>

using namespace calibration;
using namespace scene;

static bool close(double a,double b,double eps=1e-9){return std::abs(a-b)<eps;}
static bool closeVec(const Eigen::Vector3d& a,const Eigen::Vector3d& b){return (a-b).norm()<1e-9;}

int main(){
    static_assert(sizeof(Vertex)==36,".vdm vertex layout must stay 36 bytes (float3 pos, float4 color, float2 uv)");
    Pose at_origin=Pose::Identity();
    Vertex v=vertex({1,2,3},White,.25f,.75f);
    assert(close(v.p[0],1)&&close(v.p[1],2)&&close(v.p[2],3));
    assert(close(v.c[0],White.x)&&close(v.uv[0],.25f)&&close(v.uv[1],.75f));
    Pose offset=Pose::Identity();offset.translation()=Eigen::Vector3d(1,2,3);
    assert(closeVec(point(offset,0,0,0),Eigen::Vector3d(1,2,3)));
    assert(closeVec(point(offset,1,0,0),Eigen::Vector3d(2,2,3)));
    Eigen::Matrix4f m=matrix(offset);
    assert(close(m(0,3),1)&&close(m(1,3),2)&&close(m(2,3),3)&&close(m(3,3),1));
    std::vector<Vertex> lines;
    axes(lines,at_origin,.1);
    // 3 axes, 2 vertices each.
    assert(lines.size()==6);
    std::vector<Vertex> tris;
    tracker(tris,at_origin);
    // Ellipsoids are triangle lists: vertex count divisible by 3.
    assert(!tris.empty()&&tris.size()%3==0);
    auto box=panel(at_origin,2,1);
    // Panel: 6 vertices covering a 2x1 quad.
    assert(box.size()==6);
    // Model-space vertices must be finite and near the origin.
    for(const auto& vertex:box){assert(std::isfinite(vertex.p[0])&&std::isfinite(vertex.p[1])&&std::isfinite(vertex.p[2]));}
    std::cout<<"renderer-math: all checks passed\n";
    return 0;
}
