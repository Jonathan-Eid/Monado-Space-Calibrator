// SPDX-License-Identifier: MIT
#pragma once
#ifdef WIN32
#include <windows.h>
#include <d3d11.h>
#include <dxgi1_2.h>
#include <d3dcompiler.h>
#include <DirectXMath.h>
#include <wrl/client.h>
#include <wincodec.h>
#include <map>
#include <vector>
#include <stdexcept>
#include "calibration_math.h"

namespace scene {
using Microsoft::WRL::ComPtr;
using namespace DirectX;
using calibration::Pose;
struct Vertex { XMFLOAT3 p; XMFLOAT4 c; XMFLOAT2 uv{}; };
using Color = XMFLOAT4;
inline const Color White{.88f,.91f,.94f,1}, Dark{.07f,.11f,.14f,1}, Teal{.08f,.65f,.7f,1}, Orange{.95f,.43f,.12f,1};
inline void check(HRESULT result,const char* message) { if(FAILED(result)) throw std::runtime_error(message); }
inline XMMATRIX matrix(const Pose& pose) {
    const Eigen::Matrix4f m=pose.matrix().cast<float>().transpose();
    XMFLOAT4X4 result; std::memcpy(&result,m.data(),sizeof(result));
    // Eigen's column-major storage of the transpose is the row-vector matrix.
    XMFLOAT4X4 row;
    for(int i=0;i<4;++i) for(int j=0;j<4;++j) row.m[i][j]=float(pose.matrix()(j,i));
    return XMLoadFloat4x4(&row);
}
inline Eigen::Vector3d point(const Pose& pose,double x,double y,double z) { return pose*Eigen::Vector3d(x,y,z); }
inline Vertex vertex(const Eigen::Vector3d& p,Color color,float u=0,float v=0) { return {{float(p.x()),float(p.y()),float(p.z())},color,{u,v}}; }
inline void line(std::vector<Vertex>& out,const Eigen::Vector3d& a,const Eigen::Vector3d& b,Color color) { out.push_back(vertex(a,color)); out.push_back(vertex(b,color)); }
inline void axes(std::vector<Vertex>& out,const Pose& pose,double size=.1) {
    line(out,pose.translation(),point(pose,size,0,0),{.85f,.15f,.1f,1});
    line(out,pose.translation(),point(pose,0,size,0),{.1f,.65f,.2f,1});
    line(out,pose.translation(),point(pose,0,0,size),{.1f,.3f,.85f,1});
}
inline void ellipsoid(std::vector<Vertex>& out,const Pose& pose,Eigen::Vector3d center,Eigen::Vector3d radii,Color color) {
    constexpr int n=16,m=10;
    const auto p=[&](int a,int b) -> Eigen::Vector3d {
        const double u=a*6.283185307179586/n,v=b*3.141592653589793/m;
        Eigen::Vector3d q(std::sin(v)*std::cos(u),std::cos(v),std::sin(v)*std::sin(u));
        return pose*(center+q.cwiseProduct(radii));
    };
    for(int b=0;b<m;++b) for(int a=0;a<n;++a) {
        const Eigen::Vector3d points[]{p(a,b),p(a+1,b),p(a+1,b+1),p(a,b),p(a+1,b+1),p(a,b+1)};
        const float shade=.72f+.28f*float(std::sin((b+.5)*3.141592653589793/m));
        const Color c{color.x*shade,color.y*shade,color.z*shade,1};
        for(const auto& q:points) out.push_back(vertex(q,c));
    }
}
inline void controller(std::vector<Vertex>& out,const Pose& pose,bool left,bool ghost=false) {
    const auto body=ghost ? Teal : White;
    ellipsoid(out,pose,{0,-.025,.025},{.022,.07,.024},body);
    ellipsoid(out,pose,{0,.025,-.025},{.038,.015,.047},body);
    ellipsoid(out,pose,{0,.038,-.026},{.032,.005,.04},ghost ? Teal : Dark);
    const double side=left ? -1 : 1;
    ellipsoid(out,pose,{side*.014,.047,-.032},{.008,.007,.008},ghost ? Teal : Dark);
    ellipsoid(out,pose,{-side*.014,.044,-.019},{.005,.003,.005},ghost ? Teal : White);
    ellipsoid(out,pose,{-side*.019,.044,-.037},{.005,.003,.005},ghost ? Teal : White);
}
inline void tracker(std::vector<Vertex>& out,const Pose& pose,Color color=Dark) {
    ellipsoid(out,pose,{0,.009,0},{.031,.012,.031},color);
    for(int i=0;i<3;++i) {
        const double a=i*6.283185307179586/3;
        ellipsoid(out,pose,{.027*std::sin(a),.017,.027*std::cos(a)},{.015,.014,.015},color);
    }
    ellipsoid(out,pose,{0,.023,0},{.009,.002,.009},Teal);
}
inline std::vector<Vertex> panel(const Pose& pose,float width,float height) {
    std::vector<Vertex> out;
    const auto add=[&](float x,float y,float u,float v) { out.push_back(vertex(point(pose,x,y,0),{1,1,1,1},u,v)); };
    const float x=width/2,y=height/2;
    add(-x,y,0,0);add(x,y,1,0);add(x,-y,1,1);add(-x,y,0,0);add(x,-y,1,1);add(-x,-y,0,1);
    return out;
}

class Renderer {
    ComPtr<ID3D11VertexShader> vs;
    ComPtr<ID3D11PixelShader> ps;
    ComPtr<ID3D11InputLayout> layout;
    ComPtr<ID3D11Buffer> vertices,constants;
    ComPtr<ID3D11SamplerState> sampler;
    ComPtr<ID3D11RasterizerState> rasterizer;
    ComPtr<ID3D11DepthStencilState> depthState,overlayDepth;
    std::map<uint64_t,ComPtr<ID3D11DepthStencilView>> depths;
    struct Constants { XMFLOAT4X4 matrix; float texture{},padding[3]{}; };
public:
    ComPtr<ID3D11Device> device;
    ComPtr<ID3D11DeviceContext> context;
    ComPtr<IDXGISwapChain> mirror;
    ComPtr<ID3D11RenderTargetView> mirrorView;
    ComPtr<ID3D11Texture2D> depth;
    ComPtr<ID3D11DepthStencilView> depthView;
    ComPtr<ID3D11Texture2D> ui;
    ComPtr<ID3D11RenderTargetView> uiView;
    ComPtr<ID3D11ShaderResourceView> uiResource;
    void init(ID3D11Device* input,HWND window) {
        device=input;device->GetImmediateContext(&context);
        static const char shader[]=R"(
cbuffer Params:register(b0) { row_major float4x4 vp; float textured; float3 padding; };
struct Input { float3 p:POSITION; float4 c:COLOR; float2 uv:TEXCOORD; };
struct Output { float4 p:SV_POSITION; float4 c:COLOR; float2 uv:TEXCOORD; };
Output VS(Input i) { Output o; o.p=mul(float4(i.p,1),vp);o.c=i.c;o.uv=i.uv;return o; }
Texture2D image:register(t0); SamplerState samp:register(s0);
float4 PS(Output i):SV_TARGET { return textured>.5 ? float4(image.Sample(samp,i.uv).rgb*i.c.rgb,1) : i.c; }
)";
        ComPtr<ID3DBlob> v,p,error;
        check(D3DCompile(shader,sizeof(shader),nullptr,nullptr,nullptr,"VS","vs_5_0",0,0,&v,&error),"Compile scene vertex shader");
        check(D3DCompile(shader,sizeof(shader),nullptr,nullptr,nullptr,"PS","ps_5_0",0,0,&p,&error),"Compile scene pixel shader");
        check(device->CreateVertexShader(v->GetBufferPointer(),v->GetBufferSize(),nullptr,&vs),"Create vertex shader");
        check(device->CreatePixelShader(p->GetBufferPointer(),p->GetBufferSize(),nullptr,&ps),"Create pixel shader");
        const D3D11_INPUT_ELEMENT_DESC elements[]={{"POSITION",0,DXGI_FORMAT_R32G32B32_FLOAT,0,0,D3D11_INPUT_PER_VERTEX_DATA,0},{"COLOR",0,DXGI_FORMAT_R32G32B32A32_FLOAT,0,12,D3D11_INPUT_PER_VERTEX_DATA,0},{"TEXCOORD",0,DXGI_FORMAT_R32G32_FLOAT,0,28,D3D11_INPUT_PER_VERTEX_DATA,0}};
        check(device->CreateInputLayout(elements,3,v->GetBufferPointer(),v->GetBufferSize(),&layout),"Create scene layout");
        D3D11_BUFFER_DESC buffer{};buffer.ByteWidth=sizeof(Vertex)*100000;buffer.Usage=D3D11_USAGE_DYNAMIC;buffer.BindFlags=D3D11_BIND_VERTEX_BUFFER;buffer.CPUAccessFlags=D3D11_CPU_ACCESS_WRITE;
        check(device->CreateBuffer(&buffer,nullptr,&vertices),"Create scene vertex buffer");
        buffer={};buffer.ByteWidth=sizeof(Constants);buffer.Usage=D3D11_USAGE_DEFAULT;buffer.BindFlags=D3D11_BIND_CONSTANT_BUFFER;
        check(device->CreateBuffer(&buffer,nullptr,&constants),"Create scene constants");
        D3D11_SAMPLER_DESC sample{};sample.Filter=D3D11_FILTER_MIN_MAG_MIP_LINEAR;sample.AddressU=sample.AddressV=sample.AddressW=D3D11_TEXTURE_ADDRESS_CLAMP;sample.MaxLOD=D3D11_FLOAT32_MAX;
        check(device->CreateSamplerState(&sample,&sampler),"Create UI sampler");
        D3D11_RASTERIZER_DESC raster{};raster.FillMode=D3D11_FILL_SOLID;raster.CullMode=D3D11_CULL_NONE;raster.DepthClipEnable=TRUE;
        check(device->CreateRasterizerState(&raster,&rasterizer),"Create rasterizer");
        D3D11_DEPTH_STENCIL_DESC ds{};ds.DepthEnable=TRUE;ds.DepthWriteMask=D3D11_DEPTH_WRITE_MASK_ALL;ds.DepthFunc=D3D11_COMPARISON_LESS_EQUAL;
        check(device->CreateDepthStencilState(&ds,&depthState),"Create depth state");ds.DepthEnable=FALSE;
        check(device->CreateDepthStencilState(&ds,&overlayDepth),"Create overlay state");
        ComPtr<IDXGIDevice> dxgi;device.As(&dxgi);ComPtr<IDXGIAdapter> adapter;dxgi->GetAdapter(&adapter);ComPtr<IDXGIFactory> factory;adapter->GetParent(IID_PPV_ARGS(&factory));
        DXGI_SWAP_CHAIN_DESC swap{};swap.BufferDesc.Width=1280;swap.BufferDesc.Height=840;swap.BufferDesc.Format=DXGI_FORMAT_R8G8B8A8_UNORM;swap.SampleDesc.Count=1;swap.BufferUsage=DXGI_USAGE_RENDER_TARGET_OUTPUT;swap.BufferCount=2;swap.OutputWindow=window;swap.Windowed=TRUE;swap.SwapEffect=DXGI_SWAP_EFFECT_DISCARD;
        check(factory->CreateSwapChain(device.Get(),&swap,&mirror),"Create desktop mirror");ComPtr<ID3D11Texture2D> back;mirror->GetBuffer(0,IID_PPV_ARGS(&back));device->CreateRenderTargetView(back.Get(),nullptr,&mirrorView);
        D3D11_TEXTURE2D_DESC texture{};texture.Width=620;texture.Height=820;texture.MipLevels=texture.ArraySize=texture.SampleDesc.Count=1;texture.Format=DXGI_FORMAT_R8G8B8A8_UNORM;texture.BindFlags=D3D11_BIND_RENDER_TARGET|D3D11_BIND_SHADER_RESOURCE;
        check(device->CreateTexture2D(&texture,nullptr,&ui),"Create UI texture");device->CreateRenderTargetView(ui.Get(),nullptr,&uiView);device->CreateShaderResourceView(ui.Get(),nullptr,&uiResource);
    }
    void setTarget(ID3D11RenderTargetView* target,int width,int height,bool useDepth=true) {
        if(useDepth) {
            const auto key=(uint64_t(width)<<32)|uint32_t(height);
            if(!depths.count(key)) {
                ComPtr<ID3D11Texture2D> texture;D3D11_TEXTURE2D_DESC desc{};desc.Width=width;desc.Height=height;desc.MipLevels=desc.ArraySize=desc.SampleDesc.Count=1;desc.Format=DXGI_FORMAT_D32_FLOAT;desc.BindFlags=D3D11_BIND_DEPTH_STENCIL;
                check(device->CreateTexture2D(&desc,nullptr,&texture),"Create scene depth");check(device->CreateDepthStencilView(texture.Get(),nullptr,&depths[key]),"Create depth view");
            }
            depthView=depths[key];
            context->ClearDepthStencilView(depthView.Get(),D3D11_CLEAR_DEPTH,1,0);
        }
        context->OMSetRenderTargets(1,&target,useDepth ? depthView.Get() : nullptr);
        const D3D11_VIEWPORT view{0,0,float(width),float(height),0,1};context->RSSetViewports(1,&view);
        const float background[]{.78f,.84f,.88f,1};context->ClearRenderTargetView(target,background);
    }
    void exportPng(ID3D11RenderTargetView* target,const wchar_t* path) {
        ComPtr<ID3D11Resource> resource;target->GetResource(&resource);ComPtr<ID3D11Texture2D> source;check(resource.As(&source),"Read export texture");
        D3D11_TEXTURE2D_DESC desc;source->GetDesc(&desc);desc.Usage=D3D11_USAGE_STAGING;desc.BindFlags=0;desc.CPUAccessFlags=D3D11_CPU_ACCESS_READ;desc.MiscFlags=0;
        ComPtr<ID3D11Texture2D> staging;check(device->CreateTexture2D(&desc,nullptr,&staging),"Create frame export buffer");context->CopyResource(staging.Get(),source.Get());
        D3D11_MAPPED_SUBRESOURCE data;check(context->Map(staging.Get(),0,D3D11_MAP_READ,0,&data),"Read rendered frame");
        try {
            ComPtr<IWICImagingFactory> factory;check(CoCreateInstance(CLSID_WICImagingFactory,nullptr,CLSCTX_INPROC_SERVER,IID_PPV_ARGS(&factory)),"Create PNG encoder");
            ComPtr<IWICStream> stream;check(factory->CreateStream(&stream),"Create frame stream");check(stream->InitializeFromFilename(path,GENERIC_WRITE),"Open frame export");
            ComPtr<IWICBitmapEncoder> encoder;check(factory->CreateEncoder(GUID_ContainerFormatPng,nullptr,&encoder),"Create PNG encoder");check(encoder->Initialize(stream.Get(),WICBitmapEncoderNoCache),"Initialize PNG encoder");
            ComPtr<IWICBitmapFrameEncode> frame;check(encoder->CreateNewFrame(&frame,nullptr),"Create PNG frame");check(frame->Initialize(nullptr),"Initialize PNG frame");check(frame->SetSize(desc.Width,desc.Height),"Set PNG size");WICPixelFormatGUID format=GUID_WICPixelFormat32bppBGRA;check(frame->SetPixelFormat(&format),"Set PNG pixel format");
            if(!IsEqualGUID(format,GUID_WICPixelFormat32bppBGRA))throw std::runtime_error("PNG encoder changed pixel format");
            std::vector<BYTE> pixels(size_t(desc.Width)*desc.Height*4);
            for(UINT y=0;y<desc.Height;++y){const BYTE* row=static_cast<const BYTE*>(data.pData)+size_t(y)*data.RowPitch;BYTE* output=pixels.data()+size_t(y)*desc.Width*4;for(UINT x=0;x<desc.Width;++x){output[x*4]=row[x*4+2];output[x*4+1]=row[x*4+1];output[x*4+2]=row[x*4];output[x*4+3]=255;}}
            check(frame->WritePixels(desc.Height,desc.Width*4,UINT(pixels.size()),pixels.data()),"Write PNG pixels");check(frame->Commit(),"Commit PNG frame");check(encoder->Commit(),"Commit PNG file");
        }catch(...){context->Unmap(staging.Get(),0);throw;}
        context->Unmap(staging.Get(),0);
    }
    void draw(const std::vector<Vertex>& data,FXMMATRIX vp,bool lines=false,ID3D11ShaderResourceView* texture=nullptr,bool overlay=false) {
        if(data.empty())return;if(data.size()>100000)throw std::runtime_error("Scene vertex capacity exceeded");
        D3D11_MAPPED_SUBRESOURCE mapped;check(context->Map(vertices.Get(),0,D3D11_MAP_WRITE_DISCARD,0,&mapped),"Map scene vertices");std::memcpy(mapped.pData,data.data(),data.size()*sizeof(Vertex));context->Unmap(vertices.Get(),0);
        Constants parameters{};XMStoreFloat4x4(&parameters.matrix,vp);parameters.texture=texture ? 1.f:0.f;context->UpdateSubresource(constants.Get(),0,nullptr,&parameters,0,0);
        const UINT stride=sizeof(Vertex),offset=0;ID3D11Buffer* vb=vertices.Get();context->IASetVertexBuffers(0,1,&vb,&stride,&offset);context->IASetInputLayout(layout.Get());context->IASetPrimitiveTopology(lines ? D3D11_PRIMITIVE_TOPOLOGY_LINELIST:D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        context->VSSetShader(vs.Get(),nullptr,0);context->PSSetShader(ps.Get(),nullptr,0);ID3D11Buffer* cb=constants.Get();context->VSSetConstantBuffers(0,1,&cb);context->PSSetConstantBuffers(0,1,&cb);context->PSSetShaderResources(0,1,&texture);ID3D11SamplerState* ss=sampler.Get();context->PSSetSamplers(0,1,&ss);context->RSSetState(rasterizer.Get());context->OMSetDepthStencilState(overlay ? overlayDepth.Get():depthState.Get(),0);context->OMSetBlendState(nullptr,nullptr,0xffffffff);context->Draw(UINT(data.size()),0);
        ID3D11ShaderResourceView* none=nullptr;context->PSSetShaderResources(0,1,&none);
    }
};
} // namespace scene
#else
#include <EGL/egl.h>
#include <EGL/eglext.h>
#ifndef GL_GLEXT_PROTOTYPES
#define GL_GLEXT_PROTOTYPES 1
#endif
#include <GL/gl.h>
#include <GL/glcorearb.h>
#include <wayland-client.h>
#include <cstdio>
#include <cstdlib>
#include <Eigen/Dense>
#include <cstring>
#include <map>
#include <utility>
#include <vector>
#include <stdexcept>
#include <string>
#include <cmath>
#include <cstddef>
#include "calibration_math.h"

namespace scene {
using calibration::Pose;
struct Vertex { float p[3]; float c[4]; float uv[2]; };
struct Color { float x,y,z,w; };
inline const Color White{.88f,.91f,.94f,1}, Dark{.07f,.11f,.14f,1}, Teal{.08f,.65f,.7f,1}, Orange{.95f,.43f,.12f,1};
inline Eigen::Matrix4f matrix(const Pose& pose) { return pose.matrix().cast<float>(); }
inline Eigen::Vector3d point(const Pose& pose,double x,double y,double z) { return pose*Eigen::Vector3d(x,y,z); }
inline Vertex vertex(const Eigen::Vector3d& p,Color color,float u=0,float v=0) { Vertex out{};out.p[0]=float(p.x());out.p[1]=float(p.y());out.p[2]=float(p.z());out.c[0]=color.x;out.c[1]=color.y;out.c[2]=color.z;out.c[3]=color.w;out.uv[0]=u;out.uv[1]=v;return out; }
inline void line(std::vector<Vertex>& out,const Eigen::Vector3d& a,const Eigen::Vector3d& b,Color color) { out.push_back(vertex(a,color)); out.push_back(vertex(b,color)); }
inline void axes(std::vector<Vertex>& out,const Pose& pose,double size=.1) {
    line(out,pose.translation(),point(pose,size,0,0),{.85f,.15f,.1f,1});
    line(out,pose.translation(),point(pose,0,size,0),{.1f,.65f,.2f,1});
    line(out,pose.translation(),point(pose,0,0,size),{.1f,.3f,.85f,1});
}
inline void ellipsoid(std::vector<Vertex>& out,const Pose& pose,Eigen::Vector3d center,Eigen::Vector3d radii,Color color) {
    constexpr int n=16,m=10;
    const auto p=[&](int a,int b) -> Eigen::Vector3d {
        const double u=a*6.283185307179586/n,v=b*3.141592653589793/m;
        Eigen::Vector3d q(std::sin(v)*std::cos(u),std::cos(v),std::sin(v)*std::sin(u));
        return pose*(center+q.cwiseProduct(radii));
    };
    for(int b=0;b<m;++b) for(int a=0;a<n;++a) {
        const Eigen::Vector3d points[]{p(a,b),p(a+1,b),p(a+1,b+1),p(a,b),p(a+1,b+1),p(a,b+1)};
        const float shade=.72f+.28f*float(std::sin((b+.5)*3.141592653589793/m));
        const Color c{color.x*shade,color.y*shade,color.z*shade,1};
        for(const auto& q:points) out.push_back(vertex(q,c));
    }
}
inline void controller(std::vector<Vertex>& out,const Pose& pose,bool left,bool ghost=false) {
    const auto body=ghost ? Teal : White;
    ellipsoid(out,pose,{0,-.025,.025},{.022,.07,.024},body);
    ellipsoid(out,pose,{0,.025,-.025},{.038,.015,.047},body);
    ellipsoid(out,pose,{0,.038,-.026},{.032,.005,.04},ghost ? Teal : Dark);
    const double side=left ? -1 : 1;
    ellipsoid(out,pose,{side*.014,.047,-.032},{.008,.007,.008},ghost ? Teal : Dark);
    ellipsoid(out,pose,{-side*.014,.044,-.019},{.005,.003,.005},ghost ? Teal : White);
    ellipsoid(out,pose,{-side*.019,.044,-.037},{.005,.003,.005},ghost ? Teal : White);
}
inline void tracker(std::vector<Vertex>& out,const Pose& pose,Color color=Dark) {
    ellipsoid(out,pose,{0,.009,0},{.031,.012,.031},color);
    for(int i=0;i<3;++i) {
        const double a=i*6.283185307179586/3;
        ellipsoid(out,pose,{.027*std::sin(a),.017,.027*std::cos(a)},{.015,.014,.015},color);
    }
    ellipsoid(out,pose,{0,.023,0},{.009,.002,.009},Teal);
}
inline std::vector<Vertex> panel(const Pose& pose,float width,float height) {
    std::vector<Vertex> out;
    const auto add=[&](float x,float y,float u,float v) { out.push_back(vertex(point(pose,x,y,0),{1,1,1,1},u,v)); };
    const float x=width/2,y=height/2;
    add(-x,y,0,0);add(x,y,1,0);add(x,-y,1,1);add(-x,y,0,0);add(x,-y,1,1);add(-x,-y,0,1);
    return out;
}
inline std::string glError(const char* what) {
    const GLenum code=glGetError();
    return std::string(what)+": GL error "+std::to_string(code);
}
inline void glCheck(const char* what) { if(glGetError()!=GL_NO_ERROR) throw std::runtime_error(glError(what)); }

struct RenderTarget { GLuint fbo{},texture{},depthRenderbuffer{}; int width{},height{}; };

class Renderer {
    EGLDisplay display = EGL_NO_DISPLAY;
    EGLContext context = EGL_NO_CONTEXT;
    EGLSurface surface = EGL_NO_SURFACE;
    wl_display* wayland = nullptr;
    GLuint program{},vao{},vbo{};
    std::map<std::pair<int,int>,GLuint> depthBuffers;
    GLuint depthFor(int width,int height) {
        const auto key=std::make_pair(width,height);
        const auto found=depthBuffers.find(key);
        if(found!=depthBuffers.end()) return found->second;
        GLuint buffer{};glGenRenderbuffers(1,&buffer);glBindRenderbuffer(GL_RENDERBUFFER,buffer);glRenderbufferStorage(GL_RENDERBUFFER,GL_DEPTH_COMPONENT24,width,height);
        depthBuffers[key]=buffer;glBindRenderbuffer(GL_RENDERBUFFER,0);return buffer;
    }
public:
    EGLDisplay eglDisplay() const { return display; }
    EGLContext eglContext() const { return context; }
    wl_display* wlDisplay() const { return wayland; }
    void init() {
        display = eglGetPlatformDisplay(EGL_PLATFORM_SURFACELESS_MESA, EGL_DEFAULT_DISPLAY, nullptr);
        bool surfaceless = display!=EGL_NO_DISPLAY && eglInitialize(display,nullptr,nullptr);
        if(!surfaceless) {
            if(display!=EGL_NO_DISPLAY) eglTerminate(display);
            wayland = wl_display_connect(nullptr);
            if(wayland) display = eglGetPlatformDisplay(EGL_PLATFORM_WAYLAND_KHR, wayland, nullptr);
            if(display==EGL_NO_DISPLAY) display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
            if(display==EGL_NO_DISPLAY||!eglInitialize(display,nullptr,nullptr)) throw std::runtime_error("Create EGL display (surfaceless and Wayland both failed)");
        }
        eglBindAPI(EGL_OPENGL_API);
        // Mesa's surfaceless platform offers only a subset of surface types;
        // relax the surface-type requirement when no config matches.
        EGLConfig config{};
        const EGLint candidates[][15] = {
            {EGL_SURFACE_TYPE, EGL_PBUFFER_BIT, EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT, EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8, EGL_ALPHA_SIZE, 8, EGL_DEPTH_SIZE, 24, EGL_NONE},
            {EGL_SURFACE_TYPE, EGL_PBUFFER_BIT, EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT, EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8, EGL_ALPHA_SIZE, 8, EGL_NONE},
            {EGL_SURFACE_TYPE, EGL_DONT_CARE, EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT, EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8, EGL_ALPHA_SIZE, 8, EGL_DEPTH_SIZE, 24, EGL_NONE},
        };
        bool found = false;
        for(const auto& attributes : candidates) {
            EGLint count{};
            if(eglChooseConfig(display,attributes,&config,1,&count)&&count){found=true;break;}
        }
        if(!found) throw std::runtime_error("Choose EGL config");
        const EGLint contextAttributes[] = {EGL_CONTEXT_OPENGL_PROFILE_MASK, EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT, EGL_CONTEXT_MAJOR_VERSION, 3, EGL_CONTEXT_MINOR_VERSION, 3, EGL_NONE};
        context =        eglCreateContext(display,config,EGL_NO_CONTEXT,contextAttributes);
        if(context==EGL_NO_CONTEXT) throw std::runtime_error("Create EGL context");
        // A pbuffer must be current for FBO rendering on Mesa, even when the
        // display itself is the surfaceless platform (a bare EGL_NO_SURFACE
        // current context silently discards draws in some Mesa versions).
        const EGLint surfaceAttributes[] = {EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE};
        surface = eglCreatePbufferSurface(display,config,surfaceAttributes);
        if(surface==EGL_NO_SURFACE) throw std::runtime_error("Create preview surface");
        eglMakeCurrent(display,surface,surface,context);
        static const char vertexShader[] =
            "#version 330 core\n"
            "layout(location=0) in vec3 p;\n"
            "layout(location=1) in vec4 c;\n"
            "layout(location=2) in vec2 uv;\n"
            "uniform mat4 vp;\nuniform float textured;\n"
            "out vec4 color;out vec2 v_uv;out float v_textured;\n"
            "void main(){gl_Position=vp*vec4(p,1.0);color=c;v_uv=uv;v_textured=textured;}\n";
        static const char fragmentShader[] =
            "#version 330 core\n"
            "in vec4 color;in vec2 v_uv;in float v_textured;\n"
            "uniform sampler2D image;\n"
            "out vec4 frag;\n"
            "void main(){frag=v_textured>.5?vec4(texture(image,v_uv).rgb*color.rgb,1.0):color;}\n";
        const auto compile=[&](GLenum type,const char* source,const char* name) {
            const GLuint shader=glCreateShader(type);glShaderSource(shader,1,&source,nullptr);glCompileShader(shader);
            GLint status{};glGetShaderiv(shader,GL_COMPILE_STATUS,&status);
            if(!status){GLchar log[1024]{};glGetShaderInfoLog(shader,sizeof(log),nullptr,log);glDeleteShader(shader);throw std::runtime_error(std::string("Compile ")+name+": "+log);}
            return shader;
        };
        const GLuint vs=compile(GL_VERTEX_SHADER,vertexShader,"scene vertex shader");
        const GLuint fs=compile(GL_FRAGMENT_SHADER,fragmentShader,"scene fragment shader");
        program=glCreateProgram();glAttachShader(program,vs);glAttachShader(program,fs);glLinkProgram(program);
        GLint status{};glGetProgramiv(program,GL_LINK_STATUS,&status);
        if(!status){GLchar log[1024]{};glGetProgramInfoLog(program,sizeof(log),nullptr,log);throw std::runtime_error(std::string("Link scene program: ")+log);}
        glDeleteShader(vs);glDeleteShader(fs);
        glGenVertexArrays(1,&vao);glBindVertexArray(vao);
        glGenBuffers(1,&vbo);glBindBuffer(GL_ARRAY_BUFFER,vbo);
        glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(Vertex),(const void*)offsetof(Vertex,p));
        glVertexAttribPointer(1,4,GL_FLOAT,GL_FALSE,sizeof(Vertex),(const void*)offsetof(Vertex,c));
        glVertexAttribPointer(2,2,GL_FLOAT,GL_FALSE,sizeof(Vertex),(const void*)offsetof(Vertex,uv));
        glEnableVertexAttribArray(0);glEnableVertexAttribArray(1);glEnableVertexAttribArray(2);
        glBindVertexArray(0);glBindBuffer(GL_ARRAY_BUFFER,0);
    }
    RenderTarget createTarget(int width,int height) {
        RenderTarget target;target.width=width;target.height=height;
        glGenFramebuffers(1,&target.fbo);glBindFramebuffer(GL_FRAMEBUFFER,target.fbo);
        glGenTextures(1,&target.texture);glBindTexture(GL_TEXTURE_2D,target.texture);
        glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA8,width,height,0,GL_RGBA,GL_UNSIGNED_BYTE,nullptr);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
        glFramebufferTexture2D(GL_FRAMEBUFFER,GL_COLOR_ATTACHMENT0,GL_TEXTURE_2D,target.texture,0);
        GLuint depth{};glGenRenderbuffers(1,&depth);glBindRenderbuffer(GL_RENDERBUFFER,depth);glRenderbufferStorage(GL_RENDERBUFFER,GL_DEPTH_COMPONENT24,width,height);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER,GL_DEPTH_ATTACHMENT,GL_RENDERBUFFER,depth);
        target.depthRenderbuffer=depth;
        if(glCheckFramebufferStatus(GL_FRAMEBUFFER)!=GL_FRAMEBUFFER_COMPLETE){glBindFramebuffer(GL_FRAMEBUFFER,0);throw std::runtime_error("Create framebuffer");}
        glBindFramebuffer(GL_FRAMEBUFFER,0);glBindTexture(GL_TEXTURE_2D,0);
        return target;
    }
    void setTarget(GLuint target,int width,int height,bool useDepth=true) {
        glBindFramebuffer(GL_FRAMEBUFFER,target);
        if(useDepth) {
            const GLuint depth=depthFor(width,height);
            glFramebufferRenderbuffer(GL_FRAMEBUFFER,GL_DEPTH_ATTACHMENT,GL_RENDERBUFFER,depth);
        } else glFramebufferRenderbuffer(GL_FRAMEBUFFER,GL_DEPTH_ATTACHMENT,GL_RENDERBUFFER,0);
        glViewport(0,0,width,height);
        glDisable(GL_SCISSOR_TEST);glDisable(GL_STENCIL_TEST);
        if(useDepth){glEnable(GL_DEPTH_TEST);glDepthFunc(GL_LEQUAL);glDepthMask(GL_TRUE);}else glDisable(GL_DEPTH_TEST);
        glClearColor(.78f,.84f,.88f,1);
        glClear(GL_COLOR_BUFFER_BIT|(useDepth?GL_DEPTH_BUFFER_BIT:0));
        glCheckFramebufferStatus(GL_FRAMEBUFFER);
        if(glCheckFramebufferStatus(GL_FRAMEBUFFER)!=GL_FRAMEBUFFER_COMPLETE) throw std::runtime_error("Bind framebuffer target");
    }
    void unbindTarget() { glBindFramebuffer(GL_FRAMEBUFFER,0); glBindTexture(GL_TEXTURE_2D,0); }
    void exportPng(GLuint target,int width,int height,const char* path) {
        glBindFramebuffer(GL_FRAMEBUFFER,target);
        glFinish();
        std::vector<unsigned char> pixels(size_t(width)*height*4);
        glReadPixels(0,0,width,height,GL_RGBA,GL_UNSIGNED_BYTE,pixels.data());
        glBindFramebuffer(GL_FRAMEBUFFER,0);
        std::vector<unsigned char> flipped(pixels.size());
        const size_t row=size_t(width)*4;
        for(int y=0;y<height;++y) std::memcpy(flipped.data()+size_t(y)*row,pixels.data()+size_t(height-1-y)*row,row);
        if(stbi_write_png(path,width,height,4,flipped.data(),int(row))==0) throw std::runtime_error("Write PNG export");
    }
    void draw(const std::vector<Vertex>& data,const Eigen::Matrix4f& vp,bool lines=false,GLuint texture=0,bool overlay=false) {
        (void)overlay;
        if(data.empty())return;
        if(data.size()>100000)throw std::runtime_error("Scene vertex capacity exceeded");
        glUseProgram(program);
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER,vbo);
        glBufferData(GL_ARRAY_BUFFER,GLsizeiptr(data.size()*sizeof(Vertex)),data.data(),GL_STREAM_DRAW);
        glUniformMatrix4fv(glGetUniformLocation(program,"vp"),1,GL_FALSE,vp.data());
        glUniform1f(glGetUniformLocation(program,"textured"),texture?1:0);
        if(texture){glActiveTexture(GL_TEXTURE0);glBindTexture(GL_TEXTURE_2D,texture);glUniform1i(glGetUniformLocation(program,"image"),0);}
        if(overlay){glEnable(GL_DEPTH_TEST);glDepthFunc(GL_LEQUAL);glDepthMask(GL_FALSE);}else {glEnable(GL_DEPTH_TEST);glDepthFunc(GL_LEQUAL);glDepthMask(GL_TRUE);}
        glDrawArrays(lines?GL_LINES:GL_TRIANGLES,0,GLsizei(data.size()));
        if(texture){glBindTexture(GL_TEXTURE_2D,0);}
        glBindVertexArray(0);glBindBuffer(GL_ARRAY_BUFFER,0);glUseProgram(0);
        if(glGetError()!=GL_NO_ERROR) throw std::runtime_error("Draw scene");
    }
};
} // namespace scene

#endif
