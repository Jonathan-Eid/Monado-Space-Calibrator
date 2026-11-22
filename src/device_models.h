// SPDX-License-Identifier: MIT
#pragma once
#ifdef WIN32
#include "calibration_scene.h"
#include "device_identity.h"
#include <filesystem>
#include <fstream>
#include <cJSON.h>

namespace device_models {
struct Batch { std::vector<scene::Vertex> vertices;scene::ComPtr<ID3D11ShaderResourceView> texture; };
struct Asset { std::vector<Batch> batches;std::string error;bool loaded()const{return !batches.empty();} };
class Library {
    std::map<std::string,Asset> assets;
    ID3D11Device* device{};
    static void read(std::ifstream& file,void* output,size_t size) {if(!file.read(static_cast<char*>(output),size))throw std::runtime_error("Truncated device asset");}
    scene::ComPtr<ID3D11ShaderResourceView> texture(const std::vector<BYTE>& bytes) {
        using namespace scene;ComPtr<IWICImagingFactory> factory;check(CoCreateInstance(CLSID_WICImagingFactory,nullptr,CLSCTX_INPROC_SERVER,IID_PPV_ARGS(&factory)),"Model image factory");
        ComPtr<IWICStream> stream;check(factory->CreateStream(&stream),"Model image stream");check(stream->InitializeFromMemory(const_cast<BYTE*>(bytes.data()),DWORD(bytes.size())),"Model image bytes");
        ComPtr<IWICBitmapDecoder> decoder;check(factory->CreateDecoderFromStream(stream.Get(),nullptr,WICDecodeMetadataCacheOnLoad,&decoder),"Decode model texture");ComPtr<IWICBitmapFrameDecode> frame;check(decoder->GetFrame(0,&frame),"Model texture frame");UINT w,h;check(frame->GetSize(&w,&h),"Model texture size");if(!w||!h||w>4096||h>4096)throw std::runtime_error("Model texture size exceeded");
        ComPtr<IWICFormatConverter> converter;check(factory->CreateFormatConverter(&converter),"Model texture converter");check(converter->Initialize(frame.Get(),GUID_WICPixelFormat32bppRGBA,WICBitmapDitherTypeNone,nullptr,0,WICBitmapPaletteTypeCustom),"Model texture format");std::vector<BYTE> pixels(size_t(w)*h*4);check(converter->CopyPixels(nullptr,w*4,UINT(pixels.size()),pixels.data()),"Model texture pixels");
        D3D11_TEXTURE2D_DESC desc{};desc.Width=w;desc.Height=h;desc.MipLevels=desc.ArraySize=1;desc.Format=DXGI_FORMAT_R8G8B8A8_UNORM;desc.SampleDesc.Count=1;desc.Usage=D3D11_USAGE_IMMUTABLE;desc.BindFlags=D3D11_BIND_SHADER_RESOURCE;D3D11_SUBRESOURCE_DATA initial{pixels.data(),w*4,0};ComPtr<ID3D11Texture2D> image;check(device->CreateTexture2D(&desc,&initial,&image),"Upload model texture");ComPtr<ID3D11ShaderResourceView> view;check(device->CreateShaderResourceView(image.Get(),nullptr,&view),"Model texture view");return view;
    }
public:
    void init(ID3D11Device* input) {device=input;for(const auto* model:{"meta-quest-touch-plus","oculus-touch-v3","meta-quest-touch-pro"})for(const auto* side:{"left","right"})load(std::string(model)+"-"+side);load("vive-tracker-3");}
    void load(const std::string& key) {
        Asset asset;
        try {
            std::filesystem::path path=std::filesystem::path("assets/models")/(key+".vdm");
            // Manufacturer assets imported by the owner stay private and out of source control.
            if(key=="vive-tracker-3")path="data/models/vive-tracker-3.vdm";
            std::ifstream file(path,std::ios::binary);if(!file)throw std::runtime_error("Asset not installed");
            uint32_t header[3];read(file,header,sizeof(header));if(header[0]!=0x314d4456||header[1]!=1||!header[2]||header[2]>64)throw std::runtime_error("Invalid device asset header");
            for(uint32_t i=0;i<header[2];++i){uint32_t count,size;read(file,&count,4);read(file,&size,4);if(!count||count%3||count>100000||size>16*1024*1024)throw std::runtime_error("Device asset bounds exceeded");Batch batch;batch.vertices.resize(count);static_assert(sizeof(scene::Vertex)==36);read(file,batch.vertices.data(),count*sizeof(scene::Vertex));for(const auto& v:batch.vertices){const float* values=reinterpret_cast<const float*>(&v);for(int j=0;j<9;++j)if(!std::isfinite(values[j]))throw std::runtime_error("Nonfinite model vertex");if(std::abs(v.p.x)>1||std::abs(v.p.y)>1||std::abs(v.p.z)>1)throw std::runtime_error("Device dimensions exceeded");}if(size){std::vector<BYTE> bytes(size);read(file,bytes.data(),size);batch.texture=texture(bytes);}asset.batches.push_back(std::move(batch));}
        }catch(const std::exception& error){asset.batches.clear();asset.error=error.what();}
        assets[key]=std::move(asset);
    }
    const Asset* find(const std::string& key)const {const auto i=assets.find(key);return i==assets.end()?nullptr:&i->second;}
    bool loaded(const std::string& key)const {const auto* asset=find(key);return asset&&asset->loaded();}
    void draw(scene::Renderer& renderer,const std::string& key,const calibration::Pose& pose,DirectX::FXMMATRIX vp,bool ghost=false) const {
        const auto* asset=find(key);if(!asset||!asset->loaded())return;
        const auto transform=scene::matrix(pose)*vp;
        for(const auto& batch:asset->batches) {
            if(!ghost)renderer.draw(batch.vertices,transform,false,batch.texture.Get());
            else {auto tinted=batch.vertices;for(auto& v:tinted)v.c=scene::Teal;renderer.draw(tinted,transform);}
        }
    }
};
}
#else
#include "calibration_scene.h"
#include "device_identity.h"
#include <filesystem>
#include <fstream>
#include <stb_image.h>

namespace device_models {
struct Batch { std::vector<scene::Vertex> vertices;GLuint texture{}; };
struct Asset { std::vector<Batch> batches;std::string error;bool loaded()const{return !batches.empty();} };
class Library {
    std::map<std::string,Asset> assets;
    static void read(std::ifstream& file,void* output,size_t size) {if(!file.read(static_cast<char*>(output),size))throw std::runtime_error("Truncated device asset");}
    static GLuint texture(const std::vector<unsigned char>& bytes) {
        int width{},height{},channels{};
        unsigned char* pixels=stbi_load_from_memory(bytes.data(),int(bytes.size()),&width,&height,&channels,4);
        if(!pixels)throw std::runtime_error("Decode model texture");
        if(!width||!height||width>4096||height>4096){stbi_image_free(pixels);throw std::runtime_error("Model texture size exceeded");}
        GLuint image{};glGenTextures(1,&image);glBindTexture(GL_TEXTURE_2D,image);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_EDGE);glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_LINEAR);glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
        glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA8,width,height,0,GL_RGBA,GL_UNSIGNED_BYTE,pixels);
        glGenerateMipmap(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D,0);
        stbi_image_free(pixels);
        if(glGetError()!=GL_NO_ERROR)throw std::runtime_error("Upload model texture");
        return image;
    }
public:
    void init() {for(const auto* model:{"meta-quest-touch-plus","oculus-touch-v3","meta-quest-touch-pro"})for(const auto* side:{"left","right"})load(std::string(model)+"-"+side);load("vive-tracker-3");}
    void load(const std::string& key) {
        Asset asset;
        try {
            std::filesystem::path path=std::filesystem::path("assets/models")/(key+".vdm");
            // Manufacturer assets imported by the owner stay private and out of source control.
            if(key=="vive-tracker-3")path="data/models/vive-tracker-3.vdm";
            std::ifstream file(path,std::ios::binary);if(!file)throw std::runtime_error("Asset not installed");
            uint32_t header[3];read(file,header,sizeof(header));if(header[0]!=0x314d4456||header[1]!=1||!header[2]||header[2]>64)throw std::runtime_error("Invalid device asset header");
            for(uint32_t i=0;i<header[2];++i){uint32_t count,size;read(file,&count,4);read(file,&size,4);if(!count||count%3||count>100000||size>16*1024*1024)throw std::runtime_error("Device asset bounds exceeded");Batch batch;batch.vertices.resize(count);static_assert(sizeof(scene::Vertex)==36);read(file,batch.vertices.data(),count*sizeof(scene::Vertex));for(const auto& v:batch.vertices){for(int j=0;j<9;++j){const float value=reinterpret_cast<const float*>(&v)[j];if(!std::isfinite(value))throw std::runtime_error("Nonfinite model vertex");}if(std::abs(v.p[0])>1||std::abs(v.p[1])>1||std::abs(v.p[2])>1)throw std::runtime_error("Device dimensions exceeded");}if(size){std::vector<unsigned char> bytes(size);read(file,bytes.data(),size);batch.texture=texture(bytes);}asset.batches.push_back(std::move(batch));}
        }catch(const std::exception& error){asset.batches.clear();asset.error=error.what();}
        assets[key]=std::move(asset);
    }
    const Asset* find(const std::string& key)const {const auto i=assets.find(key);return i==assets.end()?nullptr:&i->second;}
    bool loaded(const std::string& key)const {const auto* asset=find(key);return asset&&asset->loaded();}
    void draw(scene::Renderer& renderer,const std::string& key,const calibration::Pose& pose,const Eigen::Matrix4f& vp,bool ghost=false) const {
        const auto* asset=find(key);if(!asset||!asset->loaded())return;
        const auto model=scene::matrix(pose);
        const auto transform=vp*model;
        for(const auto& batch:asset->batches) {
            if(!ghost)renderer.draw(batch.vertices,transform,false,batch.texture);
            else {auto tinted=batch.vertices;for(auto& v:tinted){v.c[0]=scene::Teal.x;v.c[1]=scene::Teal.y;v.c[2]=scene::Teal.z;v.c[3]=1;}renderer.draw(tinted,transform);}
        }
    }
};
}

#endif
