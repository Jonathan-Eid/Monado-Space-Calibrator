// SPDX-License-Identifier: MIT
// Linux headless port of the regular calibration app.
// Windows runs the full D3D11/OpenXR app (see calibration_app.cpp); this
// variant keeps the same model logic, inspection API and IPC, but runs
// synthetic preview without a window, GPU renderer or OpenXR runtime. It is
// the offline/CI pipeline for Linux; hardware Quest-space calibration needs
// the VDXR OpenXR runtime, which is Windows-only (see BUILD-LINUX.md).
#include "calibration_math.h"
#include "spacecal_regular.h"
#include "calibration_inspection.h"
#include "device_identity.h"
#include "vdxr_tracker_ipc.h"
#define SURVIVE_ENABLE_FULL_API 1
#include <libsurvive/survive_api.h>
#include <libsurvive/survive.h>
#undef float_format
#include <unistd.h>
#include <sys/wait.h>
#include <cJSON.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <thread>

using namespace calibration;
namespace fs=std::filesystem;
using Clock=std::chrono::steady_clock;
static constexpr size_t FastCaptureSamples=100,SlowCaptureSamples=250,ValidationSamples=50;
static constexpr double SampleInterval=.05;
static std::atomic_bool exitRequested{false};
static std::string startupError;
static bool boundedRun=false;

static void handleSignal(int){exitRequested=true;}

static std::string number(double v,int precision=1) { std::ostringstream s;s<<std::fixed<<std::setprecision(precision)<<v;return s.str(); }
static std::string poseText(const Pose& pose) {
    const Eigen::Quaterniond q(pose.rotation());std::ostringstream s;s<<std::setprecision(12)<<pose.translation().x()<<" "<<pose.translation().y()<<" "<<pose.translation().z()<<" "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<"\n";return s.str();
}
static void atomicWrite(const fs::path& path,const std::string& contents) {
    fs::path temporary=path;temporary+=".tmp";
    {std::ofstream file(temporary,std::ios::binary);if(!file)throw std::runtime_error("Cannot save profile");file<<contents;if(!file)throw std::runtime_error("Profile write failed");}
    std::error_code code;
    fs::rename(temporary,path,code); // POSIX rename over an existing file is atomic
    if(code)throw std::runtime_error("Cannot finish profile save");
}

// Offline device model verification. The Windows library uploads meshes and
// textures to D3D11; headless has no renderer, so batches are parsed and
// validated (bounds, finiteness, count limits) and textures are only checked
// for presence. state.device_models.asset_loaded reports mesh integrity, and
// asset_error states that textures are not decoded here.
class HeadlessAssets {
public:
    struct Asset { size_t batches=0;std::string error;bool loaded()const{return batches>0;} };
    void init() {
        for(const auto* model:{"meta-quest-touch-plus","oculus-touch-v3","meta-quest-touch-pro"})
            for(const auto* side:{"left","right"})load(std::string(model)+"-"+side);
        load("vive-tracker-3");
    }
    void load(const std::string& key) {
        Asset asset;
        try {
            fs::path path=fs::path("assets/models")/(key+".vdm");
            // Manufacturer assets imported by the owner stay private and out of source control.
            if(key=="vive-tracker-3")path="data/models/vive-tracker-3.vdm";
            std::ifstream file(path,std::ios::binary);if(!file)throw std::runtime_error("Asset not installed");
            auto read=[&](void* output,size_t size){if(!file.read(static_cast<char*>(output),size))throw std::runtime_error("Truncated device asset");};
            uint32_t header[3];read(header,sizeof(header));
            if(header[0]!=0x314d4456||header[1]!=1||!header[2]||header[2]>64)throw std::runtime_error("Invalid device asset header");
            for(uint32_t i=0;i<header[2];++i) {
                uint32_t count,size;read(&count,4);read(&size,4);
                if(!count||count%3||count>100000||size>16*1024*1024)throw std::runtime_error("Device asset bounds exceeded");
                static_assert(sizeof(sceneVertex)==36);
                if(count*sizeof(sceneVertex)>64*1024*1024)throw std::runtime_error("Device asset bounds exceeded");
                std::vector<char> vertices(count*sizeof(sceneVertex));read(vertices.data(),vertices.size());
                for(size_t v=0;v<count;++v) {
                    const float* values=reinterpret_cast<const float*>(vertices.data()+v*sizeof(sceneVertex));
                    for(int j=0;j<9;++j)if(!std::isfinite(values[j]))throw std::runtime_error("Nonfinite model vertex");
                    if(std::abs(values[0])>1||std::abs(values[1])>1||std::abs(values[2])>1)throw std::runtime_error("Device dimensions exceeded");
                }
                if(size)file.seekg(size,std::ios::cur); // texture present, not decoded headlessly
                if(!file)throw std::runtime_error("Truncated device asset");
                ++asset.batches;
            }
        }catch(const std::exception& error){asset.batches=0;asset.error=error.what();}
        assets[key]=std::move(asset);
    }
    const Asset* find(const std::string& key)const {const auto i=assets.find(key);return i==assets.end()?nullptr:&i->second;}
    bool loaded(const std::string& key)const {const auto* asset=find(key);return asset&&asset->loaded();}
private:
    struct sceneVertex{float p[3],c[4];float uv[2];}; // same layout as the Windows renderer vertex
    std::map<std::string,Asset> assets;
};

struct Located { Pose pose{Pose::Identity()}; uint32_t flags{}; bool tracked() const { return (flags&15)==15; } };
struct RawPose { Pose pose{Pose::Identity()}; device_models::Identity identity; double sampleTime{},age{}; Clock::time_point received{}; bool fresh() const { return sampleTime>0 && std::chrono::duration<double>(Clock::now()-received).count()+age<.1; } };
enum class Step { Devices,Capture,Fitting,Validate,Ready,Saved,Error };
struct Model {
    SurviveSimpleContext* source{};
    std::future<SurviveSimpleContext*> hardware;
    std::future<Fit> pendingFit;
    std::map<std::string,RawPose> devices;
    std::string selected,message="Connecting to tracker receivers...";
    Step step=Step::Devices;int reference=0,role=1;
    bool preview=false,mapLocked=false,hasFit=false,validationRunning=false,service=false;
    bool roomSolverDisabled=false;
    double captureSeconds{},validationSeconds{},instantPosition{},instantAngle{},lastSample{};
    std::vector<Pair> samples;
    std::vector<std::string> captureTrace;
    std::string captureFile,captureExportError;
    Metrics validation;
    Eigen::Vector3d validationMin{Eigen::Vector3d::Zero()},validationMax{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d validationFirst{Eigen::Matrix3d::Identity()};
    double validationRotation{};
    Fit result;
    Located head,grips[2];
    HeadlessAssets models;
    device_models::Identity controllerIdentity[2];
    std::string controllerOverride[2]{"auto","auto"},systemName,interactionProfiles[2];
    bool modelSettings=false;
    bool fastMode=true;
    external_trackers::OwnerLock ipcOwner;
    external_trackers::Writer ipc;
    size_t captureTarget()const{return fastMode ? FastCaptureSamples:SlowCaptureSamples;}
    Clock::time_point inspectionStarted=Clock::now();
    std::string controllerKey(int side)const{return controllerIdentity[side].model+(side ? "-right":"-left");}
    void updateIdentities(){for(int side=0;side<2;++side)controllerIdentity[side]=device_models::controller(systemName,interactionProfiles[side],controllerOverride[side]);}
    const char* phase() const { switch(step){case Step::Devices:return "devices";case Step::Capture:return "capture";case Step::Fitting:return "fitting";case Step::Validate:return "validate";case Step::Ready:return "ready_to_save";case Step::Saved:return "saved";default:return "error";} }
    bool busy() const { return step==Step::Capture||step==Step::Fitting||validationRunning||service; }
    bool can(const std::string& action) const {
        if(action=="exit")return true;
        if(action=="place_panel")return !service;
        if(action=="show_device_models")return true;
        if(action=="set_fast_mode")return !busy();
        if(action=="select_left_model"||action=="select_right_model")return !busy();
        if(action=="select_reference"||action=="select_tracker"||action=="select_role"||action=="reset")return !busy();
        if(action=="start_capture")return !preview&&!busy()&&mapLocked&&grips[reference].tracked()&&devices.count(selected)&&devices.at(selected).fresh();
        if(action=="fit")return step==Step::Capture;
        if(action=="start_validation")return !preview&&hasFit&&!busy();
        if(action=="stop_validation")return validationRunning;
        if(action=="save_profile")return !preview&&step==Step::Ready;
        if(action=="load_profile")return !preview&&!busy()&&fs::exists("data/profile.json");
        if(action=="start_service")return !preview&&!service&&step==Step::Saved;
        if(action=="lock_map")return !preview&&!busy()&&!mapLocked&&source&&!hardware.valid()&&devices.count(selected)&&devices.at(selected).fresh();
        if(action=="identify_reference")return !preview&&!service;
        return false;
    }
    bool perform(const std::string& action,int index=0,const std::string& text="") {
        if(!can(action))return false;
        if(action=="select_reference"){if(index<0||index>1)return false;reference=index;reset();}
        else if(action=="show_device_models"){if(index<0||index>1)return false;modelSettings=index!=0;}
        else if(action=="set_fast_mode"){if(index<0||index>1)return false;atomicWrite("data/calibration-settings.json",index ? "{\"fast_mode\":true}":"{\"fast_mode\":false}");fastMode=index!=0;}
        else if(action=="select_left_model"||action=="select_right_model"){
            if(!device_models::controllerChoice(text))return false;controllerOverride[action=="select_right_model" ? 1:0]=text;updateIdentities();
            std::ostringstream json;json<<"{\"left\":\""<<controllerOverride[0]<<"\",\"right\":\""<<controllerOverride[1]<<"\"}";atomicWrite("data/device-models.json",json.str());
        }
        else if(action=="select_tracker"){if(!devices.count(text))return false;selected=text;reset();}
        else if(action=="select_role"){if(index<0||index>=14)return false;role=index;}
        else if(action=="start_capture")startCapture();
        else if(action=="fit")fitCapture();
        else if(action=="start_validation"){validation={};validationSeconds=0;validationRunning=true;step=Step::Validate;}
        else if(action=="stop_validation")validationRunning=false;
        else if(action=="save_profile")save();
        else if(action=="load_profile")load();
        else if(action=="start_service")startService();
        else if(action=="reset")reset();
        else if(action=="identify_reference")return false; // haptics need the OpenXR session; never faked headlessly
        else if(action=="lock_map"){auto* previous=source;source=nullptr;devices.clear();initHardware(true,previous);}
        else if(action=="exit")exitRequested=true;
        else if(action=="place_panel"){} // no panel geometry without a renderer; kept for protocol parity
        return true;
    }
    ~Model() {
        if(pendingFit.valid()) pendingFit.wait();
        if(hardware.valid()) { try { auto* value=hardware.get();if(value)survive_simple_close(value); }catch(...){} }
        if(source)survive_simple_close(source);
    }
    void initHardware(bool lockMap,SurviveSimpleContext* previous=nullptr) {
        mapLocked=lockMap;
        hardware=std::async(std::launch::async,[lockMap,previous]() {
            if(previous)survive_simple_close(previous);
            char program[]="vdxr-calibration",flag[]="--configfile",path[]="./data/lighthouse.json",disable[]="--disable-calibrate",enabled[]="1",disabled[]="0";char* args[]{program,flag,path,disable,lockMap ? enabled:disabled};
            auto* context=survive_simple_init(5,args);if(!context)throw std::runtime_error("Tracker initialization failed");survive_simple_start_thread(context);return context;
        });
    }
    void pollHardware() {
        if(hardware.valid()&&hardware.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
            try { source=hardware.get();roomSolverDisabled=survive_configi(survive_simple_get_ctx(source),"disable-calibrate",SC_GET,0)!=0;if(mapLocked&&!roomSolverDisabled)throw std::runtime_error("Room map lock was not honored by libsurvive");message=mapLocked ? "Lighthouse map loaded and scene solver disabled. Select the device pair.":"Place the tracker stationary and visible, then lock its Lighthouse map."; }
            catch(const std::exception& error){message=error.what();step=Step::Error;}
        }
        if(!source)return;
        for(int i=0;i<64;++i) {
            const auto* object=survive_simple_get_next_updated(source);if(!object)break;
            if(survive_simple_object_get_type(object)!=SurviveSimpleObject_OBJECT)continue;
            const char* serial=survive_simple_serial_number(object);if(!serial||!serial[0])continue;
            auto& raw=devices[serial];SurvivePose pose;
            if(raw.identity.source=="unavailable")raw.identity=device_models::tracker(survive_simple_json_config(object));
            const double time=survive_simple_object_get_latest_pose(object,&pose);
            const double age=survive_simple_run_time_since_epoch(source)-time;
            Eigen::Quaterniond q(pose.Rot[0],pose.Rot[1],pose.Rot[2],pose.Rot[3]);
            const Eigen::Vector3d p(pose.Pos[0],pose.Pos[1],pose.Pos[2]);
            if(time<=0||!std::isfinite(age)||age<-.005||age>.1||!p.allFinite()||!q.coeffs().allFinite()||std::abs(q.norm()-1)>.01)continue;
            raw.pose=Pose::Identity();raw.pose.linear()=q.normalized().toRotationMatrix();raw.pose.translation()=p;raw.sampleTime=time;raw.age=std::max(0.0,age);raw.received=Clock::now();
            if(selected.empty())selected=serial;
        }
    }
    RawPose* raw() { const auto it=devices.find(selected);return it==devices.end() ? nullptr:&it->second; }
    void reset() { hasFit=false;validationRunning=false;samples.clear();captureTrace.clear();captureSeconds=validationSeconds=0;lastSample=0;validation={};step=Step::Devices;message="Select devices, attach them rigidly, then record."; }
    void startCapture() {
        if(preview){message="Preview contains simulated devices; hardware calibration is disabled.";return;}
        if(!mapLocked){message="Lock a stable Lighthouse map before recording.";return;}
        if(!raw()||!raw()->fresh()||!grips[reference].tracked()){message="Both selected devices must provide live poses.";return;}
        reset();step=Step::Capture;lastPair=Clock::now();message="Keep the pair rigid. Move slowly and rotate around several axes.";
    }
    void fitCapture() {
        try {
            fs::create_directories("data/captures");captureFile="data/captures/"+std::to_string(getpid())+"-"+std::to_string(Clock::now().time_since_epoch().count())+".csv";
            std::ostringstream csv;csv<<"sx,sy,sz,sqx,sqy,sqz,sqw,tx,ty,tz,tqx,tqy,tqz,tqw,cx,cy,cz,cqx,cqy,cqz,cqw,ox,oy,oz,oqx,oqy,oqz,oqw,clock_seconds,source_time_seconds,source_age_seconds,target_flags,other_flags,reference_side\n";for(const auto& row:captureTrace)csv<<row<<'\n';atomicWrite(captureFile,csv.str());captureExportError.clear();
        }catch(const std::exception& error){captureExportError=error.what();captureFile.clear();}
        if(samples.size()<100){step=Step::Error;message="Too few valid samples. Record again with both devices tracked.";return;}
        const auto copy=samples;step=Step::Fitting;message="Checking movement and fitting room + mounting transforms...";
        pendingFit=std::async(std::launch::async,[copy](){return calibration::fitSpaceCalibrator(copy);});
    }
    Clock::time_point lastPair=Clock::now();
    void tick() {
        pollHardware();const auto now=Clock::now();const double delta=std::min(.05,std::chrono::duration<double>(now-lastTick).count());lastTick=now;
        if(pendingFit.valid()&&pendingFit.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
            try { result=pendingFit.get();hasFit=true;step=Step::Ready;message="Calibrated. The tracker is now displayed in Quest space.";try{save();message="Calibrated and saved. The tracker is now displayed in Quest space.";}catch(const std::exception& error){message=std::string("Alignment applied, but saving failed: ")+error.what();} }
            catch(const std::exception& error){hasFit=false;step=Step::Error;message=error.what();}
        }
        auto* r=raw();bool valid=r&&r->fresh()&&grips[reference].tracked();
        if(valid) {
            const double age=r->age+std::chrono::duration<double>(now-r->received).count();
            if(r->sampleTime!=lastSample && std::chrono::duration<double>(now-lastPair).count()>=SampleInterval) {
                const double pairDelta=std::min(.05,std::chrono::duration<double>(now-lastPair).count());
                lastSample=r->sampleTime;lastPair=now;
                if(step==Step::Capture){
                    const Pair pair{r->pose,grips[reference].pose};samples.push_back(pair);captureSeconds+=pairDelta;
                    const auto other=grips[1-reference];std::ostringstream row;row<<std::setprecision(17);
                    const auto append=[&](const Pose& p){const Eigen::Quaterniond q(p.rotation());row<<p.translation().x()<<','<<p.translation().y()<<','<<p.translation().z()<<','<<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w()<<',';};
                    append(pair.source);append(pair.target);append(grips[reference].pose);append(other.pose);row<<std::chrono::duration<double>(now.time_since_epoch()).count()<<','<<r->sampleTime<<','<<age<<','<<grips[reference].flags<<','<<other.flags<<','<<reference;captureTrace.push_back(row.str());
                }
                if(validationRunning&&hasFit){
                    const Pair pair{r->pose,grips[reference].pose};
                    if(validation.count==0){validationMin=validationMax=pair.target.translation();validationFirst=pair.target.rotation();validationRotation=0;}
                    validationMin=validationMin.cwiseMin(pair.target.translation());validationMax=validationMax.cwiseMax(pair.target.translation());
                    validationRotation=std::max(validationRotation,Eigen::AngleAxisd(validationFirst.transpose()*pair.target.rotation()).angle());
                    validation.add(pair,result.world,result.mount);validationSeconds+=pairDelta;
                }
            }
        }
        if(step==Step::Capture) {
            message=valid ? "Recording. Keep the pair rigid; use several rotation axes.":"Recording paused: waiting for both devices to be tracked.";
            if(samples.size()>=captureTarget())fitCapture();
        }
        const bool motionCovered=(validationMax-validationMin).norm()>=.25&&validationRotation>=.6;
        if(validationRunning&&validation.count>=ValidationSamples&&!motionCovered)message="Validation needs broader movement: cover 25 cm and rotate around different directions.";
        if(validationRunning&&validation.count>=ValidationSamples&&(motionCovered||validationSeconds>=60)) {
            validationRunning=false;
            if(motionCovered&&validation.passes()){step=Step::Ready;message="Validation passed. Inspect the 3D overlay, then save the profile.";}
            else{step=Step::Validate;message="Validation failed. Inspect the gap, reattach the pair, and record again.";}
        }
        if(service)publish();
    }
    Clock::time_point lastTick=Clock::now();
    void save() {
        if(preview||step!=Step::Ready)return;
        atomicWrite("data/stage.txt",poseText(result.world));atomicWrite("data/mount.txt",poseText(result.mount));
        std::ostringstream json;json<<"{\n  \"kind\": \"regular\",\n  \"serial\": \""<<selected<<"\",\n  \"role\": \""<<external_trackers::Roles[role]<<"\",\n  \"fit_holdout_samples\": "<<result.reserved.count<<",\n  \"fit_holdout_position_rms_m\": "<<result.reserved.positionRms()<<",\n  \"fit_holdout_angle_rms_degrees\": "<<result.reserved.angleRms()<<",\n  \"validation_samples\": "<<validation.count<<",\n  \"position_rms_m\": "<<validation.positionRms()<<",\n  \"angle_rms_degrees\": "<<validation.angleRms()<<"\n}\n";
        atomicWrite("data/profile.json",json.str());step=Step::Saved;message="Profile saved locally. Recalibrate after recentering or moving base stations.";
    }
    void load() {
        if(preview)return;
        try {
            const auto read=[](const char* path){std::ifstream file(path);double x,y,z,qx,qy,qz,qw;if(!(file>>x>>y>>z>>qx>>qy>>qz>>qw))throw std::runtime_error("Saved transform is missing");Eigen::Quaterniond q(qw,qx,qy,qz);Eigen::Vector3d p(x,y,z);if(!p.allFinite()||!q.coeffs().allFinite()||std::abs(q.norm()-1)>.01)throw std::runtime_error("Saved transform is invalid");Pose value=Pose::Identity();value.linear()=q.normalized().toRotationMatrix();value.translation()=p;return value;};
            std::ifstream profile("data/profile.json");const std::string json((std::istreambuf_iterator<char>(profile)),{});
            const auto value=[&](const std::string& key){auto i=json.find("\""+key+"\"");if(i==std::string::npos)throw std::runtime_error("Profile metadata is incomplete");i=json.find(':',i);i=json.find('"',i);const auto end=json.find('"',i+1);if(i==std::string::npos||end==std::string::npos)throw std::runtime_error("Invalid profile metadata");return json.substr(i+1,end-i-1);};
            const auto serial=value("serial"),savedRole=value("role");int index=-1;for(int i=0;i<14;++i)if(savedRole==external_trackers::Roles[i])index=i;
            if(index<0||serial.empty()||serial.find_first_not_of("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_-")!=std::string::npos)throw std::runtime_error("Unknown profile device or role");
            result={};result.world=read("data/stage.txt");result.mount=read("data/mount.txt");selected=serial;role=index;hasFit=true;validation={};step=Step::Validate;message="Saved profile loaded. Validate it in the current room before enabling tracking.";
        }catch(const std::exception& error){message=error.what();}
    }
    void startService() {
        if(step!=Step::Saved)return;
        if(!ipcOwner.acquire()){message="Cannot start tracker transport: another provider is running.";return;}
        if(!ipc.create()){message="Cannot start tracker transport.";return;}
        service=true;message="Tracking service active. Keep this process running while playing.";
    }
    void publish() {
        external_trackers::State state;state.frequency=external_trackers::Frequency;state.samples[role].configured=1;
        auto* r=raw();if(r&&r->fresh()) {
            const Pose p=result.world*r->pose;const Eigen::Quaterniond q(p.rotation());auto& sample=state.samples[role];
            const double age=r->age+std::chrono::duration<double>(Clock::now()-r->received).count();sample.qpc=external_trackers::now()-int64_t(age*external_trackers::Frequency);
            for(int i=0;i<3;++i)sample.position[i]=float(p.translation()[i]);sample.orientation[0]=float(q.x());sample.orientation[1]=float(q.y());sample.orientation[2]=float(q.z());sample.orientation[3]=float(q.w());
        }
        ipc.publish(state);
    }
};

static std::string snapshot(Model& model,uint64_t frame) {
    cJSON* root=cJSON_CreateObject();cJSON_AddNumberToObject(root,"schema",1);cJSON_AddNumberToObject(root,"pid",double(getpid()));cJSON_AddNumberToObject(root,"frame",double(frame));cJSON_AddNumberToObject(root,"uptime_ms",std::chrono::duration<double,std::milli>(Clock::now()-model.inspectionStarted).count());
    cJSON_AddStringToObject(root,"mode",model.preview ? "synthetic_preview":model.service ? "fixed_tracking_service":"hardware_vr");cJSON_AddStringToObject(root,"phase",model.phase());cJSON_AddStringToObject(root,"message",model.message.c_str());
    cJSON_AddStringToObject(root,"reference",model.reference ? "right":"left");cJSON_AddStringToObject(root,"tracker",model.selected.c_str());cJSON_AddStringToObject(root,"role",external_trackers::Roles[model.role]);cJSON_AddBoolToObject(root,"map_locked",model.mapLocked);cJSON_AddBoolToObject(root,"has_fit",model.hasFit);
    cJSON_AddNumberToObject(root,"capture_seconds",model.captureSeconds);cJSON_AddNumberToObject(root,"capture_samples",double(model.samples.size()));cJSON_AddBoolToObject(root,"validation_running",model.validationRunning);cJSON_AddNumberToObject(root,"validation_seconds",model.validationSeconds);cJSON_AddNumberToObject(root,"validation_samples",double(model.validation.count));cJSON_AddNumberToObject(root,"position_gap_m",model.instantPosition);cJSON_AddNumberToObject(root,"angle_gap_degrees",model.instantAngle);cJSON_AddNumberToObject(root,"validation_position_rms_m",model.validation.positionRms());cJSON_AddNumberToObject(root,"validation_angle_rms_degrees",model.validation.angleRms());
    const auto pose=[](const Pose& p){cJSON* object=cJSON_CreateObject();const Eigen::Quaterniond q(p.rotation());const double position[]{p.translation().x(),p.translation().y(),p.translation().z()},orientation[]{q.x(),q.y(),q.z(),q.w()};cJSON_AddItemToObject(object,"position_m",cJSON_CreateDoubleArray(position,3));cJSON_AddItemToObject(object,"orientation_xyzw",cJSON_CreateDoubleArray(orientation,4));return object;};
    cJSON* devices=cJSON_AddArrayToObject(root,"devices");
    for(int i=0;i<3;++i){const Located& value=i==0 ? model.head:model.grips[i-1];cJSON* device=pose(value.pose);cJSON_AddStringToObject(device,"name",i==0 ? "head":i==1 ? "left_controller":"right_controller");cJSON_AddStringToObject(device,"space","Quest STAGE");cJSON_AddNumberToObject(device,"location_flags",double(value.flags));cJSON_AddBoolToObject(device,"tracked",value.tracked());cJSON_AddItemToArray(devices,device);}
    for(const auto& entry:model.devices){cJSON* device=pose(entry.second.pose);cJSON_AddStringToObject(device,"name",entry.first.c_str());cJSON_AddStringToObject(device,"space","raw libsurvive world");cJSON_AddBoolToObject(device,"fresh",entry.second.fresh());cJSON_AddNumberToObject(device,"age_seconds",entry.second.age+std::chrono::duration<double>(Clock::now()-entry.second.received).count());cJSON_AddItemToArray(devices,device);}
    cJSON_AddStringToObject(root,"system_name",model.systemName.c_str());
    cJSON_AddStringToObject(root,"calibration_solver","hyblocker_regular_kabsch_translation");cJSON_AddBoolToObject(root,"room_solver_disabled",model.roomSolverDisabled);
    cJSON_AddStringToObject(root,"capture_file",model.captureFile.c_str());cJSON_AddStringToObject(root,"capture_export_error",model.captureExportError.c_str());
    cJSON_AddBoolToObject(root,"alignment_applied",model.hasFit);cJSON_AddBoolToObject(root,"profile_saved",model.step==Step::Saved);cJSON_AddNumberToObject(root,"fit_holdout_samples",double(model.result.reserved.count));cJSON_AddNumberToObject(root,"fit_holdout_position_rms_m",model.result.reserved.positionRms());
    if(model.hasFit){cJSON_AddItemToObject(root,"quest_from_tracker_world",pose(model.result.world));cJSON_AddItemToObject(root,"tracker_from_controller_mount",pose(model.result.mount));cJSON* aligned=cJSON_AddArrayToObject(root,"aligned_trackers");for(const auto& entry:model.devices){cJSON* device=pose(Pose(model.result.world*entry.second.pose));cJSON_AddStringToObject(device,"name",entry.first.c_str());cJSON_AddStringToObject(device,"space","Quest STAGE");cJSON_AddItemToArray(aligned,device);}}
    cJSON_AddBoolToObject(root,"fast_mode",model.fastMode);cJSON_AddNumberToObject(root,"capture_target_samples",double(model.captureTarget()));cJSON_AddNumberToObject(root,"validation_target_samples",double(ValidationSamples));cJSON_AddNumberToObject(root,"sample_rate_hz",1/SampleInterval);
    cJSON_AddBoolToObject(root,"device_models_open",model.modelSettings);
    cJSON* identities=cJSON_AddArrayToObject(root,"device_models");
    const auto identity=[&](const std::string& device,const device_models::Identity& id,const std::string& key,const std::string& choice){cJSON* item=cJSON_CreateObject();cJSON_AddStringToObject(item,"device",device.c_str());cJSON_AddStringToObject(item,"model_id",id.model.c_str());cJSON_AddStringToObject(item,"name",id.name.c_str());cJSON_AddStringToObject(item,"identity_source",id.source.c_str());cJSON_AddStringToObject(item,"reported",id.reported.c_str());cJSON_AddStringToObject(item,"selection",choice.c_str());cJSON_AddStringToObject(item,"asset_key",key.c_str());const auto* asset=model.models.find(key);cJSON_AddBoolToObject(item,"asset_loaded",asset&&asset->loaded());cJSON_AddStringToObject(item,"asset_error",asset ? (asset->loaded() ? "headless: mesh verified, textures not decoded (no renderer)":asset->error.c_str()):"No matching model");cJSON_AddBoolToObject(item,"generic_fallback",!asset||!asset->loaded());cJSON_AddItemToArray(identities,item);};
    for(int side=0;side<2;++side)identity(side ? "right_controller":"left_controller",model.controllerIdentity[side],model.controllerKey(side),model.controllerOverride[side]);
    for(const auto& entry:model.devices)identity(entry.first,entry.second.identity,entry.second.identity.model,"auto");
    cJSON* actions=cJSON_AddObjectToObject(root,"actions");for(const char* action: {"select_reference","select_tracker","select_role","select_left_model","select_right_model","show_device_models","set_fast_mode","identify_reference","lock_map","start_capture","fit","start_validation","stop_validation","save_profile","load_profile","reset","start_service","place_panel","exit"})cJSON_AddBoolToObject(actions,action,model.can(action));
    cJSON* render=cJSON_AddObjectToObject(root,"render");cJSON_AddNumberToObject(render,"export_requested",0);cJSON_AddNumberToObject(render,"export_completed",0);cJSON_AddStringToObject(render,"last_export","");cJSON_AddStringToObject(render,"models","headless Linux: no renderer; device model assets verified offline");
    return inspectionJson(root);
}

int main(int argc,char** argv) {
    CalibrationInspection inspection;
    bool preview=false;double stopAfter=0;
    try {
        signal(SIGINT,handleSignal);signal(SIGTERM,handleSignal);
        // Match the Windows app: run relative to the executable's directory.
        std::error_code code;fs::current_path(fs::canonical("/proc/self/exe",code).parent_path(),code);
        fs::create_directories("data");
        for(int i=1;i<argc;++i){const std::string arg=argv[i];if(arg=="--preview")preview=true;else if(arg=="--seconds"&&i+1<argc){stopAfter=std::stod(argv[++i]);boundedRun=true;}}
        if(!preview)
            throw std::runtime_error("Hardware mode requires the VDXR OpenXR runtime (virtualdesktop-openxr.dll), which is Windows-only. On Linux, run with --preview. Quest connectivity on Linux is a separate follow-up (ALVR/WiVRN runtime integration).");
        Model model;model.preview=true;
        {std::ifstream file("data/calibration-settings.json");const std::string json((std::istreambuf_iterator<char>(file)),{});cJSON* settings=cJSON_Parse(json.c_str());const auto* value=cJSON_GetObjectItemCaseSensitive(settings,"fast_mode");if(cJSON_IsBool(value))model.fastMode=cJSON_IsTrue(value);cJSON_Delete(settings);}
        model.models.init();
        {std::ifstream file("data/device-models.json");const std::string json((std::istreambuf_iterator<char>(file)),{});cJSON* saved=cJSON_Parse(json.c_str());for(int side=0;side<2;++side){const auto* value=cJSON_GetObjectItemCaseSensitive(saved,side ? "right":"left");if(cJSON_IsString(value)&&device_models::controllerChoice(value->valuestring))model.controllerOverride[side]=value->valuestring;}cJSON_Delete(saved);}
        model.systemName="Quest 3 (synthetic preview)";model.interactionProfiles[0]=model.interactionProfiles[1]="/interaction_profiles/oculus/touch_controller";
        model.updateIdentities();model.message="Design preview. Calibration and saving are disabled.";
        fs::create_directories("data/inspection");
        const auto started=Clock::now();auto previous=started;
        uint64_t frameNumber=0;std::string lastLoggedState;
        while(!exitRequested) {
            inspection.process([&](const std::string& input){
                cJSON* request=cJSON_Parse(input.c_str());cJSON* operation=cJSON_GetObjectItemCaseSensitive(request,"op");cJSON* reply=cJSON_CreateObject();bool ok=false;
                if(cJSON_IsString(operation)) {
                    const std::string op=operation->valuestring;
                    if(op=="export_frame"){cJSON_AddStringToObject(reply,"error","Frame export requires the Windows D3D renderer; the headless Linux preview does not render frames.");}
                    else {
                        const cJSON* value=cJSON_GetObjectItemCaseSensitive(request,"value");const int index=cJSON_IsNumber(value) ? value->valueint:0;const std::string text=cJSON_IsString(value) ? value->valuestring:"";
                        if(op!="set_fast_mode"||(cJSON_IsNumber(value)&&(value->valuedouble==0||value->valuedouble==1)))ok=model.perform(op,index,text);
                    }
                }
                inspection.update(snapshot(model,frameNumber));
                cJSON_Delete(request);cJSON_AddBoolToObject(reply,"ok",ok);cJSON_AddNumberToObject(reply,"frame",double(frameNumber));if(!ok&&!cJSON_HasObjectItem(reply,"error"))cJSON_AddStringToObject(reply,"error","Action unavailable or invalid parameters; inspect state.actions");return inspectionJson(reply);
            });
            const auto now=Clock::now();const double elapsed=std::chrono::duration<double>(now-started).count();
            if(stopAfter>0&&elapsed>stopAfter)break;
            const double delta=std::chrono::duration<double>(now-previous).count();previous=now;
            if(delta<.005)std::this_thread::sleep_for(std::chrono::milliseconds(5));
            // Same synthetic fixtures as the Windows preview path.
            model.head={Pose::Identity(),15};model.head.pose.translation()=Eigen::Vector3d(0,1.55,0);
            for(int side=0;side<2;++side){model.grips[side]={Pose::Identity(),15};model.grips[side].pose.translation()=Eigen::Vector3d(side ? .25:-.25,1.05,-.5);model.grips[side].pose.linear()=Eigen::AngleAxisd(-.4,Eigen::Vector3d::UnitX()).toRotationMatrix();}
            RawPose r;r.pose=Pose::Identity();r.pose.translation()=Eigen::Vector3d(.15,1,-.65);r.received=Clock::now();r.sampleTime=1;r.identity={"vive-tracker-3","Vive Tracker 3.0","synthetic preview fixture",""};model.devices["PREVIEW TRACKER"]=r;model.selected="PREVIEW TRACKER";
            model.tick();
            ++frameNumber;const auto state=snapshot(model,frameNumber);inspection.update(state);
            const std::string transition=std::string(model.phase())+":"+model.message;
            if(transition!=lastLoggedState){try{atomicWrite("data/inspection/session-state.json",state);std::ofstream events("data/inspection/events.jsonl",std::ios::app);events<<state<<'\n';lastLoggedState=transition;}catch(...){} }
        }
        return 0;
    }catch(const std::exception& error){
        startupError=error.what();
        try{fs::create_directories("data/inspection");cJSON* failure=cJSON_CreateObject();cJSON_AddNumberToObject(failure,"schema",1);cJSON_AddNumberToObject(failure,"pid",double(getpid()));cJSON_AddStringToObject(failure,"phase","error");cJSON_AddStringToObject(failure,"error",error.what());cJSON* actions=cJSON_AddObjectToObject(failure,"actions");cJSON_AddBoolToObject(actions,"exit",true);const auto json=inspectionJson(failure);inspection.update(json);atomicWrite("data/inspection/error.json",json);}catch(...){}
        const auto started=Clock::now();
        while(!boundedRun&&!exitRequested) {
            inspection.process([](const std::string& input){cJSON* request=cJSON_Parse(input.c_str());cJSON* op=cJSON_GetObjectItemCaseSensitive(request,"op");const bool exit=cJSON_IsString(op)&&std::string(op->valuestring)=="exit";cJSON_Delete(request);if(exit)exitRequested=true;return exit ? "{\"ok\":true}":"{\"ok\":false,\"error\":\"startup failed; only state and exit are available\"}";});
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            if(stopAfter>0&&std::chrono::duration<double>(Clock::now()-started).count()>stopAfter)exitRequested=true;
        }
        return 1;
    }
}
