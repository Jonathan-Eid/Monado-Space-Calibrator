// SPDX-License-Identifier: MIT
#pragma once
// Device identity rules shared by every platform. Extracted from
// device_models.h (which keeps the D3D11 model library and includes this) so
// headless/Linux builds can use the same identity logic without a renderer.
#include <cJSON.h>
#include <string>

namespace device_models {
struct Identity { std::string model="generic",name="Unknown device",source="unavailable",reported; };
inline bool controllerChoice(const std::string& value) {
    return value=="auto"||value=="generic"||value=="meta-quest-touch-plus"||value=="oculus-touch-v3"||value=="meta-quest-touch-pro";
}
inline const char* controllerName(const std::string& model) {
    if(model=="meta-quest-touch-plus")return "Quest 3 / Touch Plus";
    if(model=="oculus-touch-v3")return "Quest 2 / Touch";
    if(model=="meta-quest-touch-pro")return "Touch Pro";
    return "Generic controller";
}
inline Identity controller(const std::string& system,const std::string& profile,const std::string& choice) {
    Identity id;id.reported=profile;
    if(choice!="auto"){id.model=choice;id.source="manual override";}
    else if(profile=="/interaction_profiles/meta/touch_controller_plus") {id.model="meta-quest-touch-plus";id.source="interaction profile";}
    else if(profile=="/interaction_profiles/facebook/touch_controller_pro") {id.model="meta-quest-touch-pro";id.source="interaction profile";}
    else if(profile=="/interaction_profiles/oculus/touch_controller") {
        if(system.find("Quest 3")!=std::string::npos)id.model="meta-quest-touch-plus";
        else if(system.find("Quest 2")!=std::string::npos)id.model="oculus-touch-v3";
        else if(system.find("Quest Pro")!=std::string::npos)id.model="meta-quest-touch-pro";
        if(id.model!="generic")id.source="headset default (controller generation unreported)";
    }
    id.name=controllerName(id.model);return id;
}
inline Identity tracker(const char* config) {
    Identity id;id.name="Unknown tracker";
    cJSON* json=cJSON_Parse(config ? config:"");
    const auto string=[&](const char* key){const auto* p=cJSON_GetObjectItemCaseSensitive(json,key);return std::string(cJSON_IsString(p) ? p->valuestring:"");};
    const auto model=string("model_number"),render=string("render_model");id.reported=model;
    // Exact known factory metadata only; the LHR serial prefix does not identify a generation.
    if(model=="VIVE Tracker 3.0 MV"||model=="VIVE Tracker 3.0"||render=="{htc}vr_tracker_vive_3_0"||render=="{htc}/rendermodels/vr_tracker_vive_3_0") {
        id.model="vive-tracker-3";id.name="Vive Tracker 3.0";id.source="device factory configuration";
    }
    cJSON_Delete(json);return id;
}
}
