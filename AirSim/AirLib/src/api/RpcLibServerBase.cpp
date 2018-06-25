// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

//in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY
//RPC code requires C++14. If build system like Unreal doesn't support it then use compiled binaries
#ifndef AIRLIB_NO_RPC
//if using Unreal Build system then include precompiled header file first

#include "api/RpcLibServerBase.hpp"


#include "common/Common.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "common/common_utils/MinWinDefines.hpp"
#undef NOUSER
//TODO: HACK: UE4 defines macro with stupid names like "check" that conflicts with msgpack library
//#undef check
#include "rpc/server.h"
#include "api/RpcLibAdapatorsBase.hpp"
//TODO: HACK: UE4 defines macro with stupid names like "check" that conflicts with msgpack library
#define check(expr) (static_cast<void>((expr)))
STRICT_MODE_ON


namespace msr { namespace airlib {

struct RpcLibServerBase::impl {
    impl(string server_address, uint16_t port)
        : server(server_address, port)
    {}

    impl(uint16_t port)
        : server(port)
    {}

    ~impl() {
    }

    rpc::server server;
};

typedef msr::airlib_rpclib::RpcLibAdapatorsBase RpcLibAdapatorsBase;

RpcLibServerBase::RpcLibServerBase(SimModeApiBase* simmode_api, string server_address, uint16_t port)
    : simmode_api_(simmode_api), port_(port)
{
    if (server_address == "")
        pimpl_.reset(new impl(port));
    else
        pimpl_.reset(new impl(server_address, port));
    pimpl_->server.bind("ping", [&]() -> bool { return true; });


    //sim only
    pimpl_->server.bind("simGetImages", [&](const std::vector<RpcLibAdapatorsBase::ImageRequest>& request_adapter) -> vector<RpcLibAdapatorsBase::ImageResponse> {
        const auto& response = getVehicleApi()->simGetImages(RpcLibAdapatorsBase::ImageRequest::to(request_adapter));
        return RpcLibAdapatorsBase::ImageResponse::from(response);
    });
    pimpl_->server.bind("simGetImage", [&](uint8_t camera_id, ImageCaptureBase::ImageType type) -> vector<uint8_t> {
        auto result = getVehicleApi()->simGetImage(camera_id, type);
        if (result.size() == 0) {
            // rpclib has a bug with serializing empty vectors, so we return a 1 byte vector instead.
            result.push_back(0);
        }
        return result;
    });

    pimpl_->server.
        bind("simSetPose", [&](const RpcLibAdapatorsBase::Pose &pose, bool ignore_collision) -> void {
        getVehicleApi()->simSetPose(pose.to(), ignore_collision);
    });
    pimpl_->server.bind("simGetPose", [&]() -> RpcLibAdapatorsBase::Pose {
        const auto& pose = getVehicleApi()->simGetPose();
        return RpcLibAdapatorsBase::Pose(pose);
    });

    pimpl_->server.
        bind("simSetSegmentationObjectID", [&](const std::string& mesh_name, int object_id, bool is_name_regex) -> bool {
        return getVehicleApi()->simSetSegmentationObjectID(mesh_name, object_id, is_name_regex);
    });
    pimpl_->server.
        bind("simGetSegmentationObjectID", [&](const std::string& mesh_name) -> int {
        return getVehicleApi()->simGetSegmentationObjectID(mesh_name);
    });

    pimpl_->server.bind("reset", [&]() -> void {
        getSimModeApi()->reset();
    });

    pimpl_->server.bind("simPrintLogMessage", [&](const std::string& message, const std::string& message_param, unsigned char severity) -> void {
        getVehicleApi()->simPrintLogMessage(message, message_param, severity);
    });

    pimpl_->server.bind("getHomeGeoPoint", [&]() -> RpcLibAdapatorsBase::GeoPoint {
        const auto& geo_point = getVehicleApi()->getHomeGeoPoint();
        return RpcLibAdapatorsBase::GeoPoint(geo_point);
    });

    pimpl_->server.bind("getCameraInfo", [&](int camera_id) -> RpcLibAdapatorsBase::CameraInfo {
        const auto& camera_info = getVehicleApi()->getCameraInfo(camera_id);
        return RpcLibAdapatorsBase::CameraInfo(camera_info);
    });

    pimpl_->server.bind("setCameraOrientation", [&](int camera_id, const RpcLibAdapatorsBase::Quaternionr& orientation) -> void {
        getVehicleApi()->setCameraOrientation(camera_id, orientation.to());
    });

    pimpl_->server.bind("enableApiControl", [&](bool is_enabled) -> void { getVehicleApi()->enableApiControl(is_enabled); });
    pimpl_->server.bind("isApiControlEnabled", [&]() -> bool { return getVehicleApi()->isApiControlEnabled(); });
    pimpl_->server.bind("armDisarm", [&](bool arm) -> bool { return getVehicleApi()->armDisarm(arm); });
    pimpl_->server.bind("getCollisionInfo", [&]() -> RpcLibAdapatorsBase::CollisionInfo {
        const auto& collision_info = getVehicleApi()->getCollisionInfo();
        return RpcLibAdapatorsBase::CollisionInfo(collision_info);
    });

    pimpl_->server.bind("simGetObjectPose", [&](const std::string& object_name) -> RpcLibAdapatorsBase::Pose {
        const auto& pose = getVehicleApi()->simGetObjectPose(object_name);
        return RpcLibAdapatorsBase::Pose(pose);
    });

    pimpl_->server.bind("simPause", [&](bool is_paused) -> void {
        getSimModeApi()->pause(is_paused);
    });
    pimpl_->server.bind("simIsPaused", [&]() -> bool {
        return getSimModeApi()->isPaused();
    });
    pimpl_->server.bind("simContinueForTime", [&](double seconds) -> void {
        getSimModeApi()->continueForTime(seconds);
    });

    pimpl_->server.suppress_exceptions(true);
}

//required for pimpl
RpcLibServerBase::~RpcLibServerBase()
{
    stop();
}

void RpcLibServerBase::start(bool block)
{
    if (block)
        pimpl_->server.run();
    else
        pimpl_->server.async_run(4);   //4 threads
}

void RpcLibServerBase::stop()
{
    pimpl_->server.stop();
}

uint16_t RpcLibServerBase::getServerPort() const
{
    return port_;
}

VehicleApiBase* RpcLibServerBase::getVehicleApi() const
{
    return simmode_api_->getVehicleApi(getServerPort());
}

void* RpcLibServerBase::getServer() const
{
    return &pimpl_->server;
}

SimModeApiBase* RpcLibServerBase::getSimModeApi() const
{
    return simmode_api_;
}


}} //namespace
#endif
#endif
