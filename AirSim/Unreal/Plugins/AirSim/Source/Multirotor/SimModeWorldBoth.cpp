#include "SimModeWorldBoth.h"
#include "ConstructorHelpers.h"
#include "Logging/MessageLog.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"

#include "AirBlueprintLib.h"
#include "vehicles/multirotor/controllers/DroneControllerBase.hpp"
#include "physics/PhysicsBody.hpp"
#include "common/ClockFactory.hpp"
#include <memory>
#include "vehicles/multirotor/MultiRotorParamsFactory.hpp"
#include "UnrealSensors/UnrealSensorFactory.h"

#ifndef AIRLIB_NO_RPC

#pragma warning(disable:4005) //warning C4005: 'TEXT': macro redefinition

#if defined _WIN32 || defined _WIN64
#include "AllowWindowsPlatformTypes.h"
#endif
#include "vehicles/multirotor/api/MultirotorRpcLibServer.hpp"
#include "vehicles/car/api/CarRpcLibServer.hpp"
#if defined _WIN32 || defined _WIN64
#include "HideWindowsPlatformTypes.h"
#endif

#endif


ASimModeWorldBoth::ASimModeWorldBoth()
{
    static ConstructorHelpers::FClassFinder<APIPCamera> external_camera_class(TEXT("Blueprint'/AirSim/Blueprints/BP_PIPCamera'"));
    external_camera_class_ = external_camera_class.Succeeded() ? external_camera_class.Class : nullptr;
    static ConstructorHelpers::FClassFinder<ACameraDirector> camera_director_class(TEXT("Blueprint'/AirSim/Blueprints/BP_CameraDirector'"));
    camera_director_class_ = camera_director_class.Succeeded() ? camera_director_class.Class : nullptr;

    follow_distance_ = -800;
}

void ASimModeWorldBoth::BeginPlay()
{
    Super::BeginPlay();

    report_wrapper_.initialize(false);
    report_wrapper_.reset();
}


std::unique_ptr<msr::airlib::ApiServerBase> ASimModeWorldBoth::createApiServer() const
{
#ifdef AIRLIB_NO_RPC
    return ASimModeBase::createApiServer();
#else
    return std::unique_ptr<msr::airlib::ApiServerBase>(new msr::airlib::MultirotorRpcLibServer(
        getSimModeApi(), getSettings().api_server_address));
#endif
}

void ASimModeWorldBoth::createApiServers(std::vector<std::unique_ptr<msr::airlib::ApiServerBase>>* api_servers)
{
    std::map<uint16_t, VehiclePawnWrapper*> fpv_vehicle_pawn_wrapper_port_map = getVehiclePawnWrapperPortMap();
    std::map<uint16_t, VehiclePawnWrapper*>::const_iterator it = fpv_vehicle_pawn_wrapper_port_map.begin();
    while(it != fpv_vehicle_pawn_wrapper_port_map.end())
    {
        uint16_t port = it->first;
#ifdef AIRLIB_NO_RPC
        api_servers->push_back(ASimModeBase::createApiServer());
#else
        std::map<uint16_t, std::string>::const_iterator it_pawn = port_pawn_map_.find(port);
        if (it_pawn->second == "Car")
            api_servers->push_back(std::unique_ptr<msr::airlib::ApiServerBase>(new msr::airlib::CarRpcLibServer(
                getSimModeApi(), getSettings().api_server_address, port)));
        else
            api_servers->push_back(std::unique_ptr<msr::airlib::ApiServerBase>(new msr::airlib::MultirotorRpcLibServer(
                getSimModeApi(), getSettings().api_server_address, port)));
#endif
        it++;
    }
}

void ASimModeWorldBoth::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    //stop physics thread before we dismental
    stopAsyncUpdator();

    //for (AActor* actor : spawned_actors_) {
    //    actor->Destroy();
    //}
    spawned_actors_.Empty();
    //fpv_vehicle_connectors_.Empty();
    CameraDirector = nullptr;

    Super::EndPlay(EndPlayReason);
}

VehiclePawnWrapper* ASimModeWorldBoth::getFpvVehiclePawnWrapper() const
{
    return fpv_vehicle_pawn_wrapper_;
}

void ASimModeWorldBoth::setupVehiclesAndCamera(std::vector<VehiclePtr>& vehicles)
{
    //get player controller
    APlayerController* player_controller = this->GetWorld()->GetFirstPlayerController();
    FTransform actor_transform = player_controller->GetViewTarget()->GetActorTransform();
    //put camera little bit above vehicle
    FTransform camera_transform(actor_transform.GetLocation() + FVector(-300, 0, 200));

    //we will either find external camera if it already exist in evironment or create one
    APIPCamera* external_camera;

    //find all vehicle pawns - multirotors
    {
        TArray<AActor*> pawns;
        UAirBlueprintLib::FindAllActor<TMultiRotorPawn>(this, pawns);
        FTransform multirotor_transform(actor_transform.GetLocation() + FVector(1000, 1000 , 0));

        //if no vehicle pawns exists in environment
        if (pawns.Num() == 0) {
            auto vehicle_bp_class = UAirBlueprintLib::LoadClass(
                getSettings().pawn_paths.at("DefaultQuadrotor").pawn_bp);

            //create vehicle pawn
            FActorSpawnParameters pawn_spawn_params;
            pawn_spawn_params.SpawnCollisionHandlingOverride =
                ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;
            TMultiRotorPawn* spawned_pawn = this->GetWorld()->SpawnActor<TMultiRotorPawn>(
                vehicle_bp_class, multirotor_transform, pawn_spawn_params);

            spawned_actors_.Add(spawned_pawn);
            pawns.Add(spawned_pawn);
        }
        //set up vehicle pawns
        int indx = 0;
        for (AActor* pawn : pawns)
        {
            uint16_t port = 41451 + indx++;
            //initialize each vehicle pawn we found
            TMultiRotorPawn* vehicle_pawn = static_cast<TMultiRotorPawn*>(pawn);
            vehicle_pawn->initializeForBeginPlay(getSettings().additional_camera_settings);

            //chose first pawn as FPV if none is designated as FPV
            VehiclePawnWrapper* wrapper = vehicle_pawn->getVehiclePawnWrapper();
            port_pawn_map_.insert(std::make_pair(port, "Multirotor"));
            addVehicle(wrapper, port);

            if (getSettings().enable_collision_passthrough)
                wrapper->getConfig().enable_passthrough_on_collisions = true;
            // if (wrapper->getConfig().is_fpv_vehicle || fpv_vehicle_pawn_wrapper_ == nullptr)
            //     fpv_vehicle_pawn_wrapper_ = wrapper;

            //now create the connector for each pawn
            VehiclePtr vehicle = createVehicle(wrapper);
            if (vehicle != nullptr) {
                vehicles.push_back(vehicle);
                fpv_vehicle_connectors_.Add(vehicle);
            }
            //else we don't have vehicle for this pawn
        }
    }


    //find all vehicle pawns - cars
    {
        TArray<AActor*> pawns;
        UAirBlueprintLib::FindAllActor<TVehiclePawn>(this, pawns);
        FTransform car_transform(actor_transform.GetLocation() + FVector(0, 0, 0));

        //if no vehicle pawns exists in environment
        if (pawns.Num() == 0) {
            //create vehicle pawn
            FActorSpawnParameters pawn_spawn_params;
            pawn_spawn_params.SpawnCollisionHandlingOverride =
                ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;

            auto vehicle_bp_class = UAirBlueprintLib::LoadClass(
                getSettings().pawn_paths.at("DefaultCar").pawn_bp);

            TVehiclePawn* spawned_pawn = this->GetWorld()->SpawnActor<TVehiclePawn>(
                vehicle_bp_class, car_transform, pawn_spawn_params);

            spawned_actors_.Add(spawned_pawn);
            pawns.Add(spawned_pawn);
        }
        //set up vehicle pawns
        int indx = 0;
        for (AActor* pawn : pawns)
        {
            uint16_t port = 42451 + indx++;
            //initialize each vehicle pawn we found
            TVehiclePawn* vehicle_pawn = static_cast<TVehiclePawn*>(pawn);
            vehicles_car_.push_back(vehicle_pawn);

            //chose first pawn as FPV if none is designated as FPV
            VehiclePawnWrapper* wrapper = vehicle_pawn->getVehiclePawnWrapper();
            port_pawn_map_.insert(std::make_pair(port, "Car"));
            addVehicle(wrapper, port);

            vehicle_pawn->initializeForBeginPlay(getSettings().engine_sound);

            if (getSettings().enable_collision_passthrough)
                wrapper->getConfig().enable_passthrough_on_collisions = true;
            if (wrapper->getConfig().is_fpv_vehicle || fpv_vehicle_pawn_wrapper_ == nullptr)
                fpv_vehicle_pawn_wrapper_ = wrapper;
        }

    }

    //find all BP camera directors in the environment
    {
        TArray<AActor*> camera_dirs;
        UAirBlueprintLib::FindAllActor<ACameraDirector>(this, camera_dirs);
        if (camera_dirs.Num() == 0) {
            //create director
            FActorSpawnParameters camera_spawn_params;
            camera_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;
            CameraDirector = this->GetWorld()->SpawnActor<ACameraDirector>(camera_director_class_, camera_transform, camera_spawn_params);
            // CameraDirector->setFollowDistance(225);
            // CameraDirector->setCameraRotationLagEnabled(false);
            // CameraDirector->setFpvCameraIndex(0);
            // CameraDirector->enableFlyWithMeMode();
            CameraDirector->setFollowDistance(follow_distance_);
            CameraDirector->setCameraRotationLagEnabled(true);
            CameraDirector->setFpvCameraIndex(3);
            spawned_actors_.Add(CameraDirector);

            //create external camera required for the director
            external_camera = this->GetWorld()->SpawnActor<APIPCamera>(external_camera_class_, camera_transform, camera_spawn_params);
            spawned_actors_.Add(external_camera);
        }
        else {
            CameraDirector = static_cast<ACameraDirector*>(camera_dirs[0]);
            external_camera = CameraDirector->getExternalCamera();
        }
    }

    fpv_vehicle_pawn_wrapper_->possess();
    CameraDirector->initializeForBeginPlay(getInitialViewMode(), fpv_vehicle_pawn_wrapper_, external_camera);
}


void ASimModeWorldBoth::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);

    getFpvVehiclePawnWrapper()->setLogLine(getLogString());

    if (pause_period_start_ > 0) {
        if (ClockFactory::get()->elapsedSince(pause_period_start_) >= pause_period_) {
            if (!isPaused())
                pause(true);

            pause_period_start_ = 0;
        }
    }

    report_wrapper_.update();
    report_wrapper_.setEnable(EnableReport);

    if (report_wrapper_.canReport()) {
        report_wrapper_.clearReport();
        updateReport();
    }
}

std::string ASimModeWorldBoth::getLogString() const
{
    const msr::airlib::Kinematics::State* kinematics = getFpvVehiclePawnWrapper()->getTrueKinematics();
    uint64_t timestamp_millis = static_cast<uint64_t>(msr::airlib::ClockFactory::get()->nowNanos() / 1.0E6);

    //TODO: because this bug we are using alternative code with stringstream
    //https://answers.unrealengine.com/questions/664905/unreal-crashes-on-two-lines-of-extremely-simple-st.html

    std::string line;
    line.append(std::to_string(timestamp_millis)).append("\t")
        .append(std::to_string(kinematics->pose.position.x())).append("\t")
        .append(std::to_string(kinematics->pose.position.y())).append("\t")
        .append(std::to_string(kinematics->pose.position.z())).append("\t")
        .append(std::to_string(kinematics->pose.orientation.w())).append("\t")
        .append(std::to_string(kinematics->pose.orientation.x())).append("\t")
        .append(std::to_string(kinematics->pose.orientation.y())).append("\t")
        .append(std::to_string(kinematics->pose.orientation.z())).append("\t");

    return line;

    //std::stringstream ss;
    //ss << timestamp_millis << "\t";
    //ss << kinematics.pose.position.x() << "\t" << kinematics.pose.position.y() << "\t" << kinematics.pose.position.z() << "\t";
    //ss << kinematics.pose.orientation.w() << "\t" << kinematics.pose.orientation.x() << "\t" << kinematics.pose.orientation.y() << "\t" << kinematics.pose.orientation.z() << "\t";
    //ss << "\n";
    //return ss.str();
}

void ASimModeWorldBoth::createVehicles(std::vector<VehiclePtr>& vehicles)
{
    //find vehicles and cameras available in environment
    //if none available then we will create one
    setupVehiclesAndCamera(vehicles);
}

ASimModeWorldBase::VehiclePtr ASimModeWorldBoth::createVehicle(VehiclePawnWrapper* wrapper)
{
    std::shared_ptr<UnrealSensorFactory> sensor_factory = std::make_shared<UnrealSensorFactory>(wrapper->getPawn(), &wrapper->getNedTransform());
    auto vehicle_params = MultiRotorParamsFactory::createConfig(wrapper->getVehicleConfigName(), sensor_factory);

    vehicle_params_.push_back(std::move(vehicle_params));

    std::shared_ptr<MultiRotorConnector> vehicle = std::make_shared<MultiRotorConnector>(
        wrapper, vehicle_params_.back().get(), manual_pose_controller);

    if (vehicle->getPhysicsBody() != nullptr)
        wrapper->setKinematics(&(static_cast<PhysicsBody*>(vehicle->getPhysicsBody())->getKinematics()));

    return std::static_pointer_cast<VehicleConnectorBase>(vehicle);
}

void ASimModeWorldBoth::setupClockSpeed()
{
    float clock_speed = getSettings().clock_speed;

    //setup clock in ClockFactory
    std::string clock_type = getSettings().clock_type;

    if (clock_type == "ScalableClock") {
        //scalable clock returns interval same as wall clock but multiplied by a scale factor
        ClockFactory::get(std::make_shared<msr::airlib::ScalableClock>(clock_speed == 1 ? 1 : 1 / clock_speed));
    }
    else if (clock_type == "SteppableClock") {
        //steppable clock returns interval that is a constant number irrespective of wall clock
        //we can either multiply this fixed interval by scale factor to speed up/down the clock
        //but that would cause vehicles like quadrotors to become unstable
        //so alternative we use here is instead to scale control loop frequency. The downside is that
        //depending on compute power available, we will max out control loop frequency and therefore can no longer
        //get increase in clock speed

        //Approach 1: scale clock period, no longer used now due to quadrotor unstability
        //ClockFactory::get(std::make_shared<msr::airlib::SteppableClock>(
        //static_cast<msr::airlib::TTimeDelta>(getPhysicsLoopPeriod() * 1E-9 * clock_speed)));

        //Approach 2: scale control loop frequency if clock is speeded up
        if (clock_speed >= 1) {
            ClockFactory::get(std::make_shared<msr::airlib::SteppableClock>(
                static_cast<msr::airlib::TTimeDelta>(getPhysicsLoopPeriod() * 1E-9))); //no clock_speed multiplier

            setPhysicsLoopPeriod(getPhysicsLoopPeriod() / static_cast<long long>(clock_speed));
        }
        else {
            //for slowing down, this don't generate instability
            ClockFactory::get(std::make_shared<msr::airlib::SteppableClock>(
                static_cast<msr::airlib::TTimeDelta>(getPhysicsLoopPeriod() * 1E-9 * clock_speed)));
        }
    }
    else
        throw std::invalid_argument(common_utils::Utils::stringf(
            "clock_type %s is not recognized", clock_type.c_str()));

    current_clockspeed_ = clock_speed;
    //setup clock in PhysX
    UAirBlueprintLib::setUnrealClockSpeed(this, current_clockspeed_);
    UAirBlueprintLib::LogMessageString("Clock Speed: ", std::to_string(current_clockspeed_), LogDebugLevel::Informational);
}

void ASimModeWorldBoth::initializePauseState()
{
    pause_period_ = 0;
    pause_period_start_ = 0;
    pause(false);
}

bool ASimModeWorldBoth::isPaused() const
{
    return current_clockspeed_ == 0;
}

void ASimModeWorldBoth::pause(bool is_paused)
{
    if (is_paused)
        current_clockspeed_ = 0;
    else
        current_clockspeed_ = getSettings().clock_speed;

    UAirBlueprintLib::setUnrealClockSpeed(this, current_clockspeed_);
}

void ASimModeWorldBoth::continueForTime(double seconds)
{
    pause_period_start_ = ClockFactory::get()->nowNanos();
    pause_period_ = seconds;
    pause(false);
}

int ASimModeWorldBoth::getRemoteControlID(const VehiclePawnWrapper& pawn) const
{
    msr::airlib::Settings settings;
    fpv_vehicle_pawn_wrapper_->getRawVehicleSettings(settings);

    msr::airlib::Settings rc_settings;
    settings.getChild("RC", rc_settings);
    return rc_settings.getInt("RemoteControlID", -1);
}

void ASimModeWorldBoth::reset()
{
    std::map<uint16_t, VehiclePawnWrapper*> fpv_vehicle_pawn_wrapper_port_map = getVehiclePawnWrapperPortMap();
    std::map<uint16_t, VehiclePawnWrapper*>::const_iterator it = fpv_vehicle_pawn_wrapper_port_map.begin();
    while(it != fpv_vehicle_pawn_wrapper_port_map.end())
    {
        std::map<uint16_t, std::string>::const_iterator it_pawn = port_pawn_map_.find(it->first);
        if (it_pawn->second == "Car")
        {
            VehiclePawnWrapper* wrapper = it->second;
            msr::airlib::VehicleApiBase* api = wrapper->getApi();
            if (api) {
                UAirBlueprintLib::RunCommandOnGameThread([api]() {
                    api->reset();
                }, true);
            }
        }
        it++;
    }
    Super::reset();
}

void ASimModeWorldBoth::updateReport()
{
    for (ACarPawn* vehicle : vehicles_car_) {
        VehiclePawnWrapper* wrapper = vehicle->getVehiclePawnWrapper();
        msr::airlib::StateReporter& reporter = *report_wrapper_.getReporter();
        std::string vehicle_name = fpv_vehicle_pawn_wrapper_->getVehicleConfigName();

        reporter.writeHeading(std::string("Vehicle: ").append(
            vehicle_name == "" ? "(default)" : vehicle_name));

        const msr::airlib::Kinematics::State* kinematics = wrapper->getTrueKinematics();

        reporter.writeValue("Position", kinematics->pose.position);
        reporter.writeValue("Orientation", kinematics->pose.orientation);
        reporter.writeValue("Lin-Vel", kinematics->twist.linear);
        reporter.writeValue("Lin-Accl", kinematics->accelerations.linear);
        reporter.writeValue("Ang-Vel", kinematics->twist.angular);
        reporter.writeValue("Ang-Accl", kinematics->accelerations.angular);
    }
}

std::string ASimModeWorldBoth::getReport()
{
    return report_wrapper_.getOutput();
}
