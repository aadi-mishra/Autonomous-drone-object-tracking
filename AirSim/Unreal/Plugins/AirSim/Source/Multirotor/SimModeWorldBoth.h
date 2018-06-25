#pragma once

#include "CoreMinimal.h"

#include "FlyingPawn.h"
#include "common/Common.hpp"
#include "MultiRotorConnector.h"
#include "vehicles/multirotor/MultiRotorParams.hpp"
#include "SimMode/SimModeWorldBase.h"
#include "VehiclePawnWrapper.h"
#include "common/StateReporterWrapper.hpp"
#include "SimModeWorldBoth.generated.h"


UCLASS()
class AIRSIM_API ASimModeWorldBoth : public ASimModeWorldBase
{
    GENERATED_BODY()

public:
    ASimModeWorldBoth();
    virtual void BeginPlay() override;

    virtual void Tick( float DeltaSeconds ) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    VehiclePawnWrapper* getFpvVehiclePawnWrapper() const override;
    std::string getLogString() const;
    virtual std::unique_ptr<msr::airlib::ApiServerBase> createApiServer() const override;
    virtual void createApiServers(std::vector<std::unique_ptr<msr::airlib::ApiServerBase>>* api_servers) override;

    virtual void reset() override;
    virtual std::string getReport() override;
    virtual bool isPaused() const override;
    virtual void pause(bool is_paused) override;
    virtual void continueForTime(double seconds) override;


protected:
    typedef AFlyingPawn TMultiRotorPawn;
    typedef ACarPawn TVehiclePawn;

    virtual void createVehicles(std::vector<VehiclePtr>& vehicles) override;
    VehiclePtr createVehicle(VehiclePawnWrapper* wrapper);
    virtual void setupClockSpeed() override;


private:
    void setupVehiclesAndCamera(std::vector<VehiclePtr>& vehicles);

    void updateReport();
    int getRemoteControlID(const VehiclePawnWrapper& pawn) const;
    void initializePauseState();

private:
    typedef msr::airlib::ClockFactory ClockFactory;
    typedef common_utils::Utils Utils;
    typedef msr::airlib::TTimePoint TTimePoint;
    typedef msr::airlib::TTimeDelta TTimeDelta;

    TArray<uint8> image_;
    std::vector <std::unique_ptr<msr::airlib::MultiRotorParams> > vehicle_params_;

    UClass* external_camera_class_;
    UClass* camera_director_class_;

    TArray<AActor*> spawned_actors_;

    VehiclePawnWrapper* fpv_vehicle_pawn_wrapper_;
    TArray <std::shared_ptr<VehicleConnectorBase> > fpv_vehicle_connectors_;

    float follow_distance_;
    msr::airlib::StateReporterWrapper report_wrapper_;
    std::vector<ACarPawn*> vehicles_car_;

    std::atomic<float> current_clockspeed_;
    std::atomic<TTimeDelta> pause_period_;
    std::atomic<TTimePoint> pause_period_start_;

    std::map<uint16_t, std::string> port_pawn_map_;
};
