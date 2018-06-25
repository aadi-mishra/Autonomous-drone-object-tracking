#pragma once

#include "CoreMinimal.h"

#include "FlyingPawn.h"
#include "common/Common.hpp"
#include "MultiRotorConnector.h"
#include "vehicles/multirotor/MultiRotorParams.hpp"
#include "SimMode/SimModeWorldBase.h"
#include "VehiclePawnWrapper.h"
#include "SimModeWorldMultiRotor.generated.h"


UCLASS()
class AIRSIM_API ASimModeWorldMultiRotor : public ASimModeWorldBase
{
    GENERATED_BODY()

public:
    ASimModeWorldMultiRotor();
    virtual void BeginPlay() override;

    virtual void Tick( float DeltaSeconds ) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    VehiclePawnWrapper* getFpvVehiclePawnWrapper() const override;
    std::string getLogString() const;
    virtual std::unique_ptr<msr::airlib::ApiServerBase> createApiServer() const override;
    virtual void createApiServers(std::vector<std::unique_ptr<msr::airlib::ApiServerBase>>* api_servers) override;



protected:
    typedef AFlyingPawn TMultiRotorPawn;

    virtual void createVehicles(std::vector<VehiclePtr>& vehicles) override;
    VehiclePtr createVehicle(VehiclePawnWrapper* wrapper);
    virtual void setupClockSpeed() override;


private:
    void setupVehiclesAndCamera(std::vector<VehiclePtr>& vehicles);

private:
    typedef msr::airlib::ClockFactory ClockFactory;

    TArray<uint8> image_;
    std::vector <std::unique_ptr<msr::airlib::MultiRotorParams> > vehicle_params_;

    UClass* external_camera_class_;
    UClass* camera_director_class_;

    TArray<AActor*> spawned_actors_;

    VehiclePawnWrapper* fpv_vehicle_pawn_wrapper_;
    TArray <std::shared_ptr<VehicleConnectorBase> > fpv_vehicle_connectors_;
};
