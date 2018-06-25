#include "SimHUD.h"
#include "ConstructorHelpers.h"
#include "Kismet/KismetSystemLibrary.h"
#include "Misc/FileHelper.h"

#include "Multirotor/SimModeWorldMultiRotor.h"
#include "Car/SimModeCar.h"
#include "Multirotor/SimModeWorldBoth.h"
#include "common/AirSimSettings.hpp"
#include "api/DebugApiServer.hpp"
#include <stdexcept>


ASimHUD* ASimHUD::instance_ = nullptr;

ASimHUD::ASimHUD()
{
    static ConstructorHelpers::FClassFinder<UUserWidget> hud_widget_class(TEXT("WidgetBlueprint'/AirSim/Blueprints/BP_SimHUDWidget'"));
    widget_class_ = hud_widget_class.Succeeded() ? hud_widget_class.Class : nullptr;
    instance_ = this;
}

void ASimHUD::BeginPlay()
{
    Super::BeginPlay();

    try {
        UAirBlueprintLib::OnBeginPlay();
        initializeSettings();
        setUnrealEngineSettings();
        createSimMode();
        createMainWidget();
        setupInputBindings();
        startApiServer();
    }
    catch (std::exception& ex) {
        UAirBlueprintLib::LogMessageString("Error at startup: ", ex.what(), LogDebugLevel::Failure);
        //FGenericPlatformMisc::PlatformInit();
        //FGenericPlatformMisc::MessageBoxExt(EAppMsgType::Ok, TEXT("Error at Startup"), ANSI_TO_TCHAR(ex.what()));
        UAirBlueprintLib::ShowMessage(EAppMsgType::Ok, std::string("Error at startup: ") + ex.what(), "Error");
    }
}

void ASimHUD::Tick(float DeltaSeconds)
{
    if (simmode_ && simmode_->EnableReport)
        widget_->updateReport(simmode_->getReport());
}

void ASimHUD::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    stopApiServer();

    if (widget_) {
        widget_->Destruct();
        widget_ = nullptr;
    }
    if (simmode_) {
        simmode_->Destroy();
        simmode_ = nullptr;
    }

    UAirBlueprintLib::OnEndPlay();

    Super::EndPlay(EndPlayReason);
}

void ASimHUD::toggleRecordHandler()
{
    simmode_->toggleRecording();
}

void ASimHUD::inputEventToggleRecording()
{
    toggleRecordHandler();
}

void ASimHUD::inputEventToggleReport()
{
    simmode_->EnableReport = !simmode_->EnableReport;
    widget_->setReportVisible(simmode_->EnableReport);
}

void ASimHUD::startApiServer()
{
    if (AirSimSettings::singleton().enable_rpc) {
        simmode_->createApiServers(&api_servers_);
        for (auto& api_server : api_servers_)
        {
#ifdef AIRLIB_NO_RPC
            api_server.reset(new msr::airlib::DebugApiServer());
#endif
            try {
                api_server->start();
            }
            catch (std::exception& ex) {
                UAirBlueprintLib::LogMessageString("Cannot start RpcLib Server", ex.what(), LogDebugLevel::Failure);
            }
        }
    }
    else
        UAirBlueprintLib::LogMessageString("API server is disabled in settings", "", LogDebugLevel::Informational);
}

void ASimHUD::stopApiServer()
{
    if (api_servers_.empty())
    {
        for (auto& api_server : api_servers_)
        {
            api_server->stop();
        }
    }
    api_servers_.clear();
}

bool ASimHUD::isApiServerStarted()
{
    return api_servers_.empty();
    // return api_server_ != nullptr;
}

void ASimHUD::inputEventToggleHelp()
{
    widget_->toggleHelpVisibility();
}

void ASimHUD::inputEventToggleTrace()
{
    simmode_->getFpvVehiclePawnWrapper()->toggleTrace();
}

ASimHUD::ImageType ASimHUD::getSubwindowCameraType(int window_index)
{
    //TODO: index check
    return getSubWindowSettings().at(window_index).image_type;
}

void ASimHUD::setSubwindowCameraType(int window_index, ImageType type)
{
    getSubWindowSettings().at(window_index).image_type = type;
    updateWidgetSubwindowVisibility();
}

APIPCamera* ASimHUD::getSubwindowCamera(int window_index)
{
    return subwindow_cameras_[window_index]; //TODO: index check
}

void ASimHUD::setSubwindowCamera(int window_index, APIPCamera* camera)
{
    subwindow_cameras_[window_index] = camera; //TODO: index check
    updateWidgetSubwindowVisibility();
}

bool ASimHUD::getSubwindowVisible(int window_index)
{
    return getSubWindowSettings().at(window_index).visible;
}

void ASimHUD::setSubwindowVisible(int window_index, bool is_visible)
{
    getSubWindowSettings().at(window_index).visible = is_visible;
    updateWidgetSubwindowVisibility();
}

void ASimHUD::updateWidgetSubwindowVisibility()
{
    for (int window_index = 0; window_index < AirSimSettings::kSubwindowCount; ++window_index) {
        APIPCamera* camera = subwindow_cameras_[window_index];
        ImageType camera_type = getSubWindowSettings().at(window_index).image_type;

        bool is_visible = getSubWindowSettings().at(window_index).visible && camera != nullptr;

        if (camera != nullptr)
            camera->setCameraTypeEnabled(camera_type, is_visible);

        widget_->setSubwindowVisibility(window_index,
            is_visible,
            is_visible ? camera->getRenderTarget(camera_type, false) : nullptr
        );
    }
}

bool ASimHUD::isWidgetSubwindowVisible(int window_index)
{
    return widget_->getSubwindowVisibility(window_index) != 0;
}

void ASimHUD::inputEventToggleSubwindow0()
{
    getSubWindowSettings().at(0).visible = !getSubWindowSettings().at(0).visible;
    updateWidgetSubwindowVisibility();
}

void ASimHUD::inputEventToggleSubwindow1()
{
    getSubWindowSettings().at(1).visible = !getSubWindowSettings().at(1).visible;
    updateWidgetSubwindowVisibility();
}

void ASimHUD::inputEventToggleSubwindow2()
{
    getSubWindowSettings().at(2).visible = !getSubWindowSettings().at(2).visible;
    updateWidgetSubwindowVisibility();
}

void ASimHUD::inputEventToggleAll()
{
    getSubWindowSettings().at(0).visible = !getSubWindowSettings().at(0).visible;
    getSubWindowSettings().at(1).visible = getSubWindowSettings().at(2).visible = getSubWindowSettings().at(0).visible;
    updateWidgetSubwindowVisibility();
}

void ASimHUD::createMainWidget()
{
    //create main widget
    if (widget_class_ != nullptr) {
        widget_ = CreateWidget<USimHUDWidget>(this->GetOwningPlayerController(), widget_class_);
    }
    else {
        widget_ = nullptr;
        UAirBlueprintLib::LogMessage(TEXT("Cannot instantiate BP_SimHUDWidget blueprint!"), TEXT(""), LogDebugLevel::Failure, 180);
    }

    initializeSubWindows();

    widget_->AddToViewport();

    //synchronize PIP views
    widget_->initializeForPlay();
    widget_->setReportVisible(simmode_->EnableReport);
    widget_->setOnToggleRecordingHandler(std::bind(&ASimHUD::toggleRecordHandler, this));
    widget_->setRecordButtonVisibility(simmode_->isRecordUIVisible());
    updateWidgetSubwindowVisibility();
}


void ASimHUD::setUnrealEngineSettings()
{
    //TODO: should we only do below on SceneCapture2D components and cameras?
    //avoid motion blur so capture images don't get
    GetWorld()->GetGameViewport()->GetEngineShowFlags()->SetMotionBlur(false);

    //use two different methods to set console var because sometime it doesn't seem to work
    static const auto custom_depth_var = IConsoleManager::Get().FindConsoleVariable(TEXT("r.CustomDepth"));
    custom_depth_var->Set(3);

    //Equivalent to enabling Custom Stencil in Project > Settings > Rendering > Postprocessing
    UKismetSystemLibrary::ExecuteConsoleCommand(GetWorld(), FString("r.CustomDepth 3"));

    //during startup we init stencil IDs to random hash and it takes long time for large environments
    //we get error that GameThread has timed out after 30 sec waiting on render thread
    static const auto render_timeout_var = IConsoleManager::Get().FindConsoleVariable(TEXT("g.TimeoutForBlockOnRenderFence"));
    render_timeout_var->Set(300000);
}

void ASimHUD::setupInputBindings()
{
    UAirBlueprintLib::EnableInput(this);

    UAirBlueprintLib::BindActionToKey("inputEventToggleRecording", EKeys::R, this, &ASimHUD::inputEventToggleRecording);
    UAirBlueprintLib::BindActionToKey("InputEventToggleReport", EKeys::Semicolon, this, &ASimHUD::inputEventToggleReport);
    UAirBlueprintLib::BindActionToKey("InputEventToggleHelp", EKeys::F1, this, &ASimHUD::inputEventToggleHelp);
    UAirBlueprintLib::BindActionToKey("InputEventToggleTrace", EKeys::T, this, &ASimHUD::inputEventToggleTrace);

    UAirBlueprintLib::BindActionToKey("InputEventToggleSubwindow0", EKeys::One, this, &ASimHUD::inputEventToggleSubwindow0);
    UAirBlueprintLib::BindActionToKey("InputEventToggleSubwindow1", EKeys::Two, this, &ASimHUD::inputEventToggleSubwindow1);
    UAirBlueprintLib::BindActionToKey("InputEventToggleSubwindow2", EKeys::Three, this, &ASimHUD::inputEventToggleSubwindow2);
    UAirBlueprintLib::BindActionToKey("InputEventToggleAll", EKeys::Zero, this, &ASimHUD::inputEventToggleAll);
}

void ASimHUD::initializeSettings()
{
    std::string settingsText;
    if (getSettingsText(settingsText))
        AirSimSettings::initializeSettings(settingsText);
    else
        AirSimSettings::createDefaultSettingsFile();

    int warning_count = AirSimSettings::singleton().load(std::bind(&ASimHUD::getSimModeFromUser, this));
    if (warning_count > 0) {
        for (const auto& warning : AirSimSettings::singleton().warning_messages) {
            UAirBlueprintLib::LogMessageString(warning, "", LogDebugLevel::Failure);
        }
    }
}

const std::vector<AirSimSettings::SubwindowSetting>& ASimHUD::getSubWindowSettings() const
{
    return AirSimSettings::singleton().subwindow_settings;
}

std::vector<AirSimSettings::SubwindowSetting>& ASimHUD::getSubWindowSettings()
{
    return AirSimSettings::singleton().subwindow_settings;
}

std::string ASimHUD::getSimModeFromUser()
{
    if (EAppReturnType::No == UAirBlueprintLib::ShowMessage(EAppMsgType::YesNo,
        "Would you like to use car simulation? Choose no to use quadrotor simulation.",
        "Choose Vehicle"))
    {
        if (EAppReturnType::Yes == UAirBlueprintLib::ShowMessage(EAppMsgType::YesNo,
            "Would you like to use car in quadrotor simulation? Choose no to simulate only quadrotor.",
            "Choose Vehicle"))
        {
            return "Both";
        }
        else
            return "Multirotor";
    }
    else
        return "Car";
}

void ASimHUD::createSimMode()
{
    std::string simmode_name = AirSimSettings::singleton().simmode_name;

    FActorSpawnParameters simmode_spawn_params;
    simmode_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;

    if (simmode_name == "Multirotor")
        simmode_ = this->GetWorld()->SpawnActor<ASimModeWorldMultiRotor>(FVector::ZeroVector, FRotator::ZeroRotator, simmode_spawn_params);
    else if (simmode_name == "Car")
        simmode_ = this->GetWorld()->SpawnActor<ASimModeCar>(FVector::ZeroVector, FRotator::ZeroRotator, simmode_spawn_params);
    else if (simmode_name == "Both")
        simmode_ = this->GetWorld()->SpawnActor<ASimModeWorldBoth>(FVector::ZeroVector, FRotator::ZeroRotator, simmode_spawn_params);
    else
        UAirBlueprintLib::LogMessageString("SimMode is not valid: ", simmode_name, LogDebugLevel::Failure);
}


void ASimHUD::initializeSubWindows()
{
    auto wrapper = simmode_->getFpvVehiclePawnWrapper();
    auto camera_count = wrapper->getCameraCount();

    //setup defaults
    if (camera_count > 0) {
        subwindow_cameras_[0] = wrapper->getCamera(0);
        subwindow_cameras_[1] = wrapper->getCamera(0); //camera_count > 3 ? 3 : 0
        subwindow_cameras_[2] = wrapper->getCamera(0); //camera_count > 4 ? 4 : 0
    }
    else
        subwindow_cameras_[0] = subwindow_cameras_[1] = subwindow_cameras_[2] = nullptr;


    for (size_t window_index = 0; window_index < AirSimSettings::kSubwindowCount; ++window_index) {

        const auto& subwindow_setting = AirSimSettings::singleton().subwindow_settings.at(window_index);

        if (subwindow_setting.camera_id >= 0 && subwindow_setting.camera_id < camera_count)
            subwindow_cameras_[subwindow_setting.window_index] = wrapper->getCamera(subwindow_setting.camera_id);
        else
            UAirBlueprintLib::LogMessageString("CameraID in <SubWindows> element in settings.json is invalid",
                std::to_string(window_index), LogDebugLevel::Failure);
    }

}



// Attempts to parse the settings text from one of multiple locations.
// First, check the command line for settings provided via "-s" or "--settings" arguments
// Next, check the executable's working directory for the settings file.
// Finally, check the user's documents folder.
// If the settings file cannot be read, throw an exception

bool ASimHUD::getSettingsText(std::string& settingsText) {
    return (getSettingsTextFromCommandLine(settingsText)
        ||
        readSettingsTextFromFile(FString(Settings::getExecutableFullPath("settings.json").c_str()), settingsText)
        ||
        readSettingsTextFromFile(FString(Settings::getUserDirectoryFullPath("settings.json").c_str()), settingsText));
}

// Attempts to parse the settings text from the command line
// Looks for the flag "--settings". If it exists, settingsText will be set to the value.
// Example: AirSim.exe -s '{"foo" : "bar"}' -> settingsText will be set to {"foo": "bar"}
// Returns true if the argument is present, false otherwise.
bool ASimHUD::getSettingsTextFromCommandLine(std::string& settingsText) {

    bool found = false;
    FString settingsTextFString;
    const TCHAR* commandLineArgs = FCommandLine::Get();

    if (FParse::Param(commandLineArgs, TEXT("-settings"))) {
        FString commandLineArgsFString = FString(commandLineArgs);
        int idx = commandLineArgsFString.Find(TEXT("-settings"));
        FString settingsJsonFString = commandLineArgsFString.RightChop(idx + 10);
        if (FParse::QuotedString(*settingsJsonFString, settingsTextFString)) {
            settingsText = std::string(TCHAR_TO_UTF8(*settingsTextFString));
            found = true;
        }
    }

    return found;
}

bool ASimHUD::readSettingsTextFromFile(FString settingsFilepath, std::string& settingsText) {

    bool found = FPaths::FileExists(settingsFilepath);
    if (found) {
        FString settingsTextFStr;
        bool readSuccessful = FFileHelper::LoadFileToString(settingsTextFStr, *settingsFilepath);
        if (readSuccessful) {
            UAirBlueprintLib::LogMessageString("Loaded settings from ", TCHAR_TO_UTF8(*settingsFilepath), LogDebugLevel::Informational);
            settingsText = TCHAR_TO_UTF8(*settingsTextFStr);
        }
        else {
            UAirBlueprintLib::LogMessageString("Cannot read file ", TCHAR_TO_UTF8(*settingsFilepath), LogDebugLevel::Failure);
            throw std::runtime_error("Cannot read settings file.");
        }
    }

    return found;
}
