#pragma once
// The whole robot, put together. Owns one of everything and wires them up;
// main.cpp only calls begin() and loop().
//
//   hardware   SteerSensor, ImuHeading, Controller (motor), esp32_Encoder
//   control    SteerDriveController, run by ControlLoop in its own 100 Hz task
//   nav        WaypointRunner + src/algorithm/ (Detour Steer, Direct)
//   network    WifiStore, NetworkManager (Wi-Fi, hotspot, mDNS), OtaService
//   ROS        MicroRosBridge
//   web        WebApp (+ the page in web/WebPage.h)
#include <motor.h>
#include <esp32_Encoder.h>
#include "app/DemoButton.h"
#include "app/Settings.h"
#include "control/ControlLoop.h"
#include "control/SteerDriveController.h"
#include "hw/ImuHeading.h"
#include "hw/SteerSensor.h"
#include "nav/WaypointRunner.h"
#include "net/NetworkManager.h"
#include "net/OtaService.h"
#include "net/WifiStore.h"
#include "ros/MicroRosBridge.h"
#include "web/WebApp.h"

class App {
public:
    App();
    void begin();
    void loop();

private:
    void saveLearnedCoast();
    void saveLearnedDriveGain();
    Settings settings_;
    WifiStore wifi_;
    NetworkManager net_;
    OtaService ota_;

    Controller motor_;
    esp32_Encoder encoder_;
    SteerSensor steer_;
    ImuHeading imu_;
    SteerDriveController ctrl_;
    ControlLoop loop_;

    WaypointRunner runner_;
    DemoButton button_;
    MicroRosBridge ros_;
    WebApp web_;
    unsigned savedCoastSamples_ = 0;
    uint32_t lastCoastCheckMs_ = 0;
    float savedDriveGain_ = 1.0f;
    uint32_t lastDriveGainCheckMs_ = 0;
};
