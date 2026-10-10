#include "app/App.h"
#include <Wire.h>
#include <math.h>
#include <config.h>
#include "app_config.h"

App::App()
    : motor_(Controller::Drive2pin, PWM_FREQUENCY, PWM_BITS, MOTOR_INV, MOTOR_BRAKE, MOTOR_PWM, MOTOR_IN_A, MOTOR_IN_B),
      encoder_(MOTOR_ENCODER_PIN_A, MOTOR_ENCODER_PIN_B, COUNTS_PER_REV, MOTOR_ENCODER_INV, MOTOR_ENCODER_RATIO,
               WHEEL_DIAMETER),
      ctrl_(motor_, encoder_, steer_, imu_) {}

void App::begin() {
    Serial.begin(115200);
    delay(50);
    Serial.printf("\n[mor_luam] firmware %s\n", FW_VERSION);

    settings_.begin();

    // sensors first: the control task reads them from its first tick
    Wire.begin(SDA_PIN, SCL_PIN);
    imu_.begin();
    delay(100);
    steer_.begin();
    ctrl_.setCoastS(settings_.coastS(STEER_COAST_INIT_S));
    {
        const SettingsData s = settings_.get();
        ctrl_.setTuning(s.steerLandDeg, s.reaimOn, s.reaimRatio);
    }
    // Validate saved values before starting the task (including invalid/NaN NVS data).
    DriveGainLearner savedGain(settings_.driveGain(1.0f, Wheel_SPIN_KS, Wheel_SPIN_KF));
    savedDriveGain_ = savedGain.gain();
    ctrl_.setDriveGain(savedDriveGain_);
    loop_.begin(&ctrl_);                      // control task starts: the wheel is halted

    wifi_.begin(WIFI_SEED);
    net_.begin(&wifi_, &settings_);
    ota_.begin(&settings_, &loop_, settings_.get().hostname);
    runner_.begin(&loop_, &settings_);
    button_.begin(DEMO_BUTTON_PIN, &runner_, &loop_);
    net_.setBusyCheck([this] { return loop_.moving() || runner_.running(); });
    ros_.begin(&loop_, &runner_, &settings_, &net_);
    web_.begin({&settings_, &wifi_, &net_, &loop_, &runner_, &ros_, &ota_});
}

// The steering coast time is learned while driving; keep it across reboots,
// but write NVS at most every 10 s and only when it really changed.
void App::saveLearnedCoast() {
    if (millis() - lastCoastCheckMs_ < 10000) return;
    lastCoastCheckMs_ = millis();
    const RobotState s = loop_.snapshot();
    if (s.coastSamples == savedCoastSamples_ || s.coasting) return;
    savedCoastSamples_ = s.coastSamples;
    settings_.saveCoastS(s.coastS);
}

void App::loop() {
    net_.loop();
    ota_.loop();
    runner_.update();
    ros_.loop();
    web_.loop();
    saveLearnedCoast();
    saveLearnedDriveGain();
    delay(1);                                 // let the idle task run
}

// Check at most every 10 s. Write only after a >= 1% change, while stopped,
// so an NVS write cannot delay a drive tick. Do not keep temporary web PID tuning.
void App::saveLearnedDriveGain() {
    if (millis() - lastDriveGainCheckMs_ < 10000) return;
    lastDriveGainCheckMs_ = millis();
    if (loop_.moving() || runner_.running()) return;
    float pid[5];
    loop_.getPid(false, pid);
    if (pid[3] != Wheel_SPIN_KF) return;
    const RobotState s = loop_.snapshot();
    if (fabsf(s.driveGain - savedDriveGain_) < 0.01f) return;
    if (settings_.saveDriveGain(s.driveGain, Wheel_SPIN_KS, Wheel_SPIN_KF)) savedDriveGain_ = s.driveGain;
}
