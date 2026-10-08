#include "control/ControlLoop.h"
#include "app_config.h"

void ControlLoop::begin(SteerDriveController* ctrl) {
    ctrl_ = ctrl;
    mtx_ = xSemaphoreCreateMutex();
    ctrl_->begin();
    if (!trace_.begin()) Serial.println("[ctrl] no RAM for the trace");
    // core 1 with loop(), but a higher priority, so a blocked loop never stalls it
    xTaskCreatePinnedToCore(taskEntry, "control", 6144, this, 3, nullptr, 1);
}

void ControlLoop::taskEntry(void* arg) { static_cast<ControlLoop*>(arg)->run(); }

void ControlLoop::run() {
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(CTRL_PERIOD_MS);
    for (;;) {
        vTaskDelayUntil(&last, period);
        const uint32_t t0 = micros();
        {
            Lock l(mtx_);
            ctrl_->step();
            trace_.record(ctrl_->state());
        }
        tickUs_ = micros() - t0;
    }
}

uint16_t ControlLoop::traceFreeze(uint32_t lastMs) {
    Lock l(mtx_);
    return trace_.freeze(lastMs);
}

void ControlLoop::traceUnfreeze() {
    Lock l(mtx_);
    trace_.unfreeze();
}

void ControlLoop::command(const DriveCommand& cmd, CommandSource src) {
    Lock l(mtx_);
    ctrl_->apply(cmd, src);
}

void ControlLoop::halt(const char* why) {
    Lock l(mtx_);
    ctrl_->halt();
    haltWhy_ = why;
}

bool ControlLoop::setPid(bool steerLoop, const float* v, size_t n) {
    Lock l(mtx_);
    if (!ctrl_->setPid(steerLoop, v, n)) return false;
    ++pidRevision_;
    return true;
}

void ControlLoop::getPid(bool steerLoop, float out[5]) {
    Lock l(mtx_);
    ctrl_->getPid(steerLoop, out);
}

uint32_t ControlLoop::pidRevision() {
    Lock l(mtx_);
    return pidRevision_;
}

void ControlLoop::resetPose() {
    Lock l(mtx_);
    ctrl_->resetPose();
}

RobotState ControlLoop::snapshot() {
    Lock l(mtx_);
    return ctrl_->state();
}

CommandSource ControlLoop::source() {
    Lock l(mtx_);
    return ctrl_->source();
}

bool ControlLoop::moving() {
    Lock l(mtx_);
    return ctrl_->moving();
}
