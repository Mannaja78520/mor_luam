#include "app/DemoButton.h"
#include "app_config.h"
#include "control/ControlLoop.h"
#include "nav/WaypointRunner.h"

void DemoButton::begin(uint8_t pin, WaypointRunner* runner, ControlLoop* ctrl) {
    pin_ = pin;
    runner_ = runner;
    ctrl_ = ctrl;
    counter_ = ClickCounter(DEMO_DEBOUNCE_MS, DEMO_CLICK_GAP_MS, DEMO_MAX_PRESS_MS);
    pinMode(pin_, INPUT_PULLUP);                   // the button pulls the pin to GND
    // below the control task (3), above loop() (1); sleeps 5 ms between reads
    xTaskCreatePinnedToCore(taskEntry, "button", 4096, this, 2, nullptr, 1);
    Serial.printf("[button] GPIO%u: 1/2 clicks = route 1/2, 3 = Direct, 4 = Detour, press while moving = stop\n", pin_);
}

void DemoButton::taskEntry(void* arg) { static_cast<DemoButton*>(arg)->run(); }

void DemoButton::run() {
    for (;;) {
        const ClickCounter::Event ev = counter_.update(digitalRead(pin_) == LOW, millis());
        if (counter_.pressed() != lastPressed_) {          // live state for the web page
            lastPressed_ = counter_.pressed();
            runner_->setButtonPressed(lastPressed_);
        }
        if (ev == ClickCounter::Event::Press && (runner_->running() || ctrl_->moving() || runner_->seriesActive())) {
            runner_->stop("กดปุ่มที่หุ่น - หยุด");     // halts the wheel as well
            counter_.cancel();                         // this press stops; it is not a click
            note("หยุดด้วยปุ่มที่หุ่น", 0);
        } else if (ev == ClickCounter::Event::Clicks) {
            startDemo(counter_.clicks());
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void DemoButton::startDemo(uint8_t clicks) {
    String err;
    bool ok = false;
    if (clicks >= 1 && clicks <= 4 && !runner_->running() && !ctrl_->moving())
        ctrl_->halt("เตรียมเดโม");               // a robot holding its wheel counts as stopped
    // Right after another demo the wheel may still settle or the sensors refresh:
    // try again for up to ~2 s before giving up.
    for (int attempt = 0; attempt < 7; ++attempt) {
        err = "";
        switch (clicks) {
            case 1:
            case 2: ok = runner_->startButtonRoute(clicks, err); break;
            case 3: ok = runner_->startCompare("direct", err); break;
            case 4: ok = runner_->startCompare("detour", err); break;
            default: err = "กด " + String(clicks) + " ครั้ง: ไม่มีโหมดนี้ (มีโหมด 1-4)"; break;
        }
        const bool transient = err.indexOf("หยุดหุ่น") >= 0 || err.indexOf("รอเซนเซอร์") >= 0;
        if (ok || !transient) break;
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    static const char* const names[] = {"", "เดโม 1: เส้นทาง 1", "เดโม 2: เส้นทาง 2",
                                        "เดโม 3: Direct (ไม่ลัด)", "เดโม 4: Detour (ลัด)"};
    const String what = clicks >= 1 && clicks <= 4 ? String(names[clicks]) : String("กด ") + clicks + " ครั้ง";
    note(ok ? what + " - เริ่มแล้ว (กดปุ่มอีกครั้งเพื่อหยุด)" : what + " - เริ่มไม่ได้: " + err, clicks);
}

void DemoButton::note(const String& text, uint8_t clicks) {
    Serial.printf("[button] %s\n", text.c_str());
    runner_->noteButton(text, clicks);
}
