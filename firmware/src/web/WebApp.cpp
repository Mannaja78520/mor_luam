#include "web/WebApp.h"
#include <config.h>
#include "app/Settings.h"
#include "app_config.h"
#include "control/ControlLoop.h"
#include "nav/WaypointRunner.h"
#include "net/NetworkManager.h"
#include "net/OtaService.h"
#include "net/WifiStore.h"
#include "ros/MicroRosBridge.h"
#include "web/WebPage.h"
#include <memory>

// ---- helpers ------------------------------------------------------------------

void WebApp::reply(AsyncWebServerRequest* r, JsonDocument& doc, int code) {
    String out;
    serializeJson(doc, out);
    r->send(code, "application/json", out);
}

void WebApp::ok(AsyncWebServerRequest* r) { r->send(200, "application/json", "{\"ok\":true}"); }

void WebApp::fail(AsyncWebServerRequest* r, const String& err, int code) {
    JsonDocument doc;
    doc["ok"] = false;
    doc["error"] = err;
    reply(r, doc, code);
}

// The page files are stored gzipped; every browser accepts that. The ETag
// changes with the content, so a reload after OTA always gets the new page
// and an unchanged page costs one 304.
void WebApp::sendAsset(AsyncWebServerRequest* r, const uint8_t* gz, size_t len, const char* type) {
    if (r->hasHeader("If-None-Match") && r->getHeader("If-None-Match")->value() == WEB_ETAG) {
        r->send(304);
        return;
    }
    AsyncWebServerResponse* res = r->beginResponse_P(200, type, gz, len);
    res->addHeader("Content-Encoding", "gzip");
    res->addHeader("Cache-Control", "no-cache");
    res->addHeader("ETag", WEB_ETAG);
    r->send(res);
}

// POST with a JSON body: gather the chunks into one malloc'd buffer (the
// request frees _tempObject with free()), parse once, then call fn.
void WebApp::onJson(const char* path, JsonHandler fn) {
    server_.on(
        path, HTTP_POST,
        [fn](AsyncWebServerRequest* r) {
            const char* body = static_cast<const char*>(r->_tempObject);
            JsonDocument doc;
            if (body && body[0] && deserializeJson(doc, body)) {
                fail(r, "JSON ไม่ถูกต้อง");
                return;
            }
            fn(r, doc);
        },
        nullptr,
        [](AsyncWebServerRequest* r, uint8_t* data, size_t len, size_t index, size_t total) {
            if (total == 0 || total > 8192) return;              // nothing here is that big
            if (index == 0) {
                r->_tempObject = malloc(total + 1);
                if (r->_tempObject) static_cast<char*>(r->_tempObject)[0] = 0;
            }
            char* buf = static_cast<char*>(r->_tempObject);
            if (!buf || index + len > total) return;
            memcpy(buf + index, data, len);
            buf[index + len] = 0;
        });
}

// ---- wiring --------------------------------------------------------------------

void WebApp::begin(const Deps& d) {
    d_ = d;
    routes();
    routesNav();
    routesWifi();
    routesConfig();
    routesOta();
    routesTest();
    server_.onNotFound([](AsyncWebServerRequest* r) { r->send(404, "text/plain", "not found"); });
    server_.begin();
    Serial.println("[web] http://<robot>/ on port 80");
}

void WebApp::loop() {
    if (rebootAtMs_ && (int32_t)(millis() - rebootAtMs_) >= 0) {
        Serial.println("[web] reboot");
        delay(100);
        ESP.restart();
    }
}

void WebApp::statusJson(JsonObject o) {
    const RobotState s = d_.ctrl->snapshot();
    JsonObject r = o["robot"].to<JsonObject>();
    r["x"] = s.x;
    r["y"] = s.y;
    r["thetaDeg"] = s.thetaRad * 57.29578f;
    r["headingDeg"] = s.headingDeg;
    r["wheelHeadingDeg"] = s.wheelHeadingDeg;
    r["steerDeg"] = s.steerDeg;
    r["steerTargetDeg"] = s.steerTargetDeg;
    r["steerErrDeg"] = s.steerErrDeg;
    r["steerOk"] = s.steerOk;
    r["steerGlitches"] = s.steerGlitches;
    r["imuOk"] = s.imuOk;
    r["imuHeadingFresh"] = s.imuHeadingFresh;
    r["imuMotionFresh"] = s.imuMotionFlags == 3;
    float gyro2 = 0, accel2 = 0;
    for (unsigned axis = 0; axis < 3; ++axis) {
        gyro2 += s.imuGyroDps[axis] * s.imuGyroDps[axis];
        accel2 += s.imuAccelMps2[axis] * s.imuAccelMps2[axis];
    }
    r["imuGyroDps"] = sqrtf(gyro2);
    r["imuAccelMps2"] = sqrtf(accel2);
    r["rpm"] = s.rpm;
    r["targetRpm"] = s.targetRpm;
    r["targetHeadingDeg"] = s.targetHeadingDeg;
    r["vx"] = s.vx;
    r["pwm"] = s.pwm;
    r["mode"] = s.halted ? "halt" : (s.driving ? "drive" : "steer");
    r["source"] = sourceName(s.source);
    r["overshot"] = s.overshot;
    r["overshootDeg"] = s.overshootDeg;
    r["steerRateDps"] = s.steerRateDps;
    r["steerPowerLimit"] = STEER_POWER_LIMIT_PWM;
    r["coasting"] = s.coasting;
    r["coastS"] = s.coastS;
    r["coastSamples"] = s.coastSamples;
    r["driveGain"] = s.driveGain;
    r["driveLearnedS"] = s.driveLearnedS;
    r["motionFault"] = s.motionFault == 1 ? "steer-no-response" : (s.motionFault == 2 ? "drive-no-response" : "");
    r["hardwareEstop"] = "not-wired";
    r["groundContact"] = "unknown";
    r["goalActive"] = s.goalActive;
    r["goalX"] = s.goalX;
    r["goalY"] = s.goalY;
    r["haltWhy"] = d_.ctrl->lastHaltReason();
    if (s.motionFault) r["haltWhy"] = "สั่งมอเตอร์แล้วไม่มีสัญญาณการหมุน: ตรวจไฟ สวิตช์ฉุกเฉิน มอเตอร์ และเซนเซอร์";
    d_.runner->statusJson(o["nav"].to<JsonObject>());
    d_.net->statusJson(o["net"].to<JsonObject>());
    d_.ros->statusJson(o["ros"].to<JsonObject>());
    JsonObject sys = o["sys"].to<JsonObject>();
    sys["fw"] = FW_VERSION;
    sys["build"] = __DATE__ " " __TIME__;
    sys["uptimeS"] = millis() / 1000;
    sys["heap"] = ESP.getFreeHeap();
    sys["tickUs"] = d_.ctrl->tickUs();
    sys["otaPct"] = d_.ota->updating() ? d_.ota->progressPct() : -1;
    sys["name"] = d_.settings->get().robotName;
}

void WebApp::routes() {
    server_.on("/", HTTP_GET, [](AsyncWebServerRequest* r) {
        sendAsset(r, WEB_INDEX_GZ, WEB_INDEX_GZ_LEN, "text/html; charset=utf-8");
    });
    server_.on("/app.css", HTTP_GET, [](AsyncWebServerRequest* r) {
        sendAsset(r, WEB_CSS_GZ, WEB_CSS_GZ_LEN, "text/css; charset=utf-8");
    });
    server_.on("/app.js", HTTP_GET, [](AsyncWebServerRequest* r) {
        sendAsset(r, WEB_JS_GZ, WEB_JS_GZ_LEN, "application/javascript; charset=utf-8");
    });

    server_.on("/api/whoami", HTTP_GET, [this](AsyncWebServerRequest* r) {
        JsonDocument doc;
        doc["type"] = "morluam";
        doc["id"] = d_.net->id();
        doc["name"] = d_.settings->get().robotName;
        doc["host"] = d_.settings->get().hostname + ".local";
        doc["ip"] = d_.net->ip().toString();
        doc["fw"] = FW_VERSION;
        reply(r, doc);
    });

    server_.on("/api/status", HTTP_GET, [this](AsyncWebServerRequest* r) {
        JsonDocument doc;
        statusJson(doc.to<JsonObject>());
        reply(r, doc);
    });

    server_.on("/api/reboot", HTTP_POST, [this](AsyncWebServerRequest* r) {
        if (d_.ctrl->moving()) { fail(r, "หุ่นกำลังวิ่ง - หยุดก่อน"); return; }
        rebootAtMs_ = millis() + 500;
        ok(r);
    });
}

void WebApp::routesNav() {
    server_.on("/api/estop", HTTP_POST, [this](AsyncWebServerRequest* r) {
        d_.runner->stop("หยุดฉุกเฉินจากหน้าเว็บ");
        ok(r);
    });

    server_.on("/api/pose/reset", HTTP_POST, [this](AsyncWebServerRequest* r) {
        if (d_.runner->running() || d_.ctrl->moving()) { fail(r, "หยุดหุ่นก่อน แล้วค่อยตั้งจุดเริ่มต้นใหม่"); return; }
        d_.ctrl->resetPose();
        ok(r);
    });

    server_.on("/api/waypoints", HTTP_GET, [this](AsyncWebServerRequest* r) {
        JsonDocument doc;
        d_.runner->pointsJson(doc["points"].to<JsonArray>());
        reply(r, doc);
    });
    onJson("/api/waypoints", [this](AsyncWebServerRequest* r, JsonDocument& doc) {
        JsonArrayConst a = doc["points"].as<JsonArrayConst>();
        Waypoint pts[NAV_MAX_POINTS];
        size_t n = 0;
        for (JsonObjectConst p : a) {
            if (n >= NAV_MAX_POINTS) { fail(r, "จุดมากเกินไป"); return; }
            pts[n++] = {p["x"] | NAN, p["y"] | NAN};
        }
        String err;
        if (!d_.runner->setPoints(pts, n, err)) { fail(r, err); return; }
        ok(r);
    });

    server_.on("/api/nav/start", HTTP_POST, [this](AsyncWebServerRequest* r) {
        String err;
        if (!d_.runner->start(err)) { fail(r, err); return; }
        ok(r);
    });
    onJson("/api/nav/test", [this](AsyncWebServerRequest* r, JsonDocument& doc) {
        if (!doc["planner"].is<const char*>() || !doc["startHeadingDeg"].is<float>() || !doc["ready"].is<bool>()) {
            fail(r, "ต้องมี planner, startHeadingDeg เป็นตัวเลข และ ready เป็น true"); return;
        }
        String err;
        if (!d_.runner->startTest(doc["planner"].as<const char*>(), doc["startHeadingDeg"].as<float>(), doc["ready"].as<bool>(), err)) {
            fail(r, err); return;
        }
        JsonDocument result;
        result["ok"] = true;
        d_.runner->statusJson(result["nav"].to<JsonObject>());
        reply(r, result);
    });
    server_.on("/api/nav/stop", HTTP_POST, [this](AsyncWebServerRequest* r) {
        d_.runner->stop("หยุดจากหน้าเว็บ");
        ok(r);
    });
    server_.on("/api/nav/heartbeat", HTTP_POST, [this](AsyncWebServerRequest* r) {
        d_.runner->heartbeat();
        ok(r);
    });
}

void WebApp::routesWifi() {
    onJson("/api/wifi/save", [this](AsyncWebServerRequest* r, JsonDocument& doc) {
        const String ssid = doc["ssid"] | "";
        const String pass = doc["pass"] | "";
        const String original = doc["original"] | "";
        if (ssid.isEmpty() || ssid.length() > 32) { fail(r, "ชื่อ WiFi ต้องยาว 1-32 ตัว"); return; }
        if (pass.length() > 0 && (pass.length() < 8 || pass.length() > 63)) { fail(r, "รหัส WiFi ต้องยาว 8-63 ตัว (หรือเว้นว่างถ้าไม่มีรหัส)"); return; }
        if (!d_.wifi->set(ssid.c_str(), pass.c_str())) { fail(r, "บันทึกไม่ได้ (เก็บได้สูงสุด " + String(WifiStore::MAX) + " วง)"); return; }
        if (!original.isEmpty() && original != ssid) d_.wifi->remove(original.c_str());
        if (!d_.net->connected()) d_.net->requestReconnect();
        ok(r);
    });
    onJson("/api/wifi/delete", [this](AsyncWebServerRequest* r, JsonDocument& doc) {
        if (!d_.wifi->remove((doc["ssid"] | ""))) { fail(r, "ไม่พบ WiFi นี้"); return; }
        ok(r);
    });
    onJson("/api/wifi/move", [this](AsyncWebServerRequest* r, JsonDocument& doc) {
        if (!d_.wifi->moveTo((doc["ssid"] | ""), doc["to"] | 0)) { fail(r, "ย้ายไม่ได้"); return; }
        ok(r);
    });
    server_.on("/api/wifi/reconnect", HTTP_POST, [this](AsyncWebServerRequest* r) {
        d_.net->requestReconnect();
        ok(r);
    });
    server_.on("/api/wifi/scan", HTTP_GET, [this](AsyncWebServerRequest* r) {
        JsonDocument doc;
        d_.net->scanJson(doc.to<JsonObject>());
        reply(r, doc);
    });
    server_.on("/api/wifi/scan", HTTP_POST, [this](AsyncWebServerRequest* r) {
        d_.net->requestScan();
        ok(r);
    });
    // LAST: the library lets "/api/wifi" also answer "/api/wifi/<anything>",
    // so registered first it would swallow GET /api/wifi/scan.
    server_.on("/api/wifi", HTTP_GET, [this](AsyncWebServerRequest* r) {
        JsonDocument doc;
        d_.wifi->toJson(doc["saved"].to<JsonArray>());
        doc["max"] = WifiStore::MAX;
        d_.net->statusJson(doc["net"].to<JsonObject>());
        reply(r, doc);
    });
}

void WebApp::routesConfig() {
    server_.on("/api/settings", HTTP_GET, [this](AsyncWebServerRequest* r) {
        JsonDocument doc;
        d_.settings->toJson(doc.to<JsonObject>(), true);
        reply(r, doc);
    });
    onJson("/api/settings", [this](AsyncWebServerRequest* r, JsonDocument& doc) {
        const String oldHost = d_.settings->get().hostname;
        String err;
        if (!d_.settings->update(doc.as<JsonObjectConst>(), err)) { fail(r, err); return; }
        if (d_.settings->get().hostname != oldHost) d_.net->requestHostnameApply();
        d_.ota->refreshPassword();
        ok(r);
    });

    server_.on("/api/pid", HTTP_GET, [this](AsyncWebServerRequest* r) {
        JsonDocument doc;
        float v[5];
        d_.ctrl->getPid(false, v);
        JsonArray spin = doc["spin"].to<JsonArray>();
        for (float x : v) spin.add(x);
        d_.ctrl->getPid(true, v);
        JsonArray steer = doc["steer"].to<JsonArray>();
        for (float x : v) steer.add(x);
        reply(r, doc);
    });
    onJson("/api/pid", [this](AsyncWebServerRequest* r, JsonDocument& doc) {
        const String loop = doc["loop"] | "";
        JsonArrayConst a = doc["values"].as<JsonArrayConst>();
        float v[9];
        size_t n = 0;
        if (a.size() > 9) { fail(r, "ค่า PID มากเกินไป"); return; }
        for (JsonVariantConst x : a) {
            if (!x.is<float>()) { fail(r, "ค่า PID ต้องเป็นตัวเลข"); return; }
            v[n++] = x.as<float>();
        }
        if ((loop != "spin" && loop != "steer") || n < 5) { fail(r, "ต้องมี loop = spin/steer และค่า Kp Ki Kd Kf tol"); return; }
        if (!d_.ctrl->setPid(loop == "steer", v, n)) { fail(r, "ค่า PID หรือขอบเขตกำลังไม่ถูกต้อง"); return; }
        ok(r);
    });

    server_.on("/api/peers", HTTP_GET, [this](AsyncWebServerRequest* r) {
        JsonDocument doc;
        d_.net->peersJson(doc.to<JsonObject>());
        reply(r, doc);
    });
    server_.on("/api/peers", HTTP_POST, [this](AsyncWebServerRequest* r) {
        d_.net->requestPeers();
        ok(r);
    });
}

void WebApp::routesOta() {
    server_.on(
        "/api/ota", HTTP_POST,
        [this](AsyncWebServerRequest* r) {
            // _tempObject is set only after the final chunk was written and verified
            const bool done = r->_tempObject != nullptr;
            if (!done) {
                const String err = d_.ota->lastError();
                fail(r, err.isEmpty() ? "อัปโหลดไม่สำเร็จ" : err);
                return;
            }
            rebootAtMs_ = millis() + 1500;
            r->send(200, "application/json", "{\"ok\":true,\"reboot\":true}");
        },
        [this](AsyncWebServerRequest* r, const String& filename, size_t index, uint8_t* data, size_t len, bool final) {
            String err;
            if (index == 0) {
                // header, not ?pass=: a query string ends up in browser history and logs
                const String pass = r->hasHeader("X-OTA-Pass") ? r->getHeader("X-OTA-Pass")->value() : String("");
                if (!d_.ota->beginWeb(0, pass, err)) {
                    Serial.printf("[OTA] refused: %s\n", err.c_str());
                    return;
                }
            }
            if (!d_.ota->updating()) return;
            if (len && !d_.ota->writeWeb(data, len, err)) return;
            if (final) {
                if (d_.ota->endWeb(err)) r->_tempObject = malloc(1);   // mark success
                else Serial.printf("[OTA] %s\n", err.c_str());
            }
        });
}

// ---- tests and tuning on the real robot ------------------------------------------

void WebApp::routesTest() {
    onJson("/api/test/move", [this](AsyncWebServerRequest* r, JsonDocument& doc) {
        if (d_.runner->running()) { fail(r, "หยุดเส้นทางก่อน"); return; }
        DriveCommand c;
        c.rpm = doc["rpm"] | 0.0f;
        c.headingDeg = doc["headingDeg"] | NAN;
        c.distM = doc["distM"] | 0.0f;
        c.tolM = doc["tolM"] | 0.0f;
        if (!isfinite(c.headingDeg)) { fail(r, "ต้องมี headingDeg"); return; }
        if (fabsf(c.rpm) > TEST_MAX_RPM) { fail(r, "rpm เกิน " + String(TEST_MAX_RPM, 0)); return; }
        if (c.rpm != 0.0f && !(c.distM > 0.0f && c.distM <= TEST_MAX_DIST_M)) {
            fail(r, "ถ้า rpm ไม่เป็น 0 ระยะต้องอยู่ระหว่าง 0-" + String(TEST_MAX_DIST_M, 1) + " m");
            return;
        }
        d_.ctrl->command(c, CommandSource::Web);
        ok(r);
    });

    server_.on("/api/trace", HTTP_GET, [this](AsyncWebServerRequest* r) {
        const float secs = r->hasParam("s") ? r->getParam("s")->value().toFloat() : 10.0f;
        ControlLoop* ctrl = d_.ctrl;
        const uint16_t n = ctrl->traceFreeze((uint32_t)(constrain(secs, 0.1f, 15.0f) * 1000.0f));
        struct Cursor { uint16_t i = 0, n = 0; bool header = true; uint32_t t0 = 0; };
        auto cur = std::make_shared<Cursor>();
        cur->n = n;
        cur->t0 = n ? ctrl->traceAt(0).ms : 0;
        AsyncWebServerResponse* res = r->beginChunkedResponse(
            "text/csv", [ctrl, cur](uint8_t* buf, size_t maxLen, size_t) -> size_t {
                size_t len = 0;
                if (cur->header) {
                    static const char header[] = "t_ms,steer_deg,target_deg,rate_dps,pwm,rpm,target_rpm,x_m,y_m,flags,drive_gain,imu_yaw_deg,gx_dps,gy_dps,gz_dps,ax_mps2,ay_mps2,az_mps2,imu_flags,gyro_seq,accel_seq\n";
                    if (maxLen < sizeof(header) - 1) return RESPONSE_TRY_AGAIN;
                    memcpy(buf, header, sizeof(header) - 1);
                    len = sizeof(header) - 1;
                    cur->header = false;
                }
                while (cur->i < cur->n) {
                    const TraceRecorder::Sample& s = ctrl->traceAt(cur->i);
                    char line[256];
                    const int used = snprintf(line, sizeof(line), "%lu,%.1f,%.1f,%d,%d,%.1f,%.1f,%.3f,%.3f,%u,%.3f,%.2f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%u,%u,%u\n",
                                    (unsigned long)(s.ms - cur->t0), s.steer10 / 10.0f, s.target10 / 10.0f, s.rateDps,
                                    s.pwm, s.rpm10 / 10.0f, s.targetRpm10 / 10.0f, s.xMm / 1000.0f, s.yMm / 1000.0f,
                                    s.flags, s.driveGain1000 / 1000.0f, s.imuYaw100 / 100.0f,
                                    s.gyro10[0] / 10.0f, s.gyro10[1] / 10.0f, s.gyro10[2] / 10.0f,
                                    s.accel100[0] / 100.0f, s.accel100[1] / 100.0f, s.accel100[2] / 100.0f,
                                    s.imuFlags, s.gyroSeq, s.accelSeq);
                    if (used < 0 || (size_t)used >= sizeof(line) || (size_t)used > maxLen - len) break;
                    memcpy(buf + len, line, used);
                    len += used;
                    ++cur->i;
                }
                if (len == 0 && cur->i < cur->n) return RESPONSE_TRY_AGAIN;   // no room this time
                if (len == 0) ctrl->traceUnfreeze();   // finished
                return len;
            });
        r->send(res);
    });
}
