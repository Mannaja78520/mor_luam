#pragma once
// The robot's own web app: http://mor-luam.local (or 192.168.4.1 on the setup hotspot).
// The page's source is firmware/web/ (index.html, app.css, js/*.js), packed into
// web/WebPage.h by web/embed.py at build time; this file is the JSON API it talks to.
//
//   GET  /  /app.css  /app.js    the page (gzipped, ETag)
//   GET  /api/whoami             {type:"morluam", id, name, host, ip, fw}   (how finders recognise a robot)
//   GET  /api/status             everything the page shows, ~4x a second
//   POST /api/estop              stop the route and hold the wheel
//   POST /api/pose/reset         here becomes (0,0), facing +x
//   GET/POST /api/waypoints      {points:[{x,y,waitS}]}; ?slot=1|2 / {slot} = the demo button's route 1/2
//   POST /api/nav/start|stop|heartbeat   (heartbeat, and every GET /api/status, keep a web route/test move alive)
//   POST /api/nav/test          {planner:"direct"|"detour",startHeadingDeg:0..360,ready:true}; response {ok,nav}
//   GET  /api/demo/compare       last button demo 3 (direct) / 4 (detour) run: start, goal, time, path, acts
//   POST /api/demo/compare/start {planner:"direct"|"detour",ready:true}: demo 3/4 from the web (heartbeat rule)
//   POST /api/demo/series/start  {rounds:1..3,ready:true}: Direct+Detour per round, one after another (heartbeat rule)
//   GET  /api/demo/series        that series: progress + every run's time and time line (no paths)
//   GET  /api/demo/series?run=i  run i with its path (flat xy) - small replies only, see seriesJson()
//   GET  /api/wifi               saved networks WITH passwords, and the link
//   POST /api/wifi/save|delete|move|reconnect,  GET/POST /api/wifi/scan
//   GET/POST /api/settings,  GET/POST /api/pid,  GET/POST /api/peers
//   POST /api/ota                firmware upload (multipart, field "firmware", header X-OTA-Pass)
//   POST /api/reboot
//   POST /api/test/move          {rpm, headingDeg, distM, tolM}: one bounded command, for tests (limits in app_config.h)
//   GET  /api/trace?s=10         last s seconds of the control loop at 100 Hz, CSV (PIDF tuning)
//
// To add an endpoint: one route in routes(), handled by one of the services.
#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <functional>

class Settings;
class WifiStore;
class NetworkManager;
class ControlLoop;
class WaypointRunner;
class MicroRosBridge;
class OtaService;

class WebApp {
public:
    struct Deps {
        Settings* settings;
        WifiStore* wifi;
        NetworkManager* net;
        ControlLoop* ctrl;
        WaypointRunner* runner;
        MicroRosBridge* ros;
        OtaService* ota;
    };
    void begin(const Deps& d);
    void loop();                                   // reboots asked for by the page

private:
    using JsonHandler = std::function<void(AsyncWebServerRequest*, JsonDocument&)>;
    void routes();
    void routesNav();
    void routesWifi();
    void routesConfig();
    void routesOta();
    void routesTest();
    void onJson(const char* path, JsonHandler fn);  // POST with a JSON body
    static void reply(AsyncWebServerRequest* r, JsonDocument& doc, int code = 200);
    static void ok(AsyncWebServerRequest* r);
    static void fail(AsyncWebServerRequest* r, const String& err, int code = 400);
    static void sendAsset(AsyncWebServerRequest* r, const uint8_t* gz, size_t len, const char* type);
    void statusJson(JsonObject o);

    AsyncWebServer server_{80};
    Deps d_{};
    volatile uint32_t rebootAtMs_ = 0;
};
