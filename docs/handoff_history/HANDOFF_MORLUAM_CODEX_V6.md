# mor_luam handoff - Codex V6 (urgent context handoff)

Date: 2026-10-27, Asia/Bangkok. Owner requested a handoff before reaching the usage limit. Read this first, then V5/V4. Do not repeat completed implementation.

## Latest request and constraints

Owner asks why the page frequently disconnects and wants it fixed through Wi-Fi. Yesterday everything worked. USB is inconvenient; do not insist on USB. Owner confirmed their previous power cycle cut BOTH ESP32 and motor power. No real motion readiness was confirmed. No movement, route start, steering or drive command has been sent in this diagnostic work. Hardware config must remain unchanged. Never print network_secrets.h or raw /api/settings or /api/wifi responses.

## Repositories / published state

- Robot: E:\GPS_Localize\old\mor_luam, origin Mannaja78520/mor_luam, main. Last published commit before this turn: **5ff0592** (PC worker handles malformed HTTP), preceded by 3012e78 (read retries/coalescing), dce4d30 (PC web/offline editor/2 m multi-leg feature). Owner previously authorized uploading all related code/docs; this authorization persists.
- Homework: E:\271401\271401_pee_aut_homework, related Detour work published **5256e45**. Unrelated pre-existing edits/untracked projects preserved. Do not git-add everything in that repository.
- Chat workspace: C:\Users\manma\Documents\Codex\2026-10-26\morluam. work/ holds maintenance scripts and evidence; outputs/ holds screenshots and PDF. Earlier handoffs record all 2 m comparison details and tests.

## Network evidence

- Hotspot manny on Windows (192.168.137.1, adapter Wi-Fi 4). Robot id 8FD570 / MAC suffix 8F:D5:70. IP changed .102 -> .120; restarting hotspot Off/On restored communication at **192.168.137.123**. Native mor-luam.local subsequently resolves to .123. Do not hard-code future IPs.
- Before this turn, PC browser visibly showed live .123, halt/PWM0, steerOk/imuOk true, installed build **Oct 27 2026 03:09:06**. Some HTTP reads still failed.
- Current diagnostics: TCP connect often succeeds in 25-55 ms, but no HTTP headers arrive within 1-2 s. One 8-ping sample had 7 replies, one loss. RSSI around -49..-51 dBm. Board uptime in successful samples increased normally (~3467-3506 s), tickUs around 6923, heap ~63488. Root cause is NOT established; do not claim Wi-Fi association is completely fixed or blame hardware conclusively.
- **Important new observation:** GET /api/status sometimes returned HTTP200, Content-Type application/json, Content-Length **0**, empty body. Other successful status replies were ~2046 bytes. This is a real invalid-success response, separate from the UI's 1.5 s stale threshold.
- Native keep-alive request experiment did not help: ESP response says Connection: close. Proxy settings returned {}. NIC RSC reports NDISCompatibility (not operating). No adapter/security settings changed. PktMon capture lacked privilege; no capture started.

## Local changes in THIS turn (not yet published at time of handoff)

1. tools/pc_web.py: validate GET /api/status JSON and require robot/nav/net/ros/sys objects inside the bounded GET retry. Empty/malformed/incomplete HTTP200 no longer counts as live. POST is still never retried. Existing single-worker queue, shared in-flight status, no autonomous heartbeat/no offline replay are preserved.
2. tools/test_pc_web.py: fake status fixture has full top-level schema; added empty-success regression. **6 tests PASS**.
3. firmware/web/js/10_api.js: JSON parsing failure returns ok:false instead of silently treating {} as success, preventing render of missing telemetry. New test_api.cjs uses VM to test this without firmware or hardware; PASS. Route-test UI regression also PASS.
4. firmware/web/js/99_main.js: PC connection message distinguishes loading, stale (data slow) and offline. Previously every non-live state said the robot was not connected. Freshness thresholds and all motion/watchdog guards remain unchanged.
5. firmware/src/web/WebApp.cpp: reply() now uses one pre-sized AsyncResponseStream rather than a temporary String copied by AsyncBasicResponse and joined to headers. Detect JSON overflow/short serialization and return503. This is intended to reduce heap copies; it is NOT yet proven to solve physical transport loss. sys now includes maxAlloc/minHeap memory diagnostics.
6. Generated firmware/src/web/WebPage.h rebuilt. Latest source JS bundle 158220 bytes -> 44955 gz. Windows/Linux gzip header OS byte can make generated headers differ despite identical raw assets. Verified **current web ETag exists in the compiled binary**, so UI content corresponds to this build. Do not hand-edit WebPage.h.

## Build and OTA attempt - read carefully

- Docker fw-build SUCCESS, RAM110636 bytes (33.8%), flash1205037 bytes (61.3%), exported firmware.bin **1211616 bytes**. Output: firmware/firmware_out/firmware.bin. Build took97.48 s. Firmware includes the latest WebApp.cpp and UI parsing/stale-message changes. No algorithm/motor/hardware config changed in this turn.
- Script: chat work/install_network_fix.py. It privately read /api/settings (never printed/saved password), checked actual halt/PWM0 and nav not running, hashed all three saved waypoint slots, and saved ONLY non-secret preflight metadata in **work/network_fix_preflight.json**. Preflight confirmed old build Oct27 03:09:06.
- Uploaded this binary once via authenticated HTTP POST /api/ota. The upload response was **RemoteDisconnected**; no success acknowledgment. Script did NOT retry upload. It then tried to verify status for30 iterations and ended **Installed build not confirmed; do not report OTA success**.
- **Installed version is now UNKNOWN. Do not say the new firmware is installed.** Wi-Fi/ICMP still respond. GET whoami sometimes returns200 but does not expose build. Repeated status reads timed out. Asset/root requests also timed out, so matching running ETag was not achieved. Last probe: root timeout, whoami200 length110, next whoami timeout. No physical power cycle since this OTA attempt.
- It is possible the old image remains, or the new image is running; a lost OTA response cannot distinguish this. Do not blindly retransmit the image. First recover readable status or root ETag and verify build. If more recovery is required, the owner prefers Wi-Fi; ArduinoOTA/espota (UDP3232) is an existing alternative, but was NOT attempted here. Local tool exists at C:\Users\manma\.platformio\packages\framework-arduinoespressif32\tools\espota.py. Avoid printing passwords in commands/logs. An interrupted HTTP upload might leave updating state; investigate before starting another upload.

## Services / UI

- PC server was stopped during the OTA attempt to avoid competing requests. **It has now been started again** from the updated source with `python -u tools/pc_web.py --no-browser`, localhost:8000. Keep it running; its terminal is owned by this chat. Check /pc/status for current discovered target. This page always supports offline simulation; do not invent telemetry.
- Existing IAB pcTab (tab5), localhost:8000, Simulation selected, 7 points stored in browser. Two tabs visible in ambient context. CUA handles from prior turn persist; call cua.rewriteDocumentation after a compacted computer-use summary, then reuse pcTab if valid. Do not open Wi-Fi/settings tabs into screenshots (credentials visible by user choice).
- JS loaded in the existing browser tab predates THIS turn's parser/stale-message changes until reloaded. Reload after confirming PC service is available; browser-local simulation points persist, animation result resets.
- Latest prior screenshot: outputs/wifi-reconnected.png was captured before this turn; it showed alternating live/stale, so do not use it to claim this turn's fix is verified. Need fresh UI verification after recovery.

## Next steps

1. Review git diff; preserve all changes above. Owner interrupted for urgent handoff, not because work was complete. Do not repeat homework/2 m implementation or ask motion readiness for read-only network work.
2. Recover/read the actual installed build over Wi-Fi. Keep honest about unknown OTA result. No motion test is authorized.
3. Test empty-status fixes plus actual HTTP stability with a bounded series of read-only status samples, compare failures and uptime/memory. One successful request is not proof of stable operation.
4. Reload PC UI; verify distinct slow/offline labels and automatic recovery. Keep server alive. Do not relax the3 s motor watchdog or make stale telemetry look fresh.
5. Update docs/testing/pc_web guide and publish source + new handoff under previous upload authorization once checks are complete. Current turn's changes are uncommitted; do not lose them. Update AGENTS/index to the newest handoff.

Earlier completed work remains: offline waypoint editor, 2 m total outbound/7 legs comparison (ideal137.82 s vs110.52 s), preserved button routes1/2, paginated compare detail, homework CCW/PDF updates. Three paired REAL 2 m runs remain unperformed. Models are not physical measurements.
