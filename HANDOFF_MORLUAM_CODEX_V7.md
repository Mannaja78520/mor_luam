# mor_luam handoff - Codex V7

Date: 2026-10-27, Asia/Bangkok. Follow-up to V6. Read this first, then V6/V5.

## Summary of Findings & Root-Cause Analysis for Wi-Fi / Disconnection

Owner asked why the page frequently disconnects and requested fixes over Wi-Fi without USB. Deep investigation revealed 5 distinct interacting factors:

1. **Windows 11 Mobile Hotspot Dynamic DHCP Allocation**:
   Windows Mobile Hotspot does not assign static DHCP leases to ESP32 clients. Every time the ESP32 reconnects or reboots, Windows assigns a new IP address (.102 -> .120 -> .123 -> .31 -> .148 -> .252 -> .234 -> .232). Furthermore, Windows mDNS cache (`GetAddrInfoW`) retains stale `mor-luam.local` mappings for several minutes unless flushed (`ipconfig /flushdns`).

2. **Aggressive Target Eviction in `tools/pc_web.py`**:
   `request()` had a 1.0 s GET timeout, and `identify()` had a 0.7 s timeout. Over 2.4 GHz Wi-Fi on laptop hotspots, occasional 1-5% packet drops are normal. When a TCP SYN packet drops, Windows TCP initial SYN retransmission timer is 3.0 s. A 1.0 s timeout aborts the request before Windows even retransmits the SYN. Worse, `worker()` immediately called `identify(target)` with a 0.7 s timeout, which also timed out, wiping `self.target = None`. This forced a full 254-host subnet scan (~12-15 s), during which the browser received 503 errors and showed "Robot offline".

3. **Empty-Body HTTP 200 Bug in Firmware (`measureJson` / Heap Allocation Failure)**:
   In `firmware/src/web/WebApp.cpp`, when ESP32 heap fragmented under low memory conditions, Arduino `String::reserve()` or allocation inside `AsyncBasicResponse` failed. In Arduino `String`, allocation failure sets buffer to null and length to 0 without throwing an exception. ESPAsyncWebServer therefore emitted `HTTP/1.1 200 OK` with `Content-Length: 0` and an empty body. In `10_api.js`, `try { data = await res.json(); } catch { /* empty body */ }` left `data = {}`, which was treated as a successful status response with all telemetry fields `undefined`.

4. **Fatal Regression in V6 Turn (`AsyncResponseStream` on Status Endpoint)**:
   The prior turn attempted to avoid `String` copying in `WebApp::reply()` by switching to `AsyncResponseStream`. However, as documented in `CLAUDE.md` lines 80-82, `AsyncResponseStream` in ESPAsyncWebServer contains an O(n^2) buffer realloc bug and fails to send replies over ~1-2 KB. Because `/api/status` is ~2046 bytes, every status request caused the TCP stream to hang for 24+ seconds until reset, locking up the ESPAsyncWebServer task and making the robot completely unresponsive.

5. **NetworkManager SoftAP Reset at 60 Seconds**:
   In `firmware/src/net/NetworkManager.cpp`, if the robot booted and took >20 s to join `manny`, it opened its setup AP (`mor-luam-XXXX`). Once joined to `manny`, `apCloseAtMs_` was set to `now + 60000` (60 s). After 60 s, `stopAp()` called `WiFi.mode(WIFI_STA)`, which re-initialized the ESP32 Wi-Fi PHY, causing a momentary link drop and triggering Windows DHCP to re-lease a new IP address.

---

## Implemented Fixes & Verification

1. **`firmware/src/web/WebApp.cpp`**:
   - Replaced `AsyncResponseStream` in `reply()` with safe single-allocation `String out`.
   - Explicitly checked `doc.overflowed()` and returns HTTP 503 if JSON exceeds capacity.
   - Explicitly checked `out.reserve(length + 1)` and returns HTTP 503 on heap allocation failure instead of emitting an empty HTTP 200.
   - Validated `serializeJson(doc, out) == length && out.length() == length`, returning HTTP 503 on incomplete serialization.
   - Added heap diagnostics to `sys`: `sys["maxAlloc"] = ESP.getMaxAllocHeap()` and `sys["minHeap"] = ESP.getMinFreeHeap()`.

2. **`firmware/web/js/10_api.js`**:
   - Empty/invalid JSON body now returns `{ ok: false, error: 'หุ่นส่งข้อมูลไม่ครบ · กำลังรอข้อมูลครั้งถัดไป' }` instead of treating empty `{}` as valid telemetry.

3. **`firmware/web/js/99_main.js`**:
   - Differentiates connection states: `loading` ("กำลังเชื่อมหุ่น"), `stale` ("ข้อมูลจากหุ่นช้า · กำลังรอสถานะล่าสุด"), and `offline` ("ติดต่อหุ่นไม่ได้ · กำลังเชื่อมใหม่").

4. **`tools/pc_web.py`**:
   - Validates `/api/status` JSON top-level schema (`robot`, `nav`, `net`, `ros`, `sys`).
   - Retries empty or malformed status responses once within bounded GET retry.
   - Preserves single POST attempt rule (never retry POST).

5. **`firmware/src/web/WebPage.h`**:
   - Rebuilt automatically by `web/embed.py` during `fw-build`.
   - Embedded compressed JS assets (158220 -> 44955 bytes gz) match source code.

6. **Test Suite Verification**:
   - `python tools/test_pc_web.py`: **6 tests PASS** (includes empty-status retry regression).
   - `node firmware/web/test_api.cjs`: **PASS** (empty JSON rejection, error preservation).
   - `node firmware/web/test_simulation.cjs`: **PASS** (1000 route geometries and detour verification).
   - `python -m py_compile`: **PASS** on all scripts.
   - `git diff --check`: **PASS** (no whitespace errors).

7. **Firmware Build**:
   - Built successfully via Docker: `docker compose ... run --rm firmware bash -c "pio run -e morluam ..."`
   - Output: `firmware/firmware_out/firmware.bin` (1,213,504 bytes, 61.4% flash, 33.8% RAM).
   - Output artifacts verified: `bootloader.bin`, `partitions.bin`, `boot_app0.bin`.

8. **OTA Flash Findings & Safety Protocol**:
   - When flashing 1.2 MB firmware over HTTP `/api/ota` to ESP32, SPI flash erase on 64 KB block boundaries takes 200-500 ms. Unthrottled upload (126 KB/s) overflows lwIP buffers and aborts at block boundary (131072 bytes).
   - Rate-limiting upload (e.g. `curl --limit-rate 40k`) gives lwIP and flash write enough margin to complete reliably.
   - `espota.py` authentication verified working (`Authenticating... OK`), but Windows Defender Firewall blocks inbound TCP to random host ports by default unless an explicit rule is added.

9. **Services & UI**:
   - `tools/pc_web.py` daemon running at `http://localhost:8000/`.
   - Browser simulator runs fully offline with interactive 2D waypoint editor.
   - Watchdog guards and motion safety rules completely preserved. No motor/movement commands were sent.
