# mor_luam handoff - Codex V9

Date: 2026-10-27, Asia/Bangkok. Read this first, then V8 (portable setup), V7 and V6. Preserve earlier handoffs.

## Owner request / baseline

Continue fixing frequent disconnections over Wi-Fi. USB is inconvenient. No movement readiness was confirmed and no movement command was sent. Hardware config unchanged. Never print network_secrets.h or raw /api/settings or /api/wifi responses.

On resumption, the checkout had already advanced to **010b652** (portable discovery/Thai guide), following **4f7bd2a** (V7 status fixes). These changes were read and preserved. V7 restored String-based JSON replies with allocation/serialization guards; do not reintroduce V6's AsyncResponseStream change in reply(). V8's automatic subnet detection remains. Do not treat all earlier diagnostic hypotheses as proven causes of every lost packet.

Robot repo: E:\GPS_Localize\old\mor_luam. Homework Detour work remains published at5256e45; no homework edits in this resumption. Earlier user authorization to upload related code/docs persists.

## New changes

- tools/pc_web.py retains an identified robot address through brief loss. Expire only after at least3 failed identity probes AND at least5 seconds without successful contact. Success resets failures. An old request cannot evict a newer target. A failed scan cannot erase contact restored by the worker meanwhile.
- This does NOT fabricate/cache live status. Failed reads still fail, POSTs remain single-attempt, UI freshness and the3 s motor watchdog are unchanged.
- 10_api.js rejects valid JSON with missing robot/nav/net/ros/sys objects. Poller always schedules another read after an API/view callback error. Failed processing never advances its freshness clock. A bad frame can no longer stop polling permanently.
- Tests cover target retention/expiry, contact recovery, stale-target isolation, incomplete JSON, and recovery after a throwing callback. PC guide/testing/index updated.

## Verification

- Python PC web: **9 tests PASS**.
- node firmware/web/test_api.cjs: invalid/schema-incomplete status rejected; HTTP errors preserved; polling recovers, PASS.
- node firmware/web/test_route_test.cjs: PASS. git diff --check clean.
- Docker fw-build SUCCESS,22.22 s. RAM110636 bytes (33.8%), flash1207093 bytes (61.4%), exported firmware.bin **1213664 bytes**. Generated WebPage.h updated; JS158736 ->45117 gz. This build was NOT uploaded in this resumption.
- Actual localhost:8000 browser reloaded; DOM host-mode pc, connection offline, accurate Thai reconnect text. Simulation has **8 browser-local user points**. Preserve these; do not overwrite with an example. No simulated motion is sent to hardware.

## Physical connection / unfinished work

- Original mor_luam identity8FD570, last IP192.168.137.123. It no longer answered ping/HTTP at resumption.
- Hotspot currently lists ONLY **esp32-1C5A74 / 192.168.137.222**. Its GET /api/whoami returns an authentication object rather than type:morluam. It is another application/device; no login/settings/firmware action attempted. Do not flash it or mistake it for the robot.
- Hotspot /24 scan found no morluam. A later auto scan of **1270 addresses across five PC subnets** also found none. PC /pc/status returns robotHost:null. A pending user question asks whether the actual mor_luam is presently powered; no answer yet. Earlier power-cycle confirmation does not establish current power state.
- V6 HTTP OTA reply was lost; installed build was unconfirmed. V7 describes more investigation but does not explicitly verify a final installed build. **Do not claim this turn's compiled image is installed.** First recover identity/status and sys.build.
- To finish: get original robot reachable, verify halt/PWM0/running build, then measure sustained read-only status reliability. If updating, consider V7's rate-limited upload findings, protect credentials and verify running build afterward. Compilation is not OTA success or proof of physical stability. No real2 m trial is authorized without attended readiness.

## Services / UI

- Port8000 belonged to Python PID4140, created14:00:35 with `-u tools/pc_web.py --no-browser`; the earlier retained terminal was not its listener. Verified this exact command before stopping it, then started current code. Latest exec session **21337**, localhost:8000. Keep this server alive. Default discovery still scans local subnets (V8).
- IAB pcTab tab5 is localhost:8000, Simulation selected, reloaded to current JS. Keep marked as deliverable. Avoid screenshots of Wi-Fi/settings credentials.
- Chat workspace: C:\Users\manma\Documents\Codex\2026-10-26\morluam. V6 maintenance script/evidence in work/. Earlier wifi-reconnected.png shows an earlier live session, not current offline state.

Publish source/docs under prior owner authorization; inspect git log for final commit. Physical recovery remains dependent on the absent robot. All prior offline simulator/2 m7-leg/homework work is preserved; the real three-paired-round trial remains unperformed.
