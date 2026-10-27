# mor_luam handoff - Codex V5

Date: 2026-10-27, Asia/Bangkok. Read V4 for implementation, tests and outstanding physical trials.

## Owner clarification

Owner confirmed the earlier power cycle cut both ESP32 and motor power. USB is inconvenient; continue recovery via Wi-Fi. They noted all functions worked yesterday. No motion test was authorized or performed here.

## Wi-Fi recovery and current evidence

- Hotspot client after full power cycle was 192.168.137.120; the old .102 no longer applied. Direct HTTP, ping and TCP80 from source 192.168.137.1 still failed at .120. Hotspot was On with one client and normal subnet route; no USB serial board present. Packet capture was unavailable without additional privileges, so no capture was started.
- Toggled Windows hotspot manny Off then On with the existing helper, preserving credentials. Robot rejoined as **192.168.137.123**, identity 8FD570. Direct whoami, ping and status then succeeded. Native name lookup **mor-luam.local resolved to .123**. Do not hard-code the IP for future sessions.
- Status confirmed build Oct 27 2026 03:09:06, mode halt, PWM 0, steerOk/imuOk true. No firmware flash, motor command, setting change or physical movement was performed. Latest compiled firmware from V4 is still not installed.
- Some HTTP reads still timed out despite reachable ping/whoami. Do not claim root cause or perfect reliability. The hotspot restart restored reachability; the remaining intermittent loss could not be attributed conclusively to Windows or ESP32.

## PC web improvement

- tools/pc_web.py retries a failed GET once with bounded timeouts. POSTs are never retried because a lost response may follow an applied command. An HTTP error is preserved rather than retried.
- Concurrent status callers share a single in-flight real request; no stale cache or fake heartbeat. Recent successful API replies avoid redundant background identity probes. A failed read only clears the target when its small identity probe also fails.
- Catch ConnectionAbortedError along with other ConnectionError variants when browser polling aborts a reply on Windows.
- Five HTTP contract tests PASS: offline simulator, recovery without replay, cross-origin refusal, GET retry versus single POST, shared status reads.
- Restarted the user-facing PC service on localhost:8000 using current code. It automatically discovered .123. Existing browser page visibly showed real .123 and live status. Four additional proxy reads had one failed read followed by three successful halt/PWM0 responses; intermittent loss remains observable.
- Keep the PC service running. Simulation stays usable offline. Do not start a physical route merely because the connection is back.

Publication: V4 implementation was pushed as dce4d30 (mor_luam); homework as 5256e45. This follow-up publishes the PC transport fix and recovery notes under the existing user request to upload all work. Check git log for its final ID.
