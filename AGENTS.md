# mor_luam — start here (any AI agent)

1. Read the NEWEST handoff first: `HANDOFF_MORLUAM_CODEX_V9.md` (2026-10-27; polling recovery, target retention, actual robot currently not found), then `HANDOFF_MORLUAM_CODEX_V8.md` (portable setup), `HANDOFF_MORLUAM_CODEX_V7.md`, `HANDOFF_MORLUAM_CODEX_V6.md`, `HANDOFF_MORLUAM_CODEX_V5.md`, `HANDOFF_MORLUAM_CODEX_V4.md`, `HANDOFF_MORLUAM_CODEX_V3.md`, `HANDOFF_MORLUAM_CODEX_V2.md`, `HANDOFF_MORLUAM_CLAUDE_V2.md` and `HANDOFF_MORLUAM_CODEX_V1.md`. The robot IP changes: use `mor-luam.local` for the ESP32; `localhost:8000` is the PC web app.
2. Read `CLAUDE.md` (source map + commands). Open only the files the task needs.
3. Never print `firmware/config/network_secrets.h` or the output of `GET /api/wifi` (Wi-Fi passwords).
4. Do not commit or push unless the user asks. Hardware config (`firmware/config/esp32_hardware.h`) is the user's: ask before changing it.
5. Real-robot motion tests only with the user present; `tools/robot_test.py` sends E-STOP on exit.
6. When you stop, write your own `HANDOFF_MORLUAM_<AGENT>_V<n>.md` (do not edit another agent's handoff).
