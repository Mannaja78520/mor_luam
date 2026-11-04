# mor_luam — start here (any AI agent)

1. Read `HANDOFF.md` first (the only live handoff: state, open items, rules). Old handoffs are history in `docs/handoff_history/`. The robot IP changes: use `mor-luam.local` for the ESP32; `localhost:8000` is the PC web app.
2. Read `CLAUDE.md` (source map + commands). Open only the files the task needs.
3. Never print `firmware/config/network_secrets.h` or the output of `GET /api/wifi` (Wi-Fi passwords).
4. Do not commit or push unless the user asks. Hardware config (`firmware/config/esp32_hardware.h`) is the user's: ask before changing it.
5. Real-robot motion tests only with the user present; `tools/robot_test.py` sends E-STOP on exit.
6. When you stop, update `HANDOFF.md` (State, Open items, one Log line). Do not create new `HANDOFF_*_V<n>.md` files.
