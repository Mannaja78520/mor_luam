# Prompt to give the next AI (ChatGPT / Codex / Gemini)

Copy everything between the lines.

---
You are continuing work on my robot project `mor_luam` at `E:\GPS_Localize\old\mor_luam`
(GitHub: https://github.com/Mannaja78520/mor_luam.git). Another AI (Claude) worked on it until its
context ran out.

1. Read the newest handoff first: `HANDOFF_MORLUAM_CLAUDE_V2.md`, then `HANDOFF_MORLUAM_CODEX_V1.md`. The "Status at a glance" block says what is finished,
   what was in progress, and what is next. Then read `CLAUDE.md` (source map + commands).
   Open only the files your task needs.
2. Continue from "Still in progress", then "Next", in order.
3. Rules:
   - Never print `firmware/config/network_secrets.h` or the output of `GET /api/wifi` (WiFi passwords).
   - Do not commit or push unless I ask. Do not change `firmware/config/esp32_hardware.h` (hardware) without asking me.
   - The robot is real. Motion tests (`tools/robot_test.py --steps 2/3/4`) only when I say I am next to it;
     the script sends E-STOP on exit. Manual stop: `curl -X POST http://mor-luam.local/api/estop`.
   - Build/flash on Windows: `docker\mor_luam.bat fw-build`, then `docker\mor_luam.bat fw-ota -Host mor-luam.local`
     (asks for the OTA password; default is MORLUAM_DEFAULT_OTA_PASS in the gitignored `firmware/config/network_secrets.h`). PC tests: `docker\mor_luam.bat fw-test`.
   - Talk to me in simple English or Thai, short sentences.
4. When you stop or finish, write `HANDOFF_MORLUAM_<YOUR_NAME>_V1.md` at the repo root with:
   Finished / Still in progress / Next, repo state, open issues with evidence, exact commands.
   Do not edit Claude's handoff.
---
