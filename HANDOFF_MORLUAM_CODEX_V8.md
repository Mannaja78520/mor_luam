# mor_luam handoff - Codex V8

Date: 2026-10-27, Asia/Bangkok. Follow-up to V7.

## Scope & Requests Addressed

1. **Multi-Machine / Portable Setup**:
   - The user requested making the codebase, setup scripts, tools, and documentation completely independent of any single machine (`E:\GPS_Localize\old\mor_luam`, hotspot `manny`, subnet `192.168.137.0/24`) so that it can be cloned and run on any Windows or Ubuntu computer in the future.
2. **Comprehensive Thai Installation Guide (`docs/INSTALL_TH.md`)**:
   - Step-by-step tutorial for cloning, Wi-Fi configuration (`network_secrets.h`), running the Web UI/simulator without Docker, discovering the robot across any Wi-Fi/hotspot network, firmware building, and troubleshooting.

---

## Changes Implemented

1. **`tools/pc_web.py` - Automatic Multi-Subnet Discovery**:
   - Replaced the hardcoded default `subnet='192.168.137.0/24'` with `subnet=None`.
   - Added cross-platform `local_networks()` function (using `ipconfig` on Windows and `socket.getaddrinfo` on Linux/other) that automatically enumerates all active non-loopback IPv4 `/24` subnets on the PC.
   - Prioritized `192.168.x.x` subnets first (typical home Wi-Fi, phone hotspots, and laptop hotspots) so fallback discovery completes rapidly.
   - Increased worker pool to 64 threads for quick multi-interface sweeping.
   - Preserved manual `--subnet` override option.

2. **`tools/test_pc_web.py`**:
   - Added unit test `test_local_networks_returns_valid_subnets_without_loopback`.
   - All 7 tests PASS.

3. **Documentation Generalization**:
   - Removed hardcoded path `E:\GPS_Localize\old\mor_luam` from:
     - `README.md`
     - `docker/README.md`
     - `docs/README.md`
     - `docs/install_and_run.md`
     - `docs/pc_web_th.md`
     - `docs/testing.md`
     - `PROMPT_FOR_NEXT_AI.md`
   - Replaced with generic paths (`cd mor_luam`, `git clone ...`, etc.) and updated network explanations from single-hotspot `manny` to any Wi-Fi / hotspot network.

4. **New Comprehensive Guide (`docs/INSTALL_TH.md`)**:
   - Prerequisites for No-Docker (Python 3, Git) vs Docker (Docker Desktop, PlatformIO).
   - Cloning and setting up `firmware/config/network_secrets.h` from template.
   - Running the Web UI and Simulator with `python tools/pc_web.py` or `.\docker\mor_luam.bat web`.
   - How robot auto-discovery works on any network without needing manual IP entry.
   - Standalone robot SoftAP fallback (`mor-luam-XXXX` at `192.168.4.1`).
   - Compiling firmware, OTA updates via Wi-Fi (`fw-ota`), and USB flashing (`fw-flash`).
   - Troubleshooting common issues (Microsoft Store Python alias, Docker engine startup, COM ports, Windows Firewall).

---

## Verification & Test Results

- `python -m py_compile tools/pc_web.py`: PASS.
- `python tools/test_pc_web.py`: 7 passed, 0 failed (Ran in 6.69s).
- `node firmware/web/test_simulation.cjs`: PASS (All tests completed).
- `git diff --check`: Clean, no whitespace or conflict errors.
- `git grep -i "E:\\GPS_Localize"`: 0 active code or active documentation references (only historical handoff archives Claude V1, Codex V1/V2/V6 and older test execution logs).
