# Install and run everything — Windows, Ubuntu + Docker, Ubuntu native

Pick ONE way:

| Way | Best for | Helper |
|---|---|---|
| **A. Windows + Docker Desktop** | this PC (the one with the `manny` hotspot) | `docker\mor_luam.bat <command>` |
| **B. Ubuntu + Docker** (recommended on Linux) | any Ubuntu PC, nothing else to install | `./docker/mor_luam.sh <command>` |
| **C. Ubuntu native** (no Docker) | if you already use ROS 2 Jazzy on Ubuntu 24.04 | plain `ros2` / `pio` commands |

The commands of A and B are the same words (`fw-build`, `fw-ota`, `wifi`, ...).
Thai web guide: [web_guide_th.md](web_guide_th.md). Safety before any motion test: [safety.md](safety.md).

---

## 1. Install

### Everyone: get the code and your Wi-Fi file
```bash
git clone https://github.com/Mannaja78520/mor_luam.git
cd mor_luam
```
Copy `firmware/config/network_secrets.example.h` to `firmware/config/network_secrets.h` and fill in:
- your Wi-Fi networks (name + password), most important first
- `MORLUAM_DEFAULT_OTA_PASS` and `MORLUAM_DEFAULT_AP_PASS` (passwords a new robot starts with)

`network_secrets.h` is ignored by git: it never goes to GitHub. Never paste its content anywhere.

### A. Windows 10/11
1. **Docker Desktop** (WSL 2 backend): https://www.docker.com/products/docker-desktop/ — start it once, wait for "Engine running".
2. **Git**: https://git-scm.com/download/win
3. **Python 3.12** from https://www.python.org/downloads/ — tick **"Add python.exe to PATH"**.
   Then Settings → Apps → Advanced app settings → App execution aliases → turn OFF the two "python" Store aliases.
4. **First USB flash only** — one of:
   - VS Code + the **PlatformIO** extension (gives `%USERPROFILE%\.platformio`, used by `fw-flash`), or
   - `winget install --exact dorssel.usbipd-win` (used by `fw-upload -BusId`)
   - If no COM port appears for the ESP32: install the **Silicon Labs CP210x** driver.
5. Optional: **Node.js LTS** (only to run the web page tests).
6. Build the images (10–15 min the first time):
   ```
   docker\mor_luam.bat build
   docker\mor_luam.bat fw-build
   ```

### B. Ubuntu 22.04 / 24.04 with Docker
```bash
sudo apt update
sudo apt install -y git curl python3 docker.io docker-compose-v2 avahi-daemon libnss-mdns
sudo usermod -aG docker,dialout $USER      # docker without sudo + USB serial port
# log out and back in once, then:
chmod +x docker/mor_luam.sh
./docker/mor_luam.sh build                 # images (first time 10-15 min)
./docker/mor_luam.sh fw-build              # firmware
```
(`docker-compose-v2` is the Ubuntu 24.04 package name. If apt cannot find it, install Docker from
https://docs.docker.com/engine/install/ubuntu/ — it includes `docker compose`.)
`avahi-daemon` + `libnss-mdns` let Ubuntu open `mor-luam.local`.
Optional: `sudo apt install -y nodejs` for the web page tests.

### C. Ubuntu 24.04 native (no Docker)
1. **ROS 2 Jazzy** — follow https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html (ros-base is enough), then:
   ```bash
   sudo apt install -y python3-colcon-common-extensions python3-yaml python3-matplotlib libyaml-cpp-dev \
       ros-jazzy-nav-msgs ros-jazzy-geometry-msgs ros-jazzy-sensor-msgs ros-jazzy-std-msgs \
       ros-jazzy-rclcpp-action ros-jazzy-ament-index-cpp git curl cmake build-essential python3-venv \
       avahi-daemon libnss-mdns
   sudo usermod -aG dialout $USER            # USB serial port (log out and in once)
   ```
2. **Build the ROS package:**
   ```bash
   source /opt/ros/jazzy/setup.bash
   cd mor_luam_ws && colcon build && source install/setup.bash && cd ..
   echo 'export ROS_DOMAIN_ID=10' >> ~/.bashrc     # the robot uses domain 10
   ```
3. **micro-ROS agent** — easiest is still the official image (Docker only for this one program):
   ```bash
   sudo apt install -y docker.io
   ```
   (or the snap: `sudo snap install micro-ros-agent`)
4. **PlatformIO** (firmware build/flash):
   ```bash
   curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
   python3 get-platformio.py
   echo 'export PATH="$HOME/.platformio/penv/bin:$PATH"' >> ~/.bashrc
   echo 'export ROS_DISTRO=jazzy' >> ~/.bashrc      # platformio.ini needs it for micro-ROS
   source ~/.bashrc
   cd firmware && pio run -e morluam                # first build downloads ~1 GB, 10+ min
   ```

---

## 2. Run

### Network first (all ways)
- The robot joins the saved Wi-Fi with the highest priority. Default here: the Windows hotspot `manny`.
- On Ubuntu you can make the PC the hotspot too: `nmcli device wifi hotspot ssid manny password '<password>'`.
- The robot finds the ROS agent by itself **only when the PC is its gateway** (PC hotspot). On any other network,
  open the robot's web page → Settings → "micro-ROS agent" and enter the PC's IP or `<pc-name>.local`.
- The robot's IP can change. Use `mor-luam.local`, or the `find` command.
- ROS domain is **10** everywhere. UDP **8888** must reach the PC (Windows Firewall / `sudo ufw allow 8888/udp`).

### Commands

| Task | A. Windows | B. Ubuntu + Docker | C. Ubuntu native |
|---|---|---|---|
| Build firmware | `docker\mor_luam.bat fw-build` | `./docker/mor_luam.sh fw-build` | `cd firmware && pio run -e morluam` |
| First flash over USB | `docker\mor_luam.bat fw-flash -Port COM9` | `./docker/mor_luam.sh fw-upload --port /dev/ttyUSB0` | `pio run -e morluam -t upload --upload-port /dev/ttyUSB0` |
| Update over Wi-Fi (OTA) | `docker\mor_luam.bat fw-ota` | `./docker/mor_luam.sh fw-ota` | see "OTA native" below |
| Serial monitor | `fw-monitor -BusId 1-3` | `./docker/mor_luam.sh fw-monitor` | `pio device monitor -b 115200` |
| PC tests | `docker\mor_luam.bat fw-test` | `./docker/mor_luam.sh fw-test` | see "Tests native" below |
| **Open the web app** (finds the robot, prints where to connect, opens the browser) | `docker\mor_luam.bat web` | `./docker/mor_luam.sh web` | `python3 tools/find_robots.py --open` |
| Find the robot | `docker\mor_luam.bat find` | `./docker/mor_luam.sh find` | `python3 tools/find_robots.py` |
| Web page | open `http://mor-luam.local` | same | same |
| Web page without robot (opens the browser) | `docker\mor_luam.bat web-mock` | `./docker/mor_luam.sh web-mock` | `python3 firmware/web/mock_robot.py` → `http://localhost:8000` |
| Start ROS agent | `docker\mor_luam.bat wifi` | `./docker/mor_luam.sh wifi` | `docker run -it --rm --net=host -e ROS_DOMAIN_ID=10 microros/micro-ros-agent:jazzy udp4 --port 8888 -v4` |
| Agent log | `docker\mor_luam.bat logs` | `./docker/mor_luam.sh logs` | (in the agent terminal) |
| ROS topics | `docker\mor_luam.bat topics` | `./docker/mor_luam.sh topics` | `ros2 topic list` |
| Run a ROS program | `docker\mor_luam.bat run drive_to_xy.py 0.3 0 --speed 0.03 --unit mps` | `./docker/mor_luam.sh run drive_to_xy.py 0.3 0 --speed 0.03 --unit mps` | `ros2 run mor_luam drive_to_xy.py 0.3 0 --speed 0.03 --unit mps` |
| ROS terminal | `docker\mor_luam.bat shell` | `./docker/mor_luam.sh shell` | any terminal (sourced) |
| Real-robot tests | `docker\mor_luam.bat robot-test --steps 1` | `./docker/mor_luam.sh robot-test --steps 1` | `python3 tools/robot_test.py --host mor-luam.local --steps 1` |
| Stop Docker parts | `docker\mor_luam.bat stop` | `./docker/mor_luam.sh stop` | Ctrl+C in each terminal |
| E-STOP from the PC | `curl.exe -X POST http://mor-luam.local/api/estop` | `curl -X POST http://mor-luam.local/api/estop` | same |

**Speed:** this robot's drive wheel tops out at ~0.039 m/s (9.7 rpm). Use `--speed 0.03` (or less) in ROS programs.
**Motion:** `robot-test` steps 2–4 and any `run`/route MOVE the robot — floor, ~2 m clear, someone next to it.

### OTA native (Ubuntu without the helper)
```bash
cd firmware && pio run -e morluam
read -rsp "OTA password: " P; echo; printf 'X-OTA-Pass: %s' "$P" > /tmp/ota.h; unset P
curl --fail-with-body -H @/tmp/ota.h -F firmware=@.pio/build/morluam/firmware.bin http://mor-luam.local/api/ota
rm -f /tmp/ota.h
```
(Alternative: `MORLUAM_OTA_PASS=<password> pio run -e morluam_ota -t upload` — espota; the robot must be able to connect back to the PC.)

### Tests native
```bash
cd firmware
g++ -std=c++17 -O1 -I test_host/stub -I test_host/old_pidf -I lib/PIDF -I config -I src test_host/tests.cpp \
    src/algorithm/DetourSteer.cpp lib/PIDF/PIDF.cpp test_host/old_pidf/PIDF_old.cpp -o /tmp/tests && /tmp/tests
g++ -std=c++17 -O1 -Wall -Wextra -I test_host/nav_runner_stubs -I src -I config test_host/nav_runner_tests.cpp \
    src/nav/WaypointRunner.cpp src/algorithm/DetourSteer.cpp -o /tmp/nav-runner-tests && /tmp/nav-runner-tests
node web/test_simulation.cjs && node web/test_simulation_noise.cjs && node web/test_route_test.cjs   # needs Node.js
python3 web/test_mock_route_test.py && python3 ../tools/imu_shake.py --self-test
```

---

## 3. Problems

| Problem | Fix |
|---|---|
| `Docker is not running` | Windows: start Docker Desktop. Ubuntu: `sudo systemctl start docker`, and be in the `docker` group. |
| `Python was not found` (Windows) | install Python with "Add to PATH" and turn off the Store aliases (step A3). |
| No COM port / no `/dev/ttyUSB0` | data cable (not charge-only), CP210x driver (Windows), `dialout` group (Ubuntu), or `--port /dev/ttyUSB1`. |
| `Permission denied: /dev/ttyUSB0` (Ubuntu) | `sudo usermod -aG dialout $USER`, then log out and in. |
| `mor-luam.local` not found | use `find` and the IP; Ubuntu: install `avahi-daemon libnss-mdns`; Android phones often need the IP. |
| ROS shows "waiting for agent" | agent not started, UDP 8888 blocked, or PC is not the gateway → set the agent host in Settings. |
| `fw-ota` says 403 | wrong OTA password. Moving robot → stop it first. |
| Build fails in micro-ROS after editing `firmware/microros.meta` | `fw-clean-ros`, then `fw-build`. |
| Files in `firmware_out/` owned by root (Ubuntu) | the helper fixes it after each build; or `sudo chown -R $USER firmware/firmware_out`. |
