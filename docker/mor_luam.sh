#!/usr/bin/env bash
# mor_luam helper for Ubuntu / Linux with Docker. Same commands as
# docker\mor_luam.bat on Windows.            ./docker/mor_luam.sh help
#
# Options anywhere after the command:
#   --host NAME_OR_IP   robot address for fw-ota / robot-test (default mor-luam.local)
#   --port /dev/ttyX    ESP32 USB port for fw-upload / fw-monitor / serial (default /dev/ttyUSB0)
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(dirname "$HERE")"
CMD="${1:-help}"
[ $# -gt 0 ] && shift

HOST="mor-luam.local"
PORT="${MORLUAM_PORT:-/dev/ttyUSB0}"
ARGS=()
while [ $# -gt 0 ]; do
    case "$1" in
        --host|-Host|-host) HOST="$2"; shift 2 ;;
        --port|-Port|-port) PORT="$2"; shift 2 ;;
        *) ARGS+=("$1"); shift ;;
    esac
done
export MORLUAM_PORT="$PORT"          # docker-compose.yml maps it to /dev/ttyUSB0 inside the container
PY="${PYTHON:-python3}"
FW_OUT="$REPO/firmware/firmware_out"

dc() { docker compose -f "$HERE/docker-compose.yml" "$@"; }

need_docker() {
    if ! docker info >/dev/null 2>&1; then
        echo "Docker is not running, or this user may not use it."
        echo "  sudo systemctl start docker"
        echo "  sudo usermod -aG docker \$USER    (then log out and in once)"
        exit 1
    fi
}

need_port() {
    if [ ! -e "$PORT" ]; then
        echo "No $PORT. Plug the ESP32 in and look for it:"
        ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "  (no USB serial device found - try another cable: some only charge)"
        echo "Then add:  --port /dev/ttyUSBx"
        exit 1
    fi
}

need_fw() {
    if [ ! -f "$FW_OUT/firmware.bin" ]; then
        echo "No firmware/firmware_out/firmware.bin yet. Build it first:  $0 fw-build"
        exit 1
    fi
}

# files the root user in the container wrote into the repo: give them back to you
fix_owner() { dc --profile tools run --rm firmware bash -c "chown -R $(id -u):$(id -g) firmware_out src/web/WebPage.h 2>/dev/null || true" >/dev/null; }

show_agent() {
    echo
    echo "This PC's IPv4 addresses: $(hostname -I 2>/dev/null || true)"
    echo "The robot finds the agent by itself only when this PC is its Wi-Fi gateway (PC hotspot)."
    echo "Otherwise set 'micro-ROS agent' on the robot's web page (Settings) to this PC's IP or $(hostname).local"
    echo "If the robot never connects and ufw is on:  sudo ufw allow 8888/udp"
}

case "$CMD" in
    build)
        need_docker
        dc --profile tools build ros firmware
        dc --profile wifi pull agent-wifi ;;
    wifi)
        need_docker
        dc --profile wifi up -d agent-wifi
        show_agent
        echo "Agent running. Watch the robot connect:  $0 logs" ;;
    serial)
        need_docker; need_port
        echo "Only for firmware built with  board_microros_transport = serial  (platformio.ini)."
        dc --profile serial up -d agent-serial ;;
    logs)   need_docker; dc --profile wifi --profile serial logs -f --tail 50 ;;
    stop)   need_docker; dc --profile wifi --profile serial --profile tools down ;;
    shell)  need_docker; dc --profile tools run --rm ros bash ;;
    topics) need_docker; dc --profile tools run --rm ros ros2 topic list ;;
    run)
        need_docker
        if [ ${#ARGS[@]} -eq 0 ]; then
            echo "Usage: $0 run drive_to_xy.py 0.3 0 --speed 0.03 --unit mps"; exit 1
        fi
        dc --profile tools run --rm ros ros2 run mor_luam "${ARGS[@]}" ;;

    fw-build)
        need_docker
        dc --profile tools run --rm firmware bash -c "pio run -e morluam && mkdir -p firmware_out && cp .pio/build/morluam/firmware.bin .pio/build/morluam/bootloader.bin .pio/build/morluam/partitions.bin /root/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin firmware_out/ && ls -l firmware_out"
        fix_owner ;;
    fw-upload|fw-flash)
        # build + flash over USB (first time, or when Wi-Fi does not work)
        need_docker; need_port
        dc --profile tools run --rm firmware-usb pio run -e morluam -t upload
        fix_owner ;;
    fw-monitor)
        need_docker; need_port
        dc --profile tools run --rm firmware-usb pio device monitor -b 115200 -p /dev/ttyUSB0 ;;
    fw-ota)
        # update over Wi-Fi through the robot's web app; the password goes in a header file
        need_fw
        PASS="${MORLUAM_OTA_PASS:-}"
        if [ -z "$PASS" ]; then read -rsp "OTA password of the robot (web app > Settings): " PASS; echo; fi
        HDR="$(mktemp)"
        trap 'rm -f "$HDR"' EXIT
        printf 'X-OTA-Pass: %s' "$PASS" > "$HDR"
        echo "Uploading firmware_out/firmware.bin to http://$HOST/ ..."
        if curl --fail-with-body -sS --max-time 180 -H @"$HDR" -F "firmware=@$FW_OUT/firmware.bin" "http://$HOST/api/ota"; then
            echo; echo "Done. The robot reboots now (~10 s)."
        else
            echo; echo "OTA failed. Find the robot:  $0 find   then  $0 fw-ota --host <IP>"; exit 1
        fi ;;
    fw-test)
        need_docker
        dc --profile tools run --rm firmware bash -c "g++ -std=c++17 -O1 -I test_host/stub -I test_host/old_pidf -I lib/PIDF -I config -I src test_host/tests.cpp src/algorithm/DetourSteer.cpp lib/PIDF/PIDF.cpp test_host/old_pidf/PIDF_old.cpp -o /tmp/tests && /tmp/tests && g++ -std=c++17 -O1 -Wall -Wextra -I test_host/nav_runner_stubs -I src -I config test_host/nav_runner_tests.cpp src/nav/WaypointRunner.cpp src/algorithm/DetourSteer.cpp -o /tmp/nav-runner-tests && /tmp/nav-runner-tests" ;;
    fw-clean-ros)
        need_docker
        dc --profile tools run --rm firmware bash -c "rm -rf .pio/libdeps/morluam/micro_ros_platformio/libmicroros && echo 'micro-ROS library removed: the next fw-build rebuilds it (several minutes)'" ;;

    find)       "$PY" "$REPO/tools/find_robots.py" "${ARGS[@]}" ;;
    web)        "$PY" "$REPO/tools/find_robots.py" --open "${ARGS[@]}" ;;   # show where to connect + open the browser
    robot-test) "$PY" "$REPO/tools/robot_test.py" --host "$HOST" "${ARGS[@]}" ;;   # steps 2-4 MOVE the robot
    web-mock)
        echo "Fake robot for web page work: open http://localhost:8000  (Ctrl+C to stop)"
        "$PY" "$REPO/firmware/web/mock_robot.py" "${ARGS[@]}" ;;
    usb)        ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "no USB serial device" ;;

    *)
        cat <<EOF
mor_luam on Ubuntu / Linux (Docker)          (Windows: docker\\mor_luam.bat, same commands)

  $0 build                      build the images (first time, and after ROS code changes)
  $0 wifi                       start the micro-ROS agent (UDP 8888)
  $0 logs                       watch the agent: the robot connecting
  $0 topics                     list ROS 2 topics
  $0 run <program> <args>       run a mor_luam program, e.g.  $0 run drive_to_xy.py 0.3 0 --speed 0.03 --unit mps
  $0 shell                      a ROS 2 terminal
  $0 stop                       stop everything

  $0 usb                        list USB serial ports (the ESP32 is usually /dev/ttyUSB0)
  $0 fw-build                   build the ESP32 firmware -> firmware/firmware_out/
  $0 fw-upload [--port /dev/ttyUSB0]   build + flash over USB (first flash)
  $0 fw-monitor [--port ...]    serial monitor (115200)
  $0 fw-ota [--host IP]         update over Wi-Fi (default mor-luam.local)
  $0 fw-test                    PC tests (algorithm, PIDF, Wi-Fi rules, route runner)
  $0 fw-clean-ros               rebuild micro-ROS next time (after editing firmware/microros.meta)
  $0 serial [--port ...]        agent over USB (serial-transport firmware only)

  $0 web                        find the robot, show where to connect, open the web app
  $0 find                       find robots on this network (then open http://<IP>/)
  $0 web-mock                   fake robot on http://localhost:8000
  $0 robot-test --steps 1       real-robot tests (steps 2-4 MOVE the robot: stand next to it)
EOF
        ;;
esac
