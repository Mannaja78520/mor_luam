# mor_luam helper for Windows + Docker Desktop.
# Run through mor_luam.bat, for example:   docker\mor_luam.bat wifi
#
# No param() block on purpose: arguments such as "--speed 0.25" must reach
# ros2 unchanged, and PowerShell would try to bind them as its own options.

$ErrorActionPreference = "Stop"
$Compose = Join-Path $PSScriptRoot "docker-compose.yml"
$Repo = Split-Path $PSScriptRoot -Parent

$Command = if ($args.Count -gt 0) { [string]$args[0] } else { "help" }
$Rest = @()
if ($args.Count -gt 1) { $Rest = @($args[1..($args.Count - 1)] | ForEach-Object { [string]$_ }) }

# -BusId <id>, -Port <COMx> and -Host <name or IP> may appear anywhere after
# the command; take them out of $Rest
$BusId = ""
$Port = ""
$RobotHost = "mor-luam.local"
$keep = @()
for ($i = 0; $i -lt $Rest.Count; $i++) {
    $opt = $Rest[$i].ToLower()
    if (($opt -eq "-busid" -or $opt -eq "--busid") -and $i + 1 -lt $Rest.Count) { $BusId = $Rest[$i + 1]; $i++ }
    elseif (($opt -eq "-port" -or $opt -eq "--port") -and $i + 1 -lt $Rest.Count) { $Port = $Rest[$i + 1]; $i++ }
    elseif (($opt -eq "-host" -or $opt -eq "--host") -and $i + 1 -lt $Rest.Count) { $RobotHost = $Rest[$i + 1]; $i++ }
    else { $keep += $Rest[$i] }
}
$Rest = $keep
$FwOut = Join-Path $Repo "firmware\firmware_out"

function Dc {
    & docker compose -f $Compose @args
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

function Need-Docker {
    & docker info *> $null
    if ($LASTEXITCODE -ne 0) {
        Write-Host "Docker is not running. Open Docker Desktop, wait until it says 'Engine running', then try again." -ForegroundColor Red
        exit 1
    }
}

function Attach-Usb {
    if (-not (Get-Command usbipd -ErrorAction SilentlyContinue)) {
        Write-Host "usbipd-win is not installed:  winget install --exact dorssel.usbipd-win" -ForegroundColor Red
        exit 1
    }
    if ($BusId -eq "") {
        Write-Host "Which USB is the ESP32? Find its BUSID (CP210x / CH340 / USB-SERIAL) below:" -ForegroundColor Yellow
        & usbipd list
        Write-Host ""
        Write-Host "Then run the same command again with:  -BusId <BUSID>      e.g.  -BusId 1-3" -ForegroundColor Yellow
        Write-Host "First time only, in an Administrator terminal:  usbipd bind --busid <BUSID>" -ForegroundColor Yellow
        exit 1
    }
    & usbipd attach --wsl --busid $BusId
    if ($LASTEXITCODE -ne 0) {
        Write-Host "Attach failed. First time only, in an Administrator terminal:  usbipd bind --busid $BusId" -ForegroundColor Red
        exit 1
    }
    Start-Sleep -Seconds 2
    Write-Host "USB $BusId attached to WSL (it shows up as /dev/ttyUSB0)." -ForegroundColor Green
}

function Need-Firmware {
    if (-not (Test-Path (Join-Path $FwOut "firmware.bin"))) {
        Write-Host "No firmware_out\firmware.bin yet. Build it first:  docker\mor_luam.bat fw-build" -ForegroundColor Red
        exit 1
    }
}

# First flash over USB from Windows itself (no usbipd, no Administrator):
# esptool from the PlatformIO install in %USERPROFILE%\.platformio.
function Flash-Com {
    Need-Firmware
    if ($Port -eq "") {
        Write-Host "Which COM port is the ESP32? (Device Manager > Ports, e.g. 'Silicon Labs CP210x (COM5)')" -ForegroundColor Yellow
        Get-CimInstance Win32_PnPEntity -ErrorAction SilentlyContinue | Where-Object { $_.Name -match "\(COM\d+\)" } |
            ForEach-Object { Write-Host "  $($_.Name)" }
        Write-Host "Then:  docker\mor_luam.bat fw-flash -Port COM5" -ForegroundColor Yellow
        Write-Host "Only Bluetooth COM ports listed? The cable may be charge-only, or the CP210x / CH340 driver is missing." -ForegroundColor Yellow
        exit 1
    }
    $py = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\python.exe"
    $esptool = Join-Path $env:USERPROFILE ".platformio\packages\tool-esptoolpy\esptool.py"
    if (-not (Test-Path $py) -or -not (Test-Path $esptool)) {
        Write-Host "esptool not found in %USERPROFILE%\.platformio. Install PlatformIO (VS Code extension) once, or use fw-upload -BusId." -ForegroundColor Red
        exit 1
    }
    Push-Location $FwOut
    & $py $esptool --chip esp32 --port $Port --baud 460800 write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect `
        0x1000 bootloader.bin 0x8000 partitions.bin 0xe000 boot_app0.bin 0x10000 firmware.bin
    $code = $LASTEXITCODE
    Pop-Location
    if ($code -ne 0) {
        Write-Host "Flash failed. Hold the BOOT button on the ESP32 when 'Connecting...' shows, then try again." -ForegroundColor Red
        exit $code
    }
    Write-Host "Flashed. From now on update over WiFi:  docker\mor_luam.bat fw-ota" -ForegroundColor Green
}

# Update over WiFi through the robot's web app (POST /api/ota). The password
# goes to curl in a header file, never on the command line.
function Flash-Ota {
    Need-Firmware
    $pass = $env:MORLUAM_OTA_PASS
    if (-not $pass) {
        $sec = Read-Host "OTA password of the robot (web app > Settings)" -AsSecureString
        $pass = [Runtime.InteropServices.Marshal]::PtrToStringAuto([Runtime.InteropServices.Marshal]::SecureStringToBSTR($sec))
    }
    $hdr = [IO.Path]::GetTempFileName()
    try {
        Set-Content -LiteralPath $hdr -Value "X-OTA-Pass: $pass" -Encoding ascii -NoNewline
        Write-Host "Uploading firmware_out\firmware.bin to http://$RobotHost/ ..."
        & curl.exe --fail-with-body -sS --max-time 180 -H "@$hdr" -F "firmware=@$(Join-Path $FwOut 'firmware.bin')" "http://$RobotHost/api/ota"
        $code = $LASTEXITCODE
    } finally {
        Remove-Item -LiteralPath $hdr -Force -ErrorAction SilentlyContinue
    }
    Write-Host ""
    if ($code -ne 0) {
        Write-Host "OTA failed. Find the robot first:  docker\mor_luam.bat find   then  fw-ota -Host <IP>" -ForegroundColor Red
        exit $code
    }
    Write-Host "Done. The robot reboots now (~10 s)." -ForegroundColor Green
}

function Show-AgentIp {
    $conf = Join-Path $Repo "firmware\config\conf_network.h"
    $agent = ""
    if (Test-Path $conf) {
        $m = Select-String -Path $conf -Pattern 'AGENT_IP\((\d+),\s*(\d+),\s*(\d+),\s*(\d+)\)' | Select-Object -First 1
        if ($m) { $g = $m.Matches[0].Groups; $agent = "$($g[1]).$($g[2]).$($g[3]).$($g[4])" }
    }
    $ips = @(Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue |
             Where-Object { $_.IPAddress -notlike "127.*" -and $_.IPAddress -notlike "169.254.*" } |
             Select-Object -ExpandProperty IPAddress)
    Write-Host ""
    Write-Host "This PC's IPv4 addresses : $($ips -join ', ')"
    if ($agent -ne "") {
        Write-Host "Firmware AGENT_IP        : $agent   (firmware\config\conf_network.h)"
        if ($ips -contains $agent) {
            Write-Host "OK - the ESP32 will send to this PC." -ForegroundColor Green
        } else {
            Write-Host "AGENT_IP is not this PC. Give this PC that IP on the robot WiFi, or change AGENT_IP and flash again." -ForegroundColor Yellow
        }
    }
    Write-Host "If the robot never connects, allow UDP 8888 in Windows Firewall (Administrator PowerShell, once):"
    Write-Host '  New-NetFirewallRule -DisplayName "micro-ROS agent UDP 8888" -Direction Inbound -Protocol UDP -LocalPort 8888 -Action Allow'
}

switch ($Command.ToLower()) {
    "build" {
        Need-Docker
        Dc --profile tools build ros firmware
        Dc --profile wifi pull agent-wifi
    }
    "wifi" {
        Need-Docker
        Dc --profile wifi up -d agent-wifi
        Show-AgentIp
        Write-Host ""
        Write-Host "Agent is running. Watch the robot connect:  docker\mor_luam.bat logs" -ForegroundColor Green
    }
    "serial" {
        Need-Docker
        Attach-Usb
        Write-Host "Note: the firmware must be built with  board_microros_transport = serial  (platformio.ini)." -ForegroundColor Yellow
        Dc --profile serial up -d agent-serial
    }
    "logs"   { Need-Docker; Dc --profile wifi --profile serial logs -f --tail 50 }
    "stop"   { Need-Docker; Dc --profile wifi --profile serial --profile tools down }
    "shell"  { Need-Docker; Dc --profile tools run --rm ros bash }
    "topics" { Need-Docker; Dc --profile tools run --rm ros ros2 topic list }
    "run" {
        Need-Docker
        if ($Rest.Count -eq 0) { Write-Host "Usage: docker\mor_luam.bat run drive_to_xy.py 2 0 --speed 0.25 --unit mps"; exit 1 }
        Dc --profile tools run --rm ros ros2 run mor_luam @Rest
    }
    "fw-build" {
        Need-Docker
        Dc --profile tools run --rm firmware bash -c "pio run -e morluam && mkdir -p firmware_out && cp .pio/build/morluam/firmware.bin .pio/build/morluam/bootloader.bin .pio/build/morluam/partitions.bin /root/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin firmware_out/ && ls -l firmware_out"
    }
    "fw-upload" {
        Need-Docker
        Attach-Usb
        Dc --profile tools run --rm firmware-usb pio run -e morluam -t upload
    }
    "fw-clean-ros" {
        # after editing firmware\microros.meta: the micro-ROS library is rebuilt on the next fw-build
        Need-Docker
        Dc --profile tools run --rm firmware bash -c "rm -rf .pio/libdeps/morluam/micro_ros_platformio/libmicroros && echo 'micro-ROS library removed: the next fw-build rebuilds it (several minutes)'"
    }
    "fw-flash" { Flash-Com }
    "fw-ota"   { Flash-Ota }
    "fw-test" {
        Need-Docker
        Dc --profile tools run --rm firmware bash -c "g++ -std=c++17 -O1 -I test_host/stub -I test_host/old_pidf -I lib/PIDF -I config -I src test_host/tests.cpp src/algorithm/DetourSteer.cpp lib/PIDF/PIDF.cpp test_host/old_pidf/PIDF_old.cpp -o /tmp/tests && /tmp/tests && g++ -std=c++17 -O1 -Wall -Wextra -I test_host/nav_runner_stubs -I src -I config test_host/nav_runner_tests.cpp src/nav/WaypointRunner.cpp src/algorithm/DetourSteer.cpp -o /tmp/nav-runner-tests && /tmp/nav-runner-tests"
    }
    "find" { & python (Join-Path $Repo "tools\find_robots.py") @Rest }
    "web"  { & python (Join-Path $Repo "tools\find_robots.py") --open @Rest }   # find the robot, show the addresses, open the browser
    "robot-test" {
        # THE ROBOT MOVES in steps 2-4: on the floor, ~2 m clear, someone next to it
        $h = $RobotHost                                  # mor-luam.local by default (the IP can change)
        & python (Join-Path $Repo "tools\robot_test.py") --host $h @Rest
    }
    "web-mock" {
        Write-Host "Fake robot for web page work: open http://localhost:8000  (Ctrl+C to stop)" -ForegroundColor Green
        & python (Join-Path $Repo "firmware\web\mock_robot.py") @Rest
    }
    "fw-monitor" {
        Need-Docker
        Attach-Usb
        Dc --profile tools run --rm firmware-usb pio device monitor -b 115200 -p /dev/ttyUSB0
    }
    "usb" { & usbipd list }
    default {
        Write-Host @"
mor_luam on Windows (Docker Desktop)

  docker\mor_luam.bat build                 build the images (first time, and after code changes)
  docker\mor_luam.bat wifi                  start the micro-ROS agent for the WiFi robot (UDP 8888)
  docker\mor_luam.bat logs                  watch the agent: the robot connecting, topics created
  docker\mor_luam.bat topics                list ROS 2 topics (the robot's /mor_luam/... ones once connected)
  docker\mor_luam.bat run <program> <args>  run a mor_luam program, e.g.
        docker\mor_luam.bat run drive_to_xy.py 2 0 --speed 0.25 --unit mps
        docker\mor_luam.bat run send_heading_speed.py 90 120
  docker\mor_luam.bat shell                 a ROS 2 terminal (ros2 topic echo, ros2 run ...)
  docker\mor_luam.bat stop                  stop everything

  docker\mor_luam.bat usb                   list USB devices (find the ESP32 BUSID)
  docker\mor_luam.bat serial -BusId 1-3     agent over USB instead of WiFi (serial firmware only)
  docker\mor_luam.bat fw-build              build the ESP32 firmware -> firmware\firmware_out\
  docker\mor_luam.bat fw-flash -Port COM5   first flash over USB, straight from Windows (no usbipd)
  docker\mor_luam.bat fw-ota [-Host IP]     update over WiFi (default host mor-luam.local)
  docker\mor_luam.bat fw-upload -BusId 1-3  build + flash over USB through WSL (usbipd)
  docker\mor_luam.bat fw-monitor -BusId 1-3 serial monitor (115200)
  docker\mor_luam.bat fw-test               PC tests: Detour Steer vs the homework, PIDF

  docker\mor_luam.bat web                   find the robot, show where to connect, open the web app
  docker\mor_luam.bat find                  find robots on this WiFi (then open http://<IP>/)
  docker\mor_luam.bat web-mock              fake robot on http://localhost:8000 for web page work
"@
    }
}
