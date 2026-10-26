# mor_luam — คู่มือคำสั่ง Windows และ Docker

โค้ดจริงในเครื่องนี้อยู่ที่ `E:\GPS_Localize\old\mor_luam` (ตรวจ 26 ตุลาคม 2026)
เปิด PowerShell ที่โฟลเดอร์นั้น:

```powershell
Set-Location E:\GPS_Localize\old\mor_luam
.\docker\mor_luam.bat help
```

เครื่องใหม่อ่าน [ติดตั้ง Windows/Ubuntu](../docs/install_and_run.md)
Ubuntu ใช้ `./docker/mor_luam.sh` แทน `.\docker\mor_luam.bat`
ตัวเลือก USB บน Ubuntu ใช้ `--port /dev/ttyUSB0`

## เปิดเว็บหุ่น

เปิดหุ่น ให้คอมกับหุ่นอยู่ Wi-Fi เดียวกัน แล้วรัน:

```powershell
.\docker\mor_luam.bat web
```

ใช้ Python บน Windows หาหุ่น แสดง URL/เครือข่ายสำหรับคอมและมือถือ และเปิดเว็บ
**ไม่ต้องเปิด Docker หรือ ROS agent** เว็บอยู่บน ESP32
เครื่องนี้เคยใช้ Mobile Hotspot `manny`; หุ่นเลือก Wi-Fi ตามลำดับที่บันทึกไว้

- เปิดตรง: [mor-luam.local](http://mor-luam.local/)
- หาที่อยู่: `.\docker\mor_luam.bat find` แล้วเปิด IP ที่แสดง
- ต่อ Wi-Fi ไม่ได้เกินประมาณ 20 s: hotspot `mor-luam-XXXX` → ต่อแล้วเปิด [192.168.4.1](http://192.168.4.1/)
- IP เปลี่ยนได้ มือถือที่เปิด `.local` ไม่ได้ให้ใช้ IP

อ่าน [คู่มือเว็บทุกแท็บ/เดโมภาษาไทย](../docs/web_guide_th.md)
เว็บมีเส้นทาง, Simulation, Wi-Fi, ตั้งค่า, ระบบ และกล่องเดโม Direct/Detour
แกน +x หน้า, +y ซ้าย, หน่วยเมตร; ล้อเลี้ยวทวนเข็มนาฬิกาอย่างเดียว

## ตัวช่วยกับ Docker ทำงานตรงไหน

| ส่วน | ทำงานที่ไหน | ใช้ Docker |
|---|---|---|
| ควบคุมมอเตอร์/odometry/เว็บจริง | ESP32 | ไม่ต้อง |
| `find`, `web`, `web-mock`, `robot-test` | Python บนคอม | ไม่ต้อง |
| `fw-ota` | Windows curl ส่งไฟล์ที่ build แล้ว | ไม่ต้อง |
| `fw-flash` | Windows PlatformIO/esptool ส่งไฟล์ที่ build แล้ว | ไม่ต้อง |
| `fw-build`, `fw-test`, `fw-clean-ros` | container `firmware` | ต้อง |
| `wifi`, `serial`, `logs` | micro-ROS agent container | ต้อง |
| `topics`, `run`, `shell` | ROS 2 Jazzy container `ros` | ต้อง |

คำสั่งที่ใช้ Docker ต้องเปิด **Docker Desktop** และรอ engine พร้อมก่อน
`stop` ใช้ Docker Compose ปิด containers

## สร้าง images และเฟิร์มแวร์

เครื่องใหม่คัดลอก `firmware/config/network_secrets.example.h`
เป็น `firmware/config/network_secrets.h` แล้วใส่ Wi-Fi/รหัสส่วนตัวในเครื่อง
อย่าคัดลอกทับไฟล์ที่ตั้งค่าแล้ว ไฟล์จริงถูก Git ignore

```powershell
.\docker\mor_luam.bat build
.\docker\mor_luam.bat fw-test
.\docker\mor_luam.bat fw-build
```

`build` สร้าง images ฝั่ง ROS/firmware และดึง agent image
`fw-test` รัน C++ บนคอม รวม algorithm, PIDF, Wi-Fi rules และ route runner
`fw-build` รวมเว็บจาก `firmware/web/` ได้ไฟล์ใน `firmware\firmware_out\`
PlatformIO cache อยู่ใน Docker volumes `pio-core` / `pio-build`
แก้ ROS source ต้อง `build` ใหม่ เพราะ source ถูกคัดลอกเข้า image
firmware source ถูก mount จึงใช้ `fw-build` หลังแก้ได้เลย

## แฟลช USB ครั้งแรก

ติดตั้ง PlatformIO/esptool บน Windows ใช้สาย USB ที่ส่งข้อมูลได้
ถ้าไม่พบพอร์ต ตรวจไดรเวอร์ CP210x/CH340 ใน Device Manager

```powershell
.\docker\mor_luam.bat fw-flash
.\docker\mor_luam.bat fw-flash -Port COM9
```

คำสั่งแรกแสดงพอร์ตและวิธีใช้ คำสั่งสองเขียนไฟล์ที่ build แล้ว
`COM9` เป็นพอร์ตที่เคยใช้ ตรวจของจริงก่อน ถ้าค้าง `Connecting...` ให้กด BOOT ค้างจนเริ่มเขียน
อีกทางผ่าน WSL + usbipd (ต้องเปิด Docker):

```powershell
.\docker\mor_luam.bat usb
# ครั้งแรก ใน PowerShell แบบ Administrator: usbipd bind --busid <BUSID>
.\docker\mor_luam.bat fw-upload -BusId 1-3
.\docker\mor_luam.bat fw-monitor -BusId 1-3
```

แทน `1-3` ด้วย BUSID จริง `fw-upload` build และ upload; monitor 115200 baud

## อัปเดต Wi-Fi (OTA)

หยุดหุ่นก่อน แล้ว:

```powershell
.\docker\mor_luam.bat fw-build
.\docker\mor_luam.bat fw-ota
```

ค่าเริ่ม host `mor-luam.local` ระบบถามรหัสแบบไม่แสดงข้อความ
หากชื่อใช้ไม่ได้ ใช้ IP จาก `find`: `.\docker\mor_luam.bat fw-ota -Host <IP>`
หรือแท็บ **ระบบ** → เลือก `firmware\firmware_out\firmware.bin` → ใส่รหัส → อัปโหลด
รองรับ `MORLUAM_OTA_PASS` ที่ตั้งแบบส่วนตัวใน process environment
อย่าแสดงรหัสหรือ `network_secrets.h` ใน log/แชต

## หุ่นจำลองบนคอม

```powershell
.\docker\mor_luam.bat web-mock
```

Python เปิด mock ที่ [localhost:8000](http://localhost:8000/) และเปิดเบราว์เซอร์
Ctrl+C ปิด mock; เพิ่ม `--no-browser` เพื่อไม่เปิดเบราว์เซอร์อัตโนมัติ
mock/Simulation ไม่ยืนยันการทำงานของมอเตอร์ พื้น หรือการตัดไฟจริง

## ROS 2 ผ่าน Docker

ใช้เมื่อต้องการสั่งจากโปรแกรม ROS:

```powershell
.\docker\mor_luam.bat build
.\docker\mor_luam.bat wifi
.\docker\mor_luam.bat logs
```

Ctrl+C เลิกดู log; `topics` ดู topics; `shell` เปิด terminal ROS
ROS_DOMAIN_ID = **10**, agent UDP **8888**
หุ่นลองหา agent จากค่าที่บันทึกและ gateway ถ้า PC ไม่ใช่ gateway
ให้ใส่ IP ของ PC ใน micro-ROS agent บนเว็บ
ถ้า Windows Firewall ปิดพอร์ต เปิดครั้งเดียวใน PowerShell แบบ Administrator:

```powershell
New-NetFirewallRule -DisplayName "micro-ROS agent UDP 8888" -Direction Inbound -Protocol UDP -LocalPort 8888 -Action Allow
```

ตัวอย่างนี้ **ขยับหุ่นจริง** ต้องมีคนข้างหุ่น พื้นที่พร้อม และพร้อมตัดไฟมอเตอร์:

```powershell
.\docker\mor_luam.bat run drive_to_xy.py 0.3 0 --speed 0.03 --unit mps
```

ความเร็วทดสอบ 0.03 m/s เต็มกำลังประมาณ 0.039 m/s (9.7 RPM)
`/mor_luam/cmd_vel/move`: `linear.x` = RPM, `angular.z` = มุมเป้าองศา
ROS เข้าควบคุมแทนเส้นทางเว็บ

## หยุดหุ่นและปิด Docker

หยุดจากเว็บ **E-STOP** หรือ:

```powershell
curl.exe -X POST http://mor-luam.local/api/estop
```

เมื่อหยุดหุ่นแล้วจึงปิด containers:

```powershell
.\docker\mor_luam.bat stop
```

`stop` คือ Docker Compose down ไม่ใช่ E-STOP
เว็บ route/test ใช้ watchdog 3 s เมื่อไม่มี heartbeat/status; หน้าอื่นยังเลี้ยงสัญญาณได้
เดโมจากปุ่มไม่ใช้ web heartbeat และมีขีดจำกัดเวลาเคลื่อนที่แยก
ไม่มี hardware E-STOP input หรือ sensor ยืนยันแตะพื้น อ่าน [safety.md](../docs/safety.md)

## คำสั่งอื่นและแก้ปัญหา

| คำสั่ง / อาการ | ใช้ทำอะไร / วิธีแก้ |
|---|---|
| `fw-clean-ros` | ล้าง micro-ROS library หลังแก้ `firmware/microros.meta` แล้ว `fw-build` |
| `robot-test --steps 1` | ตรวจ API จริง รวมตั้งค่า/pose ต้องมีหุ่น แม้ขั้นนี้ไม่ขยับ |
| `robot-test --steps 2` / `3` / `4` | ขยับจริง ต้องมีคนข้างหุ่น อ่าน [testing.md](../docs/testing.md) |
| `serial -BusId <BUSID>` | USB agent สำหรับเฟิร์มแวร์ serial transport เท่านั้น |
| Docker is not running | เปิด Docker Desktop รอ engine พร้อม |
| Python not found | ติดตั้ง Python ให้ PATH ถูก ปิด Store app aliases หากชี้ผิด |
| เปิดชื่อหุ่นไม่ได้ | `find` แล้วเปิด IP ตรวจ Wi-Fi วงเดียวกัน |
| ไม่พบหุ่น | ตรวจไฟ/Wi-Fi หรือ hotspot `mor-luam-XXXX` |
| OTA 403 | รหัสผิด; ถ้าหุ่นเคลื่อนที่ให้หยุดก่อน |
| ROS ไม่เชื่อม | ตรวจ agent, IP ของ PC, UDP 8888 และ domain 10 |

แผนที่โค้ด [CLAUDE.md](../CLAUDE.md), รวมเอกสาร [docs/README.md](../docs/README.md)
