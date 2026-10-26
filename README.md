# mor_luam — หุ่นล้อเดียว ESP32 + เว็บ + Docker + ROS 2

หุ่นใช้มอเตอร์ตัวเดียว: หมุนทิศหนึ่งเพื่อขับล้อ อีกทิศเพื่อเลี้ยว จึงหยุดขับก่อนเลี้ยว
ESP32 ควบคุมทุก 10 ms และคำนวณตำแหน่งจาก encoder กับ IMU BNO085
เปิดเว็บเพื่อวางเส้นทาง จำลอง Direct/Detour ตั้งค่า PID/Wi-Fi และอัปเดตเฟิร์มแวร์ได้
ROS 2 Jazzy เป็นทางเลือกสำหรับสั่งจากโปรแกรมบนคอม

## อยู่ที่ไหนในเครื่องนี้

โฟลเดอร์โค้ดจริงที่ตรวจพบวันที่ **26 ตุลาคม 2026**:

```text
E:\GPS_Localize\old\mor_luam
```

เชื่อมกับ [GitHub นี้](https://github.com/Mannaja78520/mor_luam)
ส่วนโฟลเดอร์แชต Codex เป็นพื้นที่ของแชต เครื่องอื่นจะ clone ไว้ที่ใดก็ได้
คำสั่งทั้งหมดด้านล่างให้รันจากโฟลเดอร์โปรเจกต์

## เปิดใช้งานบน Windows

ถ้าหุ่นมีเฟิร์มแวร์แล้ว เปิดหุ่น ให้คอมอยู่ Wi-Fi เดียวกัน แล้วเปิด PowerShell:

```powershell
Set-Location E:\GPS_Localize\old\mor_luam
.\docker\mor_luam.bat web
```

`web` หาหุ่น แสดงที่อยู่สำหรับคอม/มือถือ แล้วเปิดเบราว์เซอร์
หรือเปิด [mor-luam.local](http://mor-luam.local/) โดยตรง
เครื่องนี้เคยใช้ Mobile Hotspot ชื่อ `manny` หากชื่อเปิดไม่ได้ให้หา IP ปัจจุบัน:

```powershell
.\docker\mor_luam.bat find
```

IP เปลี่ยนได้ ไม่ควรยึด IP จากบันทึกเก่า มือถือที่เปิด `.local` ไม่ได้ให้ใช้ IP
หากหุ่นต่อ Wi-Fi ไม่ได้ จะเปิด hotspot `mor-luam-XXXX`
ให้ต่อวงนั้นแล้วเปิด [192.168.4.1](http://192.168.4.1/)

### คำสั่งไหนต้องเปิด Docker

`mor_luam.bat` เป็นตัวรวมคำสั่ง บางคำสั่งใช้ Python บน Windows โดยตรง
เว็บจริงอยู่บน ESP32 จึงเปิดได้โดยไม่ต้องเปิด ROS agent

| งาน | คำสั่งหลัง `.\docker\mor_luam.bat` | ต้องเปิด Docker Desktop |
|---|---|---|
| เปิดเว็บ / หาหุ่น | `web` / `find` | ไม่ต้อง; ใช้ Python บนคอม |
| หุ่นจำลองบนคอม | `web-mock` | ไม่ต้อง; ใช้ Python บนคอม |
| สร้าง images ครั้งแรก | `build` | ต้อง |
| ทดสอบ C++ / สร้างเฟิร์มแวร์ | `fw-test` / `fw-build` | ต้อง |
| ส่งไฟล์ที่ build แล้วผ่าน Wi-Fi | `fw-ota` | ไม่ต้อง; ใช้ Windows curl |
| แฟลชไฟล์ที่ build แล้วผ่าน USB | `fw-flash -Port COM9` | ไม่ต้อง; ต้องมี PlatformIO/esptool |
| ROS agent / log / topics | `wifi` / `logs` / `topics` | ต้อง |
| โปรแกรม ROS / terminal / ปิด containers | `run` / `shell` / `stop` | ต้อง |

ดูทั้งหมดด้วย `.\docker\mor_luam.bat help`

## ติดตั้งครั้งแรก / สร้างและอัปเดตเฟิร์มแวร์

ติดตั้ง Git, Python และ Docker Desktop (WSL 2 backend)
อ่าน [คู่มือติดตั้ง Windows/Ubuntu](docs/install_and_run.md) สำหรับรายละเอียด
สำหรับเครื่องใหม่:

```powershell
git clone https://github.com/Mannaja78520/mor_luam.git
Set-Location mor_luam
Copy-Item firmware\config\network_secrets.example.h firmware\config\network_secrets.h
```

แก้ `network_secrets.h` ในเครื่องเพื่อใส่ Wi-Fi และรหัส OTA/hotspot ของคุณ
ไฟล์นี้ถูก Git ignore อย่าคัดลอกทับไฟล์เดิมในเครื่องที่ตั้งค่าไว้แล้ว
เปิด Docker Desktop รอ engine พร้อม จากนั้น:

```powershell
.\docker\mor_luam.bat build
.\docker\mor_luam.bat fw-test
.\docker\mor_luam.bat fw-build
```

ไฟล์อยู่ใน `firmware\firmware_out\` หน้าเว็บถูกรวมเข้าเฟิร์มแวร์ตอน build
หยุดหุ่นก่อนอัปเดตผ่าน Wi-Fi:

```powershell
.\docker\mor_luam.bat fw-ota
```

ระบบถามรหัส OTA แบบไม่แสดงข้อความ ใช้ `-Host <IP>` หากชื่อ `.local` ใช้ไม่ได้
หรือเว็บแท็บ **ระบบ** → เลือก `firmware\firmware_out\firmware.bin` → ใส่รหัส → อัปโหลด
ครั้งแรกที่ ESP32 ยังไม่มีเฟิร์มแวร์ ให้ build แล้วใช้ USB:

```powershell
.\docker\mor_luam.bat fw-flash -Port COM9
```

`COM9` เป็นพอร์ตที่เคยใช้ ตรวจพอร์ตจริงใน Device Manager ก่อน

## เว็บ เส้นทาง และเดโม

| ส่วน | ใช้ทำอะไร |
|---|---|
| เส้นทาง | วาง/ลาก/ลบจุด แก้ x/y และเวลารอ `waitS` 0–60 s บันทึกเส้นทางหน้าเว็บหรือปุ่ม 1/2 |
| จำลอง | เทียบ Direct กับ Detour พร้อมความคลาดเคลื่อนจำลอง; ไม่ส่งคำสั่งมอเตอร์ |
| เดโม 3/4 | ภาพเส้นทาง เส้นเวลาหมุน/ขับ และปุ่มทดสอบจริง 3 รอบ (6 runs) |
| Wi-Fi | จัดลำดับเครือข่าย สแกน และเชื่อมต่อ |
| ตั้งค่า | ความเร็ว/ระยะถึงจุด, algorithm, PIDF, ตัวเลือกเดโมและการเลี้ยว |
| ระบบ | OTA, ROS agent, สถานะระบบ และรีบูต |

ตำแหน่งมีหน่วยเมตร: +x คือด้านหน้าตอนตั้ง pose, +y คือซ้าย และมุมบวกทวนเข็มนาฬิกา
ล้อเลี้ยวได้ทิศเดียว **ทวนเข็มนาฬิกาเมื่อมองจากด้านบน**
แท็บเส้นทางตั้งตรงนี้เป็น (0,0) หรือระบุ x/y/heading ได้เมื่อหยุด
ตำแหน่งจาก encoder/IMU เป็น odometry จึงยังคลาดเคลื่อนได้เมื่อไถล

ปุ่มบนหุ่น (GPIO19 ลง GND):

| กด | ผล |
|---|---|
| 1 / 2 ครั้ง | วิ่งเส้นทางปุ่ม 1 / 2 ที่ตั้งในเว็บ แล้วกลับจุดเริ่ม |
| 3 / 4 ครั้ง | Direct / Detour: ค่าเริ่มเป้า 0.30 m เยื้องขวา 6°, รอ 5 s แล้วกลับ |
| กดขณะเคลื่อนที่ | หยุด |
| ค้างเกิน 1.5 s | ไม่เริ่มเดโม |

เริ่มหลังคลิกสุดท้ายประมาณ 1 s ค่าเริ่มตั้งตำแหน่งปัจจุบันเป็น (0,0) ทุกครั้งที่เริ่มจากปุ่ม
ปรับระยะ/มุมและการ reset ได้ในตั้งค่า อ่าน [คู่มือเว็บภาษาไทย](docs/web_guide_th.md)
ก่อนใช้ปุ่มวิ่งจริง

ลองเว็บโดยไม่มีหุ่น:

```powershell
.\docker\mor_luam.bat web-mock
```

เปิด [localhost:8000](http://localhost:8000/) กด Ctrl+C ใน terminal เพื่อปิด mock

## ROS 2 ผ่าน Docker (ทางเลือก)

เปิด Docker Desktop แล้วรัน:

```powershell
.\docker\mor_luam.bat build
.\docker\mor_luam.bat wifi
.\docker\mor_luam.bat topics
```

ดูการเชื่อมต่อด้วย `logs` (Ctrl+C เลิกดู log; agent ยังทำงาน)
ROS domain **10**, agent UDP **8888** หากคอมไม่ใช่ gateway ของหุ่น
ให้ใส่ IP ของคอมในช่อง micro-ROS agent ในเว็บ

ตัวอย่างนี้ **ทำให้หุ่นเคลื่อนที่** ใช้เมื่อมีคนอยู่ข้างหุ่นและพื้นที่พร้อมเท่านั้น:

```powershell
.\docker\mor_luam.bat run drive_to_xy.py 0.3 0 --speed 0.03 --unit mps
```

`/mor_luam/cmd_vel/move` ใช้ `geometry_msgs/Twist`: `linear.x` เป็น **RPM**
และ `angular.z` เป็น **มุมเป้าล้อหน่วยองศา** (0–360) ซึ่งต่างจาก Twist มาตรฐาน
ตำแหน่งจาก ESP32 อยู่ที่ `/mor_luam/odom/esp`
คำสั่ง ROS เข้าควบคุมแทนเส้นทางเว็บ จึงควรใช้แหล่งคำสั่งเดียวขณะทดสอบ

## หยุดและข้อจำกัด

- **E-STOP** บนเว็บ หรือ `curl.exe -X POST http://mor-luam.local/api/estop`
- `docker\mor_luam.bat stop` ปิด containers; **ไม่ใช่ E-STOP ของหุ่น** หยุดหุ่นก่อน
- เว็บ route/test ใช้ watchdog 3 s เมื่อไม่มี heartbeat/status polling;
  หน้าเว็บอื่นยังเลี้ยงสัญญาณได้ เดโมจากปุ่มมีขีดจำกัดเวลาเคลื่อนที่แยก
- ไม่มี hardware E-STOP input และไม่มี sensor ยืนยันล้อแตะพื้น ต้องมีคนอยู่ข้างหุ่นและตัดไฟมอเตอร์ได้
- ความเร็วทดสอบประมาณ 0.03 m/s เต็มกำลังประมาณ 0.039 m/s จึงเลิกใช้ตัวอย่างเก่า 0.25 m/s / 120 RPM
- หน้า Wi-Fi/ข้อมูล settings มีรหัสลับ ใช้ในเครือข่ายที่ไว้ใจ อย่านำรหัสขึ้น Git หรือโพสต์ log

อ่าน [ความปลอดภัย](docs/safety.md) และ [วิธีทดสอบ](docs/testing.md) ก่อนสั่งมอเตอร์

## เอกสารและโค้ด

| ต้องการ | อ่าน |
|---|---|
| คำสั่ง Windows / Docker ทั้งหมด | [Docker README](docker/README.md) |
| ติดตั้ง Windows / Ubuntu / แบบไม่ใช้ Docker | [ติดตั้งและรัน](docs/install_and_run.md) |
| เว็บทุกแท็บและเดโม | [คู่มือเว็บภาษาไทย](docs/web_guide_th.md) |
| รวมเอกสาร เทคนิค หลักฐานทดสอบ | [Documentation map](docs/README.md) |
| Direct/Detour และการเพิ่ม planner | [Algorithm README](firmware/src/algorithm/README.md) |
| แผนที่ source | [CLAUDE.md](CLAUDE.md) |
| งานเอกสารล่าสุด 26 ต.ค. 2026 | [Codex V2](HANDOFF_MORLUAM_CODEX_V2.md) |
| ประวัติเฟิร์มแวร์/ทดสอบจริง 8–10 ต.ค. | [Claude V2](HANDOFF_MORLUAM_CLAUDE_V2.md) |

แก้เว็บที่ `firmware/web/`; `firmware/src/web/WebPage.h` สร้างอัตโนมัติตอน build
หลังแก้ firmware/เว็บ: `fw-test` → `fw-build` → หยุดหุ่น → `fw-ota`
บน Ubuntu ใช้ `./docker/mor_luam.sh <command>` แทนตัวช่วย `.bat`
