# mor_luam บน Windows — คู่มือใช้งาน

> ติดตั้งและรันบน Ubuntu (Docker หรือไม่ใช้ Docker) และวิธีติดตั้งทุกอย่าง: `docs/install_and_run.md` · บน Ubuntu ใช้ `./docker/mor_luam.sh` แทน `docker\mor_luam.bat` (คำสั่งเหมือนกัน)

คำสั่งทั้งหมดพิมพ์ใน cmd หรือ PowerShell ที่โฟลเดอร์ `E:\GPS_Localize\old\mor_luam`
คำสั่งที่ใช้ Docker ต้องเปิด **Docker Desktop** ก่อน

## 1. ภาพรวม

```
คอม (Windows)  ปล่อย WiFi "manny" (Mobile Hotspot)            หุ่น mor_luam (ESP32)
┌───────────────────────────────────────┐   WiFi           ┌───────────────────────────────┐
│ เบราว์เซอร์ → http://mor-luam.local     │ ◀── เว็บแอป ───▶ │ เว็บแอป + เส้นทาง 2 มิติ (ในหุ่น) │
│ docker agent-wifi (UDP 8888)          │ ◀── micro-ROS ─▶ │ ROS 2 topics /mor_luam/...    │
│ docker\mor_luam.bat fw-ota            │ ─── OTA ───────▶ │ อัปเดตเฟิร์มแวร์ผ่าน WiFi        │
└───────────────────────────────────────┘                  └───────────────────────────────┘
```

- หุ่นจำ WiFi ได้ 6 วง ลำดับในรายการคือลำดับความสำคัญ ค่าเริ่มต้นคือ `manny` (hotspot ของคอมเครื่องนี้)
  - WiFi หลุด: ต่อวงเดิมก่อน ถ้าไม่เจอจะไล่วงที่ 1, 2, 3 … วงไหนต่อไม่ได้ก็ข้ามไปวงถัดไป
  - ต่อวงที่ 2 หรือ 3 อยู่ แล้ววงที่ 1 กลับมา: หุ่นย้ายไปวงที่ 1 เอง (เช็กทุก 30 วินาที เฉพาะตอนหุ่นหยุดนิ่ง)
- ต่อ WiFi ไม่ได้เกิน 20 วินาที → หุ่นเปิด hotspot ของตัวเองชื่อ `mor-luam-XXXX` ให้เข้า `http://192.168.4.1`
- รองรับ WPA3-SAE แบบ H2E ของ hotspot มือถือ: ตั้ง `WPA3_SAE_PWE_BOTH` ก่อนเริ่มต่อแต่ละวง โดยยังใช้ WPA2 ได้ตามเดิม
  ([Espressif: วิธีแก้ค่า SAE เริ่มต้น](https://documentation.espressif.com/AR2026-003_OTA_Bug_Advisory_for_WPA3-SAE_H2E_Configuration_Issues_in_ESP-IDF_EN.html))
- หุ่นประกาศตัวผ่าน mDNS: `mor-luam.local` และ `_module._tcp` (แบบเดียวกับโมดูลในโปรเจกต์ mice)
- วิ่งตามจุดจากเว็บ คำนวณในหุ่นเอง (Detour Steer) ไม่ต้องใช้ ROS

## 2. ครั้งแรก: แฟลชผ่านสาย USB

1. build:
   ```
   docker\mor_luam.bat fw-build
   ```
   ได้ไฟล์ใน `firmware\firmware_out\`
2. ดูว่า ESP32 อยู่ COM ไหน:
   ```
   docker\mor_luam.bat fw-flash
   ```
   หาบรรทัด `Silicon Labs CP210x ... (COMx)` (เครื่องนี้คือ COM9)
3. แฟลช:
   ```
   docker\mor_luam.bat fw-flash -Port COM9
   ```
   ถ้าค้างที่ `Connecting...` ให้กดปุ่ม BOOT บน ESP32 ค้างไว้จนเริ่มเขียน

ทำครั้งเดียว ครั้งต่อไปใช้ WiFi (ข้อ 5)

## 3. เปิดเว็บของหุ่น

**ง่ายที่สุด:** `docker\mor_luam.bat web` = หาหุ่น บอกที่อยู่ (คอม/มือถือ/WiFi ที่ต้องต่อ) แล้วเปิดเบราว์เซอร์ให้

> คู่มือเว็บฉบับเต็ม (ทุกแท็บ ทุกปุ่ม วิธีเปิดทุกแบบ): `docs/web_guide_th.md`

- เปิด `http://mor-luam.local` ในเบราว์เซอร์ (คอมหรือมือถือที่อยู่ WiFi เดียวกับหุ่น)
- ถ้าเปิดไม่ได้ ให้หา IP:
  ```
  docker\mor_luam.bat find
  ```
  แล้วเปิด `http://<IP>/`

### แท็บในเว็บ

| แท็บ | ทำอะไร |
|---|---|
| เส้นทาง | แตะบนระนาบ 2 มิติเพื่อวางจุด (หน่วยเมตร) ลากจุดเพื่อย้าย โหมด "ลบจุด" แตะเพื่อลบ แก้ตัวเลขในตารางได้ แล้วกด **เริ่มวิ่ง** |
| WiFi | ดูวงที่ต่ออยู่ เพิ่ม / แก้ / ลบ / เลื่อนลำดับ WiFi ดูชื่อและรหัส WiFi ที่บันทึก สแกนหา WiFi รอบตัว |
| ตั้งค่า | ชื่อหุ่น ชื่อ mDNS เลือก algorithm (Detour Steer / เลี้ยวตรง) ความเร็ว ระยะถึงจุด ROS agent รหัส OTA / hotspot และค่า PIDF |
| ระบบ | อัปเดตเฟิร์มแวร์ (OTA) หาหุ่นตัวอื่นในวง สถานะ ROS ข้อมูลระบบ รีบูต เปลี่ยนธีม |

ปุ่ม **หยุด (E-STOP)** อยู่มุมขวาบนทุกแท็บ

**ความปลอดภัยตอนวิ่งตามจุด:** หน้าเว็บส่งสัญญาณ "ยังดูอยู่" ทุก 1 วินาที
ถ้าปิดหน้าเว็บ มือถือจอดับ หรือ WiFi หลุดเกิน 3 วินาที หุ่นจะหยุดเอง

**คำเตือน:** แท็บ WiFi แสดงรหัส WiFi ทุกวง ทุกคนที่เปิดหน้าเว็บของหุ่นได้จะเห็นรหัสเหล่านี้
ใช้หุ่นในวง WiFi ที่ไว้ใจเท่านั้น และเปลี่ยนรหัส OTA / hotspot จากค่าเริ่มต้นในแท็บตั้งค่า

## 4. ตำแหน่งและทิศบนระนาบ

- (0,0) คือจุดที่หุ่นอยู่ตอนเปิดเครื่อง หรือตอนกด "ตั้งตรงนี้เป็น (0,0)"
- แกน +x คือทิศที่หุ่นหันตอนนั้น แกน +y คือทางซ้ายของหุ่น
- ลูกศรขาว = ตัวหุ่น เส้นเขียว = ทิศล้อ เส้นประส้ม = ขาที่กำลังวิ่ง

## 5. อัปเดตเฟิร์มแวร์ผ่าน WiFi (OTA)

1. build:
   ```
   docker\mor_luam.bat fw-build
   ```
2. ส่งเข้าหุ่น (ถามรหัส OTA):
   ```
   docker\mor_luam.bat fw-ota
   ```
   ถ้า `mor-luam.local` ใช้ไม่ได้ ใส่ IP จาก `docker\mor_luam.bat find` เช่น `docker\mor_luam.bat fw-ota -Host 192.168.137.212` (IP เปลี่ยนได้ทุกครั้งที่ต่อใหม่)

หรือใช้หน้าเว็บ แท็บ **ระบบ** → เลือก `firmware\firmware_out\firmware.bin` → ใส่รหัส OTA → อัปโหลด

หุ่นไม่ยอมอัปเดตตอนกำลังวิ่ง ต้องหยุดก่อน
รหัส OTA ค่าเริ่มต้นอยู่ใน `firmware\config
etwork_secrets.h` (`MORLUAM_DEFAULT_OTA_PASS`, ไฟล์นี้ไม่ขึ้น git) เปลี่ยนได้ในแท็บตั้งค่า

## 6. ROS 2 (สั่งหุ่นจากโปรแกรมบนคอม)

1. build image ครั้งแรก: `docker\mor_luam.bat build`
2. เปิด agent: `docker\mor_luam.bat wifi`
3. ดูหุ่นต่อติด: `docker\mor_luam.bat logs` (เห็น `session established`) หรือดูช่อง ROS ในเว็บ
4. ดู topic: `docker\mor_luam.bat topics`
5. สั่งหุ่น:
   ```
   docker\mor_luam.bat run drive_to_xy.py 2 0 --speed 0.25 --unit mps
   ```
6. เลิกใช้: `docker\mor_luam.bat stop`

- ROS_DOMAIN_ID = 10 ทั้งหุ่นและ Docker
- หุ่นหา agent เอง: ค่าในแท็บตั้งค่า → เครื่องที่ปล่อย WiFi (gateway) → `AGENT_IP` ใน `conf_network.h`
- คำสั่งจาก ROS ยกเลิกเส้นทางจากเว็บทันที
- ถ้าหุ่นต่อ agent ไม่ติด ให้เปิด UDP 8888 ใน Firewall (PowerShell แบบ Administrator ครั้งเดียว):
  ```
  New-NetFirewallRule -DisplayName "micro-ROS agent UDP 8888" -Direction Inbound -Protocol UDP -LocalPort 8888 -Action Allow
  ```

## 7. แก้โค้ด

แผนที่ไฟล์ทั้งหมดอยู่ที่ `CLAUDE.md` (โฟลเดอร์บนสุด)

| อยากแก้ | ไฟล์ | ทดสอบ |
|---|---|---|
| algorithm เลี้ยว / วางเส้นทาง | `firmware\src\algorithm\` (อ่าน README.md ในโฟลเดอร์นั้น) | `docker\mor_luam.bat fw-test` |
| ค่า PID | `firmware\config\PIDF_config.h` หรือแท็บตั้งค่า (ชั่วคราวจนรีบูต) | ลองบนหุ่น |
| หน้าเว็บ | `firmware\web\` (index.html, app.css, js\) | `docker\mor_luam.bat web-mock` แล้วเปิด `http://localhost:8000` |
| ค่าตั้งต้น WiFi | `firmware\config\network_secrets.h` (ไม่ขึ้น git) | – |

หลังแก้: `fw-build` แล้ว `fw-ota`

## 8. คำสั่งทั้งหมด

| คำสั่ง | ทำอะไร |
|---|---|
| `fw-build` | build เฟิร์มแวร์ → `firmware\firmware_out\` |
| `fw-flash -Port COM9` | แฟลชผ่าน USB จาก Windows ตรง ๆ |
| `fw-ota [-Host IP]` | อัปเดตผ่าน WiFi |
| `fw-test` | ทดสอบ algorithm กับ PIDF บนคอม |
| `fw-clean-ros` | ลบ micro-ROS library ให้ build ใหม่ (หลังแก้ `firmware\microros.meta`) |
| `fw-upload -BusId 1-3` / `fw-monitor -BusId 1-3` | แฟลช / ดู serial ผ่าน WSL (usbipd) |
| `find` | หาหุ่นในวง WiFi |
| `web-mock` | หุ่นจำลองสำหรับแก้หน้าเว็บ |
| `build` / `wifi` / `logs` / `topics` / `run` / `shell` / `stop` | ฝั่ง ROS 2 |

## 9. ปัญหาที่เจอบ่อย

| อาการ | แก้ |
|---|---|
| `fw-flash` เห็นแต่ COM ของ Bluetooth | สาย USB อาจชาร์จได้อย่างเดียว หรือยังไม่มีไดรเวอร์ CP210x |
| เปิด `mor-luam.local` ไม่ได้ | ใช้ `find` แล้วเปิดด้วย IP |
| `find` ไม่เจอหุ่น | หุ่นอาจเปิด hotspot `mor-luam-XXXX` อยู่: ต่อวงนั้นแล้วเปิด `http://192.168.4.1` |
| `fw-ota` ขึ้น 403 | รหัส OTA ผิด |
| `fw-ota` ขึ้นว่าหุ่นกำลังเคลื่อนที่ | กดหยุดในเว็บก่อน |
| เว็บขึ้นแถบแดง "ติดต่อหุ่นไม่ได้" | หุ่นดับ / รีบูต / หลุด WiFi: รอ หน้าเว็บต่อใหม่เอง |
| ช่อง ROS ขึ้น "รอ agent" | ยังไม่ได้ `docker\mor_luam.bat wifi` หรือ Firewall ปิด UDP 8888 |
| `Docker is not running` | เปิด Docker Desktop รอจนขึ้น Engine running |
