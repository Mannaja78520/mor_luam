# คู่มือการติดตั้งและใช้งาน mor_luam สำหรับทุกเครื่อง (Multi-Machine Setup)

คู่มือนี้จัดทำขึ้นเพื่อให้สามารถนำโค้ดไป **ติดตั้ง รัน และใช้งานบนคอมพิวเตอร์เครื่องใหม่เครื่องใดก็ได้** (Windows 10/11, Ubuntu Linux) โดยไม่ต้องแก้ไขโค้ดหรือผูกติดกับเครื่องใดเครื่องหนึ่ง

---

## 1. สิ่งที่ต้องติดตั้ง (Prerequisites)

แบ่งตามรูปแบบการใช้งานที่คุณต้องการ:

### แบบที่ 1: ใช้งาน Web UI / จำลองเส้นทาง (Simulator) / ค้นหาหุ่น (ง่ายที่สุด ไม่ต้องใช้ Docker)
ใช้สำหรับวางเส้นทาง (Waypoints), จำลอง Direct/Detour, ตรวจสอบสถานะหุ่น, สั่งการหุ่นผ่านเว็บ:
1. **Git**: [https://git-scm.com/download/win](https://git-scm.com/download/win)
2. **Python 3.10 – 3.12**: [https://www.python.org/downloads/](https://www.python.org/downloads/)
   - ⚠️ ตอนติดตั้งให้ติ๊กถูก **"Add python.exe to PATH"**
   - บน Windows: ให้ปิด Store Aliases โดยไปที่ **Windows Settings → Apps → Advanced app settings → App execution aliases** แล้วปิดสวิตช์ที่เป็น `python.exe` และ `python3.exe` (เพื่อไม่ให้เด้งไป Microsoft Store)

### แบบที่ 2: คอมไพล์เฟิร์มแวร์ ESP32 / micro-ROS (ถ้าต้องการแก้โค้ด C++ หรือใช้ ROS 2)
1. **Docker Desktop** (สำหรับ Windows ใช้ WSL 2 backend): [https://www.docker.com/products/docker-desktop/](https://www.docker.com/products/docker-desktop/)
   - หลังติดตั้ง ให้เปิด Docker Desktop รอจนขึ้นสถานะ **"Engine running"** (สีเขียวมุมซ้ายล่าง)
2. (สำหรับแฟลชบอร์ดผ่าน USB ครั้งแรก): ติดตั้ง **VS Code** พร้อม Extension **PlatformIO** หรือติดตั้งไดรเวอร์ USB ชิป CP210x / CH340

---

## 2. ขั้นตอนการโคลนและเตรียมไฟล์โปรเจกต์ (ทุกเครื่องต้องทำ)

### 2.1 Clone Repository ลงเครื่อง
เปิด PowerShell (Windows) หรือ Terminal (Ubuntu) แล้ว clone ไปไว้ที่โฟลเดอร์หรือไดรฟ์ใดก็ได้ตามต้องการ:

```powershell
git clone https://github.com/Mannaja78520/mor_luam.git
cd mor_luam
```

### 2.2 สร้างไฟล์ตั้งค่า Wi-Fi (`network_secrets.h`)
ไฟล์รหัสผ่านจะไม่ถูกอัปโหลดขึ้น GitHub เพื่อความปลอดภัย ให้คัดลอกไฟล์ต้นแบบมาใช้งาน:

- **บน Windows (PowerShell):**
  ```powershell
  Copy-Item firmware\config\network_secrets.example.h firmware\config\network_secrets.h
  ```
- **บน Ubuntu / Linux:**
  ```bash
  cp firmware/config/network_secrets.example.h firmware/config/network_secrets.h
  ```

เปิดไฟล์ `firmware/config/network_secrets.h` ด้วยโปรแกรมแก้ไขข้อความ (เช่น VS Code, Notepad) เพื่อใส่ชื่อ Wi-Fi และรหัสผ่าน:

```c
#pragma once
static const WifiSeed WIFI_SEED[] = {
    {"ชื่อ_WIFI_บ้านหรือออฟฟิศ", "รหัสผ่าน_WIFI"},
    {"ชื่อ_Hotspot_มือถือ_หรือคอม", "รหัสผ่าน_Hotspot"},
};

#define MORLUAM_DEFAULT_AP_PASS "12345678"     // รหัส Hotspot สำรองของหุ่น (8 ตัวขึ้นไป)
#define MORLUAM_DEFAULT_OTA_PASS "morluam123"  // รหัสสำหรับอัปเดตเฟิร์มแวร์ผ่าน Wi-Fi
```
*หุ่นจะพยายามเชื่อมต่อตามลำดับเครือข่ายที่คุณระบุไว้ข้างต้น*

---

## 3. วิธีเปิดใช้งาน Web UI และระบบจำลองบนคอมพิวเตอร์ (ไม่ต้องใช้ Docker)

ระบบมีเซิร์ฟเวอร์คอมพาเนียนในตัวที่สามารถรันได้ทันทีด้วย Python:

### 3.1 เริ่มเซิร์ฟเวอร์เว็บ
ในโฟลเดอร์โปรเจกต์ ให้รันคำสั่ง:

```powershell
python tools/pc_web.py
```
หรือใช้คำสั่งผ่านตัวช่วย (บน Windows):
```powershell
.\docker\mor_luam.bat web
```
*(บน Ubuntu ใช้ `./docker/mor_luam.sh web`)*

### 3.2 การเปิดหน้าเว็บ
- เบราว์เซอร์จะเปิดไปที่ [http://localhost:8000/](http://localhost:8000/) อัตโนมัติ
- **เมื่อไม่มีหุ่น (Offline Simulator)**: สามารถคลิกวางจุดพิกัด, ลากจุด, ทดสอบจำลองการวิ่งแบบ Direct vs Detour Steer, คำนวณระยะทางและเวลาได้ทันที
- **เมื่อเปิดหุ่น (Auto-Discovery)**: เมื่อเปิดสวิตช์หุ่นและหุ่นเกาะอยู่ในวง Wi-Fi เดียวกัน โปรแกรมจะสแกนหาหุ่นทุก Subnet ที่คอมพิวเตอร์เชื่อมต่ออยู่โดยอัตโนมัติ (ไม่ว่าจะเป็น 192.168.1.x, 192.168.137.x หรือวงอื่น) และแถบสถานะจะเปลี่ยนเป็น **"เชื่อมหุ่นจริง (IP: ...)"** ทันทีโดยไม่ต้องตั้งค่าใดๆ เพิ่มเติม

---

## 4. การค้นหาและเข้าถึงหุ่นโดยตรงผ่าน Wi-Fi

หากต้องการตรวจสอบว่าหุ่นได้รับ IP อะไรในเครือข่ายปัจจุบัน:

### 4.1 ค้นหา IP ของหุ่น
```powershell
python tools/find_robots.py
```
หรือ
```powershell
.\docker\mor_luam.bat find
```
โปรแกรมจะสแกนหาและแสดงชื่อ IP, mDNS และเวอร์ชันเฟิร์มแวร์ของหุ่นทุกตัวในวง

### 4.2 เปิดเว็บที่ฝังอยู่ในตัว ESP32 โดยตรง
- พิมพ์ URL: [http://mor-luam.local/](http://mor-luam.local/)
- หรือเปิดผ่าน IP ที่ค้นพบ เช่น `http://192.168.1.50/`
- หรือรันคำสั่ง: `python tools/find_robots.py --open`

### 4.3 กรณีไม่มีเราเตอร์ Wi-Fi ในสถานที่ (Standalone Hotspot)
- หากเปิดหุ่นแล้วไม่พบสัญญาณ Wi-Fi ที่บันทึกไว้ภายใน 20 วินาที หุ่นจะปล่อย Hotspot ของตัวเองชื่อ:
  `mor-luam-XXXX` (XXXX คือรหัสประจำตัวหุ่น)
- ให้ใช้คอมพิวเตอร์หรือมือถือเชื่อมต่อเข้ากับ Wi-Fi `mor-luam-XXXX` ดังกล่าว (รหัสผ่านตามที่ตั้งใน `MORLUAM_DEFAULT_AP_PASS`)
- จากนั้นเปิดเบราว์เซอร์ไปที่: [http://192.168.4.1/](http://192.168.4.1/) เพื่อควบคุมหุ่นหรือเพิ่มเครือข่าย Wi-Fi ใหม่ได้ทันที

---

## 5. การคอมไพล์และอัปเดตเฟิร์มแวร์ (ด้วย Docker)

เมื่อต้องการปรับปรุงโค้ดเฟิร์มแวร์ C++ ในโฟลเดอร์ `firmware/src/`:

### 5.1 คอมไพล์เฟิร์มแวร์
เปิด Docker Desktop แล้วรัน:
```powershell
.\docker\mor_luam.bat fw-test    # รัน Unit tests ของระบบนำทางและคณิตศาสตร์
.\docker\mor_luam.bat fw-build   # คอมไพล์เฟิร์มแวร์ ESP32 (ได้ไฟล์ใน firmware/firmware_out/)
```

### 5.2 อัปเดตเฟิร์มแวร์ผ่าน Wi-Fi (OTA Update)
เมื่อคอมไพล์เสร็จแล้ว สามารถส่งไฟล์เฟิร์มแวร์เข้าหุ่นผ่าน Wi-Fi ได้ทันทีโดยไม่ต้องเสียบสาย USB:
```powershell
.\docker\mor_luam.bat fw-ota
```
*(ระบบจะถามรหัสผ่าน OTA ตามที่ตั้งใน `network_secrets.h` และทำการอัปโหลด จากนั้นหุ่นจะรีบูตตัวเองประมาณ 10 วินาที)*
หรือจะเข้าไปที่หน้าเว็บแท็บ **ระบบ** → เลือกไฟล์ `firmware\firmware_out\firmware.bin` → กรอกรหัส OTA → กดอัปโหลด ก็ได้เช่นกัน

### 5.3 แฟลชผ่านสาย USB (สำหรับ ESP32 ตัวใหม่ที่ยังไม่เคยลงเฟิร์มแวร์)
1. เสียบสาย USB เข้าคอมพิวเตอร์ ตรวจสอบพอร์ตใน Device Manager (เช่น `COM5`)
2. สั่งแฟลช:
   ```powershell
   .\docker\mor_luam.bat fw-flash -Port COM5
   ```

---

## 6. การใช้งาน micro-ROS และ ROS 2 (ทางเลือก)

หากต้องการส่งคำสั่ง Velocity (`cmd_vel`) หรือรับค่า Odometry ผ่านระบบ ROS 2:
1. เปิด Docker Desktop
2. เริ่ม micro-ROS Agent:
   ```powershell
   .\docker\mor_luam.bat wifi
   ```
3. ตรวจสอบหัวข้อ Topics:
   ```powershell
   .\docker\mor_luam.bat topics
   ```
   *(จะพบ `/mor_luam/cmd_vel/move`, `/mor_luam/odom/esp` ฯลฯ)*

> [!NOTE]
> ค่าเริ่มต้นของ ROS_DOMAIN_ID คือ `10` และส่งข้อมูลผ่านพอร์ต UDP `8888`

---

## 7. วิธีแก้ปัญหาที่พบบ่อย (Troubleshooting)

### ปัญหาที่ 1: หน้าเว็บขึ้นแถบสีเหลือง "หุ่นยังไม่เชื่อม · จำลองบนคอมได้ตามปกติ"
- **สาเหตุ:** คอมพิวเตอร์ยังหาหุ่นไม่เจอ หรือหุ่นยังไม่ได้เชื่อมต่อ Wi-Fi เดียวกัน
- **วิธีแก้:**
  1. ตรวจสอบว่าหุ่นเปิดสวิตช์เปิดเครื่องอยู่หรือไม่
  2. รัน `python tools/find_robots.py` ใน Terminal ดูว่าพบ IP ของหุ่นหรือไม่
  3. หากพบ IP แล้วแต่หน้าเว็บยังขึ้นสีเหลือง ให้กด **Refresh (F5 หรือ Ctrl + F5)** บนหน้าเบราว์เซอร์ 1 ครั้งเพื่อให้หน้าเว็บอ่านสถานะล่าสุด

### ปัญหาที่ 2: พิมพ์ `python` แล้วเด้งไปที่ Microsoft Store
- **วิธีแก้:** ไปที่ **Windows Settings → Apps → Advanced app settings → App execution aliases** แล้วเลื่อนปิดสวิตช์ `python.exe` และ `python3.exe`

### ปัญหาที่ 3: `Docker is not running`
- **วิธีแก้:** ให้เปิดโปรแกรม **Docker Desktop** แล้วรอ 1-2 นาทีจนกระทั่งไอคอนด้านล่างซ้ายเปลี่ยนเป็นสีเขียว "Engine running" ก่อนรันคำสั่ง Docker

### ปัญหาที่ 4: เสียบสาย USB แล้วไม่ขึ้น COM Port
- **วิธีแก้:**
  1. สาย USB ต้องเป็นสาย Data (ไม่ใช่สายชาร์จไฟอย่างเดียว)
  2. ติดตั้งไดรเวอร์ [Silicon Labs CP210x](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers) หรือ CH340 ตามชิปบนบอร์ด ESP32

---

## 8. สรุปคำสั่งที่ใช้งานบ่อย

| คำสั่ง | ผลลัพธ์ | ต้องเปิด Docker ไหม |
|---|---|---|
| `python tools/pc_web.py` หรือ `.\docker\mor_luam.bat web` | เปิดเว็บจำลองเส้นทางบนคอม และเชื่อมต่อหุ่นอัตโนมัติ | ❌ ไม่ต้อง |
| `python tools/find_robots.py` หรือ `.\docker\mor_luam.bat find` | สแกนหา IP ของหุ่นทุกตัวในวง Wi-Fi | ❌ ไม่ต้อง |
| `.\docker\mor_luam.bat fw-build` | คอมไพล์เฟิร์มแวร์ ESP32 | ✅ ต้องเปิด |
| `.\docker\mor_luam.bat fw-ota` | อัปเดตเฟิร์มแวร์ผ่าน Wi-Fi | ❌ ไม่ต้อง (ใช้ curl) |
| `.\docker\mor_luam.bat fw-flash -Port COMx` | แฟลชเฟิร์มแวร์ผ่านสาย USB | ❌ ไม่ต้อง (ใช้ PlatformIO) |
| `.\docker\mor_luam.bat wifi` | รัน micro-ROS Agent (UDP 8888) | ✅ ต้องเปิด |
