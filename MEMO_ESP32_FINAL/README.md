# 🤖 MEMO ESP32-S3 AI DESK ROBOT — PRODUCTION FINAL FIRMWARE

Canonical Firmware Package: `MEMO_ESP32_FINAL`  
Target Module: **ESP32-S3-N16R8-V2775** (16MB Flash, 8MB Octal OPI PSRAM)  
Framework: **Arduino ESP32 Core 3.3.13** (ESP-IDF 5.x)  

---

## 1. Hardware Specifications & GPIO Mapping

All original hardware pin assignments have been strictly preserved:

| Component | Interface / Pins | Details |
|---|---|---|
| **Display** | SPI (ILI9341 320x240 Landscape) | MOSI 11, MISO 13, SCK 12, CS 10, DC 9, RST 14, BCKL 47 |
| **Touchscreen** | SPI (XPT2046 Resistive) | MOSI 11, MISO 13, SCK 12, CS 4, IRQ 3 |
| **Microphone** | I2S_NUM_0 (INMP441 MEMS) | SCK 16, WS 17, SD 15 (16kHz 16-bit Mono) |
| **Speaker** | I2S_NUM_1 (MAX98357A Class-D) | BCLK 5, LRC 6, DIN 7 (24kHz 16-bit Mono) |
| **Head Touch** | GPIO 38 (TTP223 Capacitive) | Active HIGH with internal pull-down; Barge-In enabled |
| **Volume Up** | GPIO 41 (Physical Button) | Active LOW with internal pull-up |
| **Volume Down** | GPIO 42 (Physical Button) | Active LOW with internal pull-up |
| **RGB LED** | GPIO 48 (WS2812 NeoPixel) | Status indication |
| **RFID Reader** | UART1 HSU (PN532 Module) | RX GPIO 1, TX GPIO 2 (115200 baud, SEL0=0, SEL1=0) |

---

## 2. Key Architecture Fixes Implemented

### Fix A — Reliable WakeNet9 Re-arm & Disarm
- **`activeListening` Thread-Safety**: Replaced raw volatile boolean with `std::atomic<bool>` using explicit memory order barriers (`memory_order_release` / `memory_order_acquire`).
- **Comprehensive State Transitions**: WakeNet9 is automatically re-armed (`setListening(true)`) upon every transition into `RobotState::IDLE` (including normal playback buffer drain, thinking timeout, server cancel, or disconnect).
- **Echo & Loop Prevention**: WakeNet is immediately disarmed during voice capture (`LISTENING`), cloud query (`THINKING`), and speaker audio output (`SPEAKING`).

### Fix B — TTP223 Head Touch Barge-In Interruption
- **Immediate Cancellation**: Touching the TTP223 head sensor while the robot is in `SPEAKING` or `THINKING` state immediately stops and flushes speaker audio output (`stopAndFlush()`), sends an `audio:cancel` control frame over WebSocket to abort backend LLM/TTS generation, and transitions directly into `LISTENING` mode so the user can speak immediately.

### Fix C — Audio Output Stability & Protection
- **Decoupled FreeRTOS Architecture**: Dedicated `taskAudio` on Core 1 (Priority 3) acts as the single writer to `I2S_NUM_1`.
- **256KB Octal OPI PSRAM RingBuffer**: Smoothly absorbs cloud network jitter without dropping frames.
- **Watermark Flow Control**: Automatically issues `audio:flow pause` at 75% buffer occupancy and `audio:flow resume` at 40% occupancy, tracked inside `AudioOutputDriver::update()`.
- **Hardware Protection**: ESP32 brownout detection (`RTC_CNTL_BROWN_OUT_REG`) and hardware watchdogs are strictly enabled.

### Fix D — Official Partitioning & Model Layout
- **Partition Scheme**: `esp_sr_16` (`ESP SR 16M (3MB APP/6MB SPIFFS/3.9MB MODEL)`).
- **Flash Offsets**:
  - `0x010000`: Application Firmware (`MEMO_ESP32_FINAL.ino`)
  - `0xC10000`: ESP-SR WakeNet9 Model Partition (`srmodels.bin`, 3.3MB)
- **Built-in Wake Word**: Official Espressif `wn9_hiesp` ("Hi ESP" test model; target custom phrase "Hey Memo" clearly isolated for future model training).

### Feature E — RFID Learning Cards (5 Educational Items)
- **Hardware UART1**: Fast non-blocking ISO14443A card polling supporting 4, 7, and 10-byte UIDs with uppercase hex normalization and 2.5s presence debouncing.
- **WebSocket Protocol**:
  - Outgoing event: `rfid:scan` -> `{ "cardUid": "04A1B2C3D4" }`
  - Backend response: `rfid:result` -> `{ "cardUid": "...", "found": true, "itemKey": "apple", "nameEn": "Apple", "nameMm": "ပန်းသီး" }`
- **5 Educational Items**:
  1. `apple` — Apple / ပန်းသီး (Cute red apple vector graphic with leaf & shine)
  2. `elephant` — Elephant / ဆင် (Baby blue elephant with big ears, tusks & curved trunk)
  3. `cat` — Cat / ကြောင် (Warm orange kitten face with pointy ears & whiskers)
  4. `dog` — Dog / ခွေး (Friendly golden puppy face with floppy ears & tongue)
  5. `banana` — Banana / ငှက်ပျောသီး (Bright yellow curved banana bundle)
- **Crisp Myanmar Typography**: Pre-rendered 1-bit bitmap matrix (`myanmar_bitmaps.h`) generated from Windows official Myanmar Text font ensures 100% correct glyph shaping, subjoined consonants, and tone marks on ILI9341 without rendering boxes.
- **Display Duration**: Shows the educational learning card for 6 seconds, then automatically returns to animated robot eyes (or immediately upon screen tap).

---

## 3. Flashing & Upload Instructions

### Step 1: Flash WakeNet Model Data (One-Time Setup)
Connect your ESP32-S3 board via USB. Note the COM port (e.g., `COM3`).
Double-click `flash_model.bat` or run:
```cmd
flash_model.bat COM3
```
This writes `srmodels.bin` to offset `0xC10000`.

### Step 2: Upload Firmware via Arduino IDE 2.x
1. Open `MEMO_ESP32_FINAL.ino` in Arduino IDE.
2. Select **Tools** menu settings:
   - **Board**: `ESP32S3 Dev Module`
   - **Flash Size**: `16MB (128Mb)`
   - **Flash Mode**: `QIO 80MHz`
   - **PSRAM**: `OPI PSRAM`
   - **Partition Scheme**: `ESP SR 16M (3MB APP/6MB SPIFFS/3.9MB MODEL)`
   - **USB CDC On Boot**: `Enabled`
   - **Upload Speed**: `921600`
3. Click **Upload**.

---

## 4. MongoDB RFID Card Registration Guide

Do NOT invent card UIDs. Scan your physical MIFARE / NTAG cards on the robot. The serial monitor and touchscreen will output the exact uppercase UID.

Then register the card in MongoDB (database: `memo_robot`, collection: `rfid_cards`):

```javascript
db.rfid_cards.insertOne({
  uid: "04A1B2C3D4",            // Replace with your real scanned card UID
  itemKey: "apple",             // 'apple' | 'elephant' | 'cat' | 'dog' | 'banana'
  nameEn: "Apple",
  nameMm: "ပန်းသီး",
  imageKey: "apple",
  enabled: true,
  createdAt: new Date(),
  updatedAt: new Date()
});
```
