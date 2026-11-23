/**
 * ============================================================================
 *           🤖 MEMO ESP32-S3 AI DESK ROBOT — PRODUCTION FINAL FIRMWARE
 * Unified Audio Engine | Local ESP-SR WakeNet9 | Reliable Session Handshake | Watermark Flow Control
 * ============================================================================
 * Target: ESP32-S3-N16R8-V2775 (ESP32-S3 Dev Module)
 * Framework: Arduino ESP32 Core 3.3.13 / ESP-IDF 5.x
 * Memory: 16 MB QIO Flash | 8 MB Octal OPI PSRAM
 * Display: ILI9341 2.8" SPI TFT (320x240 Landscape) | Touch: XPT2046 SPI
 * Audio In: INMP441 MEMS Microphone (I2S_NUM_0, 16kHz 16-bit mono)
 * Audio Out: MAX98357A Class-D Amplifier (I2S_NUM_1, 24kHz 16-bit mono)
 * Wake Word: Espressif ESP-SR WakeNet9 ("Hi ESP" Official Test Model / Target: "Hey Memo")
 * Head Touch: TTP223 Capacitive Touch Sensor (GPIO 38, Active HIGH, Universal Fallback)
 * Volume Buttons: GPIO 41 & 42 (Active LOW with internal pull-up)
 * RFID: PN532 Module (Hardware UART1 HSU: RX GPIO 1, TX GPIO 2, 115200 baud)
 * Backend: wss://107.23.154.150.sslip.io:443/ws/esp32 (Fastify + Gemini Live)
 * ============================================================================
 */

#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "model_path.h"

#ifdef __cplusplus
extern "C" {
#endif
extern const esp_wn_iface_t esp_sr_wakenet9_quantized;
#ifdef __cplusplus
}
#endif

#include <Arduino.h>
#include <atomic>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include "myanmar_bitmaps.h"

#include <ArduinoWebsockets.h>
// ArduinoWebsockets (Gil Maimon, v0.5.4) waits only _CONNECTION_TIMEOUT = 1000 ms for the
// HTTP "101 Switching Protocols" status line. That value is hard-coded in the library and
// cannot be overridden from the sketch. (WEBSOCKETS_TCP_TIMEOUT belongs to a different
// library, Links2004 arduinoWebSockets, and has no effect here.)
// One-time patch in <Arduino>/libraries/ArduinoWebsockets/src/tiny_websockets/ws_config_defs.hpp:
//     #define _CONNECTION_TIMEOUT 1000   ->   #define _CONNECTION_TIMEOUT 4000
// (kept < 5 s: readLine() busy-waits, and the default task watchdog is 5 s)
#if defined(_CONNECTION_TIMEOUT) && (_CONNECTION_TIMEOUT < 3000)
#warning "ArduinoWebsockets _CONNECTION_TIMEOUT is 1000 ms. If cloud TLS/Upgrade handshake is slow, consider patching ws_config_defs.hpp to 4000."
#endif
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <driver/i2s.h>
#include <time.h>
#include <esp_system.h>
#include <esp_chip_info.h>
#include <esp_flash.h>
#include <esp_heap_caps.h>
#include <freertos/ringbuf.h>
#include <memory>
#include <Adafruit_PN532.h>
#include <LovyanGFX.hpp>

using namespace websockets;

// ---------------------------------------------------------------------------
// ESP32-S3 serial diagnostics:
// When USB CDC On Boot is enabled, Arduino Serial is USB CDC while Serial0 is
// UART0. The ROM boot log may be visible on a different interface from Serial.
// Mirror diagnostic output to both ports in that configuration.
// ---------------------------------------------------------------------------
class MemoDebugConsole : public Print {
public:
    void begin(unsigned long baud) {
        Serial.begin(baud);
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
        Serial0.begin(baud);
#endif
    }

    int available() {
        int count = Serial.available();
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
        count += Serial0.available();
#endif
        return count;
    }

    int read() {
        if (Serial.available()) return Serial.read();
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
        if (Serial0.available()) return Serial0.read();
#endif
        return -1;
    }

    void flush() {
        Serial.flush();
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
        Serial0.flush();
#endif
    }

    size_t write(uint8_t value) override {
        size_t written = Serial.write(value);
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
        Serial0.write(value);
#endif
        return written;
    }

    size_t write(const uint8_t* buffer, size_t size) override {
        size_t written = Serial.write(buffer, size);
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
        Serial0.write(buffer, size);
#endif
        return written;
    }
};

static MemoDebugConsole MemoConsole;
#ifdef Serial
#undef Serial
#endif
#define Serial MemoConsole

// ============================================================================
// SECTION 1: HARDWARE PIN DEFINITIONS & SYSTEM CONSTANTS
// ============================================================================
#define FIRMWARE_NAME       "Memo ESP32-S3 AI Robot"
#define FIRMWARE_VERSION    "1.0.1"
#define HARDWARE_VERSION    "ESP32-S3-N16R8-V2775"

// Display Orientation: 1 = Landscape (320x240), 3 = Inverted Landscape
#define DISPLAY_ROTATION    1

// 1. SPI Display: ILI9341 2.8" TFT (SPI2_HOST)
#define PIN_TFT_MOSI        11    // Master Out Slave In (SDI)
#define PIN_TFT_MISO        13    // Master In Slave Out (SDO)
#define PIN_TFT_SCK         12    // SPI Clock (SCL / SCK)
#define PIN_TFT_CS          10    // Display Chip Select (CS)
#define PIN_TFT_DC          9     // Data / Command (DC / RS)
#define PIN_TFT_RST         14    // Display Reset (RST)
#define PIN_TFT_BL          4     // Backlight PWM Brightness Control

// 2. Touchscreen Controller: XPT2046 (Shared SPI with ILI9341)
#define PIN_TOUCH_CS        8     // Touch Chip Select (T_CS) - Assert HIGH on boot!
#define PIN_TOUCH_IRQ       18    // Touch Pen Down Interrupt (T_IRQ)

// 3. I2S Audio Input: INMP441 MEMS Microphone (I2S_NUM_0)
#define PIN_I2S_MIC_SCK     15    // Bit Clock (BCLK)
#define PIN_I2S_MIC_WS      16    // Word Select / LRCLK
#define PIN_I2S_MIC_SD      17    // Serial Audio In (SD)

// 4. I2S Audio Output: MAX98357A Class-D Amplifier (I2S_NUM_1)
#define PIN_I2S_SPK_BCLK    5     // Bit Clock (BCLK)
#define PIN_I2S_SPK_LRC     6     // Word Select / WS
#define PIN_I2S_SPK_DIN     7     // Serial Audio Out (DIN)

// 5. Head Touch & Physical Push Buttons
#define PIN_TOUCH_SENSOR    38    // TTP223 Capacitive Head Touch (Active HIGH)
#define PIN_BTN_VOL_UP      41    // Push Button: Volume Up (Active LOW)
#define PIN_BTN_VOL_DOWN    42    // Push Button: Volume Down (Active LOW)

// 6. PN532 RFID Module (High Speed UART Mode)
#define PIN_PN532_RX        1     // ESP32 RX <- PN532 TXD
#define PIN_PN532_TX        2     // ESP32 TX -> PN532 RXD
#define PN532_UART_NUM      1     // HardwareSerial Port 1

// 7. Onboard RGB LED
#define PIN_RGB_LED         48    // WS2812 Onboard LED (driven LOW)

// System & Audio Parameters
#define AUDIO_IN_SAMPLE_RATE     16000
#define AUDIO_OUT_SAMPLE_RATE    24000
#define AUDIO_DMA_BUF_COUNT      8
#define AUDIO_DMA_BUF_LEN        512
#define AUDIO_CHUNK_SIZE         1024
#define AUDIO_RING_BUFFER_BYTES_PSRAM  (384 * 1024) // 384 KiB in Octal PSRAM (~8.2 sec of 24kHz mono PCM)
#define AUDIO_RING_BUFFER_BYTES_SRAM   32768        // 32 KB fallback in internal SRAM
#define DEFAULT_VOLUME           80
#define DEFAULT_BRIGHTNESS       80
#define MAX_VOLUME_LIMIT         100
#define VOLUME_STEP              5

// Cloud Backend Configuration
#define DEFAULT_WS_HOST          "107.23.154.150.sslip.io"
#define DEFAULT_WS_PORT          443
#define DEFAULT_WS_PATH          "/ws/esp32"
#define WS_HEARTBEAT_INTERVAL_MS 15000

// Wi-Fi SoftAP Provisioning Portal Configuration
#define PROV_AP_SSID             "ESP32-Robot-Setup"
#define PROV_AP_PASSWORD         "" // Optional password (leave empty for open captive portal)

// ============================================================================
// ESP-SR WakeNet9 Local Wake-Word Engine Configuration
// ============================================================================
// Built-in official test model in flash partition: "wn9_hiesp" ("Hi ESP")
// Final production target: "Hey Memo" (requires custom-trained model artifact "wn9_heymemo")
#define ENABLE_ESP_SR_WAKENET      1
#define WAKE_WORD_MODEL_NAME       "wn9_hiesp"
#define WAKE_WORD_PHRASE           "Hi ESP (Temp Test - Target: Hey Memo)"
#define WAKE_DETECTION_COOLDOWN_MS 3000

// ============================================================================
// SECTION 2: VERIFIED PRODUCTION TLS ROOT CA CERTIFICATES
// ============================================================================
// Complete, verified Let's Encrypt (RSA & ECDSA) and ZeroSSL Root CA Bundle
static const char PROD_ROOT_CA_BUNDLE[] PROGMEM =
// ISRG Root X1 (Let's Encrypt RSA Root)
"-----BEGIN CERTIFICATE-----\n"
"MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw\n"
"TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh\n"
"cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4\n"
"WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu\n"
"ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY\n"
"MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc\n"
"h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+\n"
"0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U\n"
"A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW\n"
"T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH\n"
"B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC\n"
"B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv\n"
"KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn\n"
"OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn\n"
"jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw\n"
"qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI\n"
"rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV\n"
"HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq\n"
"hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL\n"
"ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ\n"
"3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK\n"
"NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5\n"
"ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur\n"
"TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC\n"
"jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc\n"
"oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq\n"
"4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA\n"
"mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d\n"
"emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=\n"
"-----END CERTIFICATE-----\n"
// ISRG Root X2 (Let's Encrypt ECDSA Root - Self-Signed)
"-----BEGIN CERTIFICATE-----\n"
"MIICGzCCAaGgAwIBAgIQQdKd0XLq7qeAwSxs6S+HUjAKBggqhkjOPQQDAzBPMQsw\n"
"CQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJuZXQgU2VjdXJpdHkgUmVzZWFyY2gg\n"
"R3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBYMjAeFw0yMDA5MDQwMDAwMDBaFw00\n"
"MDA5MTcxNjAwMDBaME8xCzAJBgNVBAYTAlVTMSkwJwYDVQQKEyBJbnRlcm5ldCBT\n"
"ZWN1cml0eSBSZXNlYXJjaCBHcm91cDEVMBMGA1UEAxMMSVNSRyBSb290IFgyMHYw\n"
"EAYHKoZIzj0CAQYFK4EEACIDYgAEzZvVn4CDCuwJSvMWSj5cz3es3mcFDR0HttwW\n"
"+1qLFNvicWDEukWVEYmO6gbf9yoWHKS5xcUy4APgHoIYOIvXRdgKam7mAHf7AlF9\n"
"ItgKbppbd9/w+kHsOdx1ymgHDB/qo0IwQDAOBgNVHQ8BAf8EBAMCAQYwDwYDVR0T\n"
"AQH/BAUwAwEB/zAdBgNVHQ4EFgQUfEKWrt5LSDv6kviejM9ti6lyN5UwCgYIKoZI\n"
"zj0EAwMDaAAwZQIwe3lORlCEwkSHRhtFcP9Ymd70/aTSVaYgLXTWNLxBo1BfASdW\n"
"tL4ndQavEi51mI38AjEAi/V3bNTIZargCyzuFJ0nN6T5U6VR5CmD1/iQMVtCnwr1\n"
"/q4AaOeMSQ+2b1tbFfLn\n"
"-----END CERTIFICATE-----\n"
// ISRG Root X2 (Let's Encrypt ECDSA Root - Cross-Signed by ISRG Root X1)
"-----BEGIN CERTIFICATE-----\n"
"MIIEcDCCAligAwIBAgIQbI8dxyfHEX97r4U6yYD5zTANBgkqhkiG9w0BAQsFADBP\n"
"MQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJuZXQgU2VjdXJpdHkgUmVzZWFy\n"
"Y2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBYMTAeFw0yNjA1MTMwMDAwMDBa\n"
"Fw0zMjA5MDIyMzU5NTlaME8xCzAJBgNVBAYTAlVTMSkwJwYDVQQKEyBJbnRlcm5l\n"
"dCBTZWN1cml0eSBSZXNlYXJjaCBHcm91cDEVMBMGA1UEAxMMSVNSRyBSb290IFgy\n"
"MHYwEAYHKoZIzj0CAQYFK4EEACIDYgAEzZvVn4CDCuwJSvMWSj5cz3es3mcFDR0H\n"
"ttwW+1qLFNvicWDEukWVEYmO6gbf9yoWHKS5xcUy4APgHoIYOIvXRdgKam7mAHf7\n"
"AlF9ItgKbppbd9/w+kHsOdx1ymgHDB/qo4H1MIHyMA4GA1UdDwEB/wQEAwIBBjAd\n"
"BgNVHSUEFjAUBggrBgEFBQcDAQYIKwYBBQUHAwIwDwYDVR0TAQH/BAUwAwEB/zAd\n"
"BgNVHQ4EFgQUfEKWrt5LSDv6kviejM9ti6lyN5UwHwYDVR0jBBgwFoAUebRZ5nu2\n"
"5eQBc4AIiMgaWPbpm24wMgYIKwYBBQUHAQEEJjAkMCIGCCsGAQUFBzAChhZodHRw\n"
"Oi8veDEuaS5sZW5jci5vcmcvMBMGA1UdIAQMMAowCAYGZ4EMAQIBMCcGA1UdHwQg\n"
"MB4wHKAaoBiGFmh0dHA6Ly94MS5jLmxlbmNyLm9yZy8wDQYJKoZIhvcNAQELBQAD\n"
"ggIBAD2/e9frmMxNpCV03qUHegg+MV2wz9644YoXdqtH8RyWYcBO7xfjjGEXdU1e\n"
"/o0OkEFiynUCOSIk/vLLo7ttz6CPAeNlWfC0XNkoGeWgK6jjXvozBaGuGH5n0Ufo\n"
"shMeWTuURqNN5G00sSXDTBrpp2+mgvdZQjb8K11TYMA25QA+YHNfbIEL0BniAhKS\n"
"2gsnJjSzrdZLI+EZ7SEyqdR2rkjd1KutLDU+n3TFyxjniZVGur4YlhMP3mY/dV95\n"
"IruAkkjOZier6hGBdEgZXXvaCz9u9iVEadsIE75pAGL8oHV5vxdARDiotRpul1IN\n"
"/UZwzAbrfUFcw1HkAcYD/mlZfnQ2ieCF2MS7j3Vhv7JPDKp45fmykmzYNSrumRW0\n"
"upFFKDBOoF7hsOb7oLyHS+Uft6jOUfOrogj8YUx38hKb2K20r42OgsSdDdxdeYWc\n"
"MS3Sb6mwJeSZEYxJ2gaXnDSPaKhhrNkYwljyVQyr4Nq+MEJytXNTnHqaAcrNwZlV\n"
"pcJL1KBnMrMjP7eanvUwL3FYj3cF17jtboLt7gLoi4+2rWZFvn+w54jmd/FIuhhZ\n"
"cEaU/wvU6BUNMtcVquVGHp7itQeDth5j+XL3j4WJ2SABwzUl6OeYdgpIt/ITZa+p\n"
"TT0mQ/r5XyA4MEAiabn7XJjvCERlF2dcn2wqJw+CreTkkQ2R\n"
"-----END CERTIFICATE-----\n"
// Root YE (Let's Encrypt ECDSA Intermediate Root)
"-----BEGIN CERTIFICATE-----\n"
"MIICpjCCAiugAwIBAgIRAIchZfw0tuX7qK3Vs3BftTowCgYIKoZIzj0EAwMwTzEL\n"
"MAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2VhcmNo\n"
"IEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDIwHhcNMjYwNTEzMDAwMDAwWhcN\n"
"MzIwOTAyMjM1OTU5WjAuMQswCQYDVQQGEwJVUzENMAsGA1UEChMESVNSRzEQMA4G\n"
"A1UEAxMHUm9vdCBZRTB2MBAGByqGSM49AgEGBSuBBAAiA2IABDwS/6vhrcVqcbBo\n"
"+wgdI3fwn9x7DNJJOY/lTOti0vkwuRN87RhEhTH17E7XyFjWsPYhIPt/wzOqxTd2\n"
"b+4ZJNy9ID04YywF9U5zasDVyGSNErVNtz8uSGh5izW87j77GaOB6zCB6DAOBgNV\n"
"HQ8BAf8EBAMCAQYwEwYDVR0lBAwwCgYIKwYBBQUHAwEwDwYDVR0TAQH/BAUwAwEB\n"
"/zAdBgNVHQ4EFgQUo8gmWo6hTNA1Y/ybI8g6rlbzT1YwHwYDVR0jBBgwFoAUfEKW\n"
"rt5LSDv6kviejM9ti6lyN5UwMgYIKwYBBQUHAQEEJjAkMCIGCCsGAQUFBzAChhZo\n"
"dHRwOi8veDIuaS5sZW5jci5vcmcvMBMGA1UdIAQMMAowCAYGZ4EMAQIBMCcGA1Ud\n"
"HwQgMB4wHKAaoBiGFmh0dHA6Ly94Mi5jLmxlbmNyLm9yZy8wCgYIKoZIzj0EAwMD\n"
"aQAwZgIxAMU19WCtmxVND8UHBZRoma49Z7jPs64Dma0eTu1OChVbB/2J7GV3nvYK\n"
"Ax54uk1G9QIxAO0miLVJu8PLNiXXXkiE/gsK3CTRTF/aeo4bMX42Zw40csRU6AC2\n"
"6hSW1/IWaas6dg==\n"
"-----END CERTIFICATE-----\n"
// YE2 (Let's Encrypt ECDSA Intermediate CA)
"-----BEGIN CERTIFICATE-----\n"
"MIICjDCCAhGgAwIBAgIQTfOxXdbAeExQfNN7WObxFTAKBggqhkjOPQQDAzAuMQsw\n"
"CQYDVQQGEwJVUzENMAsGA1UEChMESVNSRzEQMA4GA1UEAxMHUm9vdCBZRTAeFw0y\n"
"NTA5MDMwMDAwMDBaFw0yODA5MDIyMzU5NTlaMDMxCzAJBgNVBAYTAlVTMRYwFAYD\n"
"VQQKEw1MZXQncyBFbmNyeXB0MQwwCgYDVQQDEwNZRTIwdjAQBgcqhkjOPQIBBgUr\n"
"gQQAIgNiAARxmrQzkdbEEL3MqXt3dJQttYc47axkdDTHud5TPqM2z5uSD5cmk0Wr\n"
"HlWXvnlvqBLqiB34kluxIbmMyAiq3/YD6e80/vV259K8XQIdjFXloYOa0mIU71f7\n"
"HQ09PvYDlw+jge4wgeswDgYDVR0PAQH/BAQDAgGGMBMGA1UdJQQMMAoGCCsGAQUF\n"
"BwMBMBIGA1UdEwEB/wQIMAYBAf8CAQAwHQYDVR0OBBYEFLlZ8o7PIvCG0zdI/3YU\n"
"GLqC2FWHMB8GA1UdIwQYMBaAFKPIJlqOoUzQNWP8myPIOq5W809WMDIGCCsGAQUF\n"
"BwEBBCYwJDAiBggrBgEFBQcwAoYWaHR0cDovL3llLmkubGVuY3Iub3JnLzATBgNV\n"
"HSAEDDAKMAgGBmeBDAECATAnBgNVHR8EIDAeMBygGqAYhhZodHRwOi8veWUuYy5s\n"
"ZW5jci5vcmcvMAoGCCqGSM49BAMDA2kAMGYCMQDIcnw5dcZLN9ffynXnnkLD/itS\n"
"JEycJPb3sRkzeqBowup7vOsAwaqoCnNn/jh9wycCMQCJM6CPlaOC4pQYYbJtVPYb\n"
"DKrIb2EKk5NpOpE6/XttQYZV/3gilB9l+Cc/DOVwmyg=\n"
"-----END CERTIFICATE-----\n"
// USERTrust RSA Certification Authority (ZeroSSL)
"-----BEGIN CERTIFICATE-----\n"
"MIIF3jCCA8agAwIBAgIQAf1tMPyjylGoG7xkDjUDLTANBgkqhkiG9w0BAQwFADCB\n"
"iDELMAkGA1UEBhMCVVMxEzARBgNVBAgTCk5ldyBKZXJzZXkxFDASBgNVBAcTC0pl\n"
"cnNleSBDaXR5MR4wHAYDVQQKExVUaGUgVVNFUlRSVVNUIE5ldHdvcmsxLjAsBgNV\n"
"BAMTJVVTRVJUcnVzdCBSU0EgQ2VydGlmaWNhdGlvbiBBdXRob3JpdHkwHhcNMTAw\n"
"MjAxMDAwMDAwWhcNMzgwMTE4MjM1OTU5WjCBiDELMAkGA1UEBhMCVVMxEzARBgNV\n"
"BAgTCk5ldyBKZXJzZXkxFDASBgNVBAcTC0plcnNleSBDaXR5MR4wHAYDVQQKExVU\n"
"aGUgVVNFUlRSVVNUIE5ldHdvcmsxLjAsBgNVBAMTJVVTRVJUcnVzdCBSU0EgQ2Vy\n"
"dGlmaWNhdGlvbiBBdXRob3JpdHkwggIiMA0GCSqGSIb3DQEBAQUAA4ICDwAwggIK\n"
"AoICAQCAEmUXNg7D2wiz0KxXDXbtzSfTTK1Qg2HiqiBNCS1kCdzOiZ/MPans9s/B\n"
"3PHTsdZ7NygRK0faOca8Ohm0X6a9fZ2jY0K2dvKpOyuR+OJv0OwWIJAJPuLodMkY\n"
"tJHUYmTbf6MG8YgYapAiPLz+E/CHFHv25B+O1ORRxhFnRghRy4YUVD+8M/5+bJz/\n"
"Fp0YvVGONaanZshyZ9shZrHUm3gDwFA66Mzw3LyeTP6vBZY1H1dat//O+T23LLb2\n"
"VN3I5xI6Ta5MirdcmrS3ID3KfyI0rn47aGYBROcBTkZTmzNg95S+UzeQc0PzMsNT\n"
"79uq/nROacdrjGCT3sTHDN/hMq7MkztReJVni+49Vv4M0GkPGw/zJSZrM233bkf6\n"
"c0Plfg6lZrEpfDKEY1WJxA3Bk1QwGROs0303p+tdOmw1XNtB1xLaqUkL39iAigmT\n"
"Yo61Zs8liM2EuLE/pDkP2QKe6xJMlXzzawWpXhaDzLhn4ugTncxbgtNMs+1b/97l\n"
"c6wjOy0AvzVVdAlJ2ElYGn+SNuZRkg7zJn0cTRe8yexDJtC/QV9AqURE9JnnV4ee\n"
"UB9XVKg+/XRjL7FQZQnmWEIuQxpMtPAlR1n6BB6T1CZGSlCBst6+eLf8ZxXhyVeE\n"
"Hg9j1uliutZfVS7qXMYoCAQlObgOK6nyTJccBz8NUvXt7y+CDwIDAQABo0IwQDAd\n"
"BgNVHQ4EFgQUU3m/WqorSs9UgOHYm8Cd8rIDZsswDgYDVR0PAQH/BAQDAgEGMA8G\n"
"A1UdEwEB/wQFMAMBAf8wDQYJKoZIhvcNAQEMBQADggIBAFzUfA3P9wF9QZllDHPF\n"
"Up/L+M+ZBn8b2kMVn54CVVeWFPFSPCeHlCjtHzoBN6J2/FNQwISbxmtOuowhT6KO\n"
"VWKR82kV2LyI48SqC/3vqOlLVSoGIG1VeCkZ7l8wXEskEVX/JJpuXior7gtNn3/3\n"
"ATiUFJVDBwn7YKnuHKsSjKCaXqeYalltiz8I+8jRRa8YFWSQEg9zKC7F4iRO/Fjs\n"
"8PRF/iKz6y+O0tlFYQXBl2+odnKPi4w2r78NBc5xjeambx9spnFixdjQg3IM8WcR\n"
"iQycE0xyNN+81XHfqnHd4blsjDwSXWXavVcStkNr/+XeTWYRUc+ZruwXtuhxkYze\n"
"Sf7dNXGiFSeUHM9h4ya7b6NnJSFd5t0dCy5oGzuCr+yDZ4XUmFF0sbmZgIn/f3gZ\n"
"XHlKYC6SQK5MNyosycdiyA5d9zZbyuAlJQG03RoHnHcAP9Dc1ew91Pq7P8yF1m9/\n"
"qS3fuQL39ZeatTXaw2ewh0qpKJ4jjv9cJ2vhsE/zB+4ALtRZh8tSQZXq9EfX7mRB\n"
"VXyNWQKV3WKdwrnuWih0hKWbt5DHDAff9Yk2dDLWKMGwsAvgnEzDHNb842m1R0aB\n"
"L6KCq9NjRHDEjf8tM7qtj3u1cIiuPhnPQCjY/MiQu12ZIvVS5ljFH4gxQ+6IHdfG\n"
"jjxDah2nGN59PRbxYvnKkKj9\n"
"-----END CERTIFICATE-----\n";

// ============================================================================
// SECTION 3: SYSTEM ENUMS & EVENT BUS
// ============================================================================
enum class RobotState {
    BOOT,
    IDLE,
    LISTENING,
    THINKING,
    SPEAKING,
    ALERT,
    ERROR
};

const char* robotStateToString(RobotState st) {
    switch (st) {
        case RobotState::BOOT:      return "BOOT";
        case RobotState::IDLE:      return "IDLE";
        case RobotState::LISTENING: return "LISTENING";
        case RobotState::THINKING:  return "THINKING";
        case RobotState::SPEAKING:  return "SPEAKING";
        case RobotState::ALERT:     return "ALERT";
        case RobotState::ERROR:     return "ERROR";
        default:                    return "UNKNOWN";
    }
}

enum class RobotEmotion {
    NEUTRAL,
    HAPPY,
    ANGRY,
    SAD,
    EXCITED,
    ANNOYED,
    QUESTIONING,
    IDLE1,
    IDLE2,
    IDLE3,
    WINK
};

enum class ActiveScreen {
    SCREEN_FACE,
    SCREEN_MENU,
    SCREEN_EMOTIONS,
    SCREEN_VOICE,
    SCREEN_VOLUME,
    SCREEN_WIFI,
    SCREEN_INFO,
    SCREEN_DIAGNOSTICS,
    SCREEN_RFID
};

enum class WakePrompt : uint8_t { None, Speak, Offline };
static std::atomic<WakePrompt> wakePrompt{WakePrompt::None};

enum class EventType {
    NONE,
    TOUCH_SENSOR_PRESS,
    TOUCH_SENSOR_RELEASE,
    BUTTON_VOL_UP,
    BUTTON_VOL_DOWN,
    VOICE_SESSION_START,
    VOICE_SESSION_END,
    VOICE_RESPONSE_START,
    VOICE_RESPONSE_END,
    VOICE_AUDIO_CANCEL,
    WS_AUTHENTICATED,
    WS_DISCONNECTED,
    RFID_CARD_DETECTED,
    CMD_SET_VOLUME,
    CMD_SET_BRIGHTNESS,
    CMD_STOP_AUDIO,
    PROV_PORTAL_START,
    PROV_PORTAL_STOP
};

struct AppEvent {
    EventType type;
    int intValue;
    char text[64];
};

QueueHandle_t appEventQueue = nullptr;

struct RfidDisplayResult {
    char uid[32];
    char itemKey[24];
    char nameEn[96];
    bool found;
    unsigned long receivedAt;
};
QueueHandle_t rfidDisplayQueue = nullptr;
QueueHandle_t rfidLookupQueue = nullptr;

void postEvent(EventType type, int intValue = 0, const char* str = nullptr) {
    if (!appEventQueue) return;
    AppEvent ev;
    ev.type = type;
    ev.intValue = intValue;
    if (str) {
        strncpy(ev.text, str, sizeof(ev.text) - 1);
        ev.text[sizeof(ev.text) - 1] = '\0';
    } else {
        ev.text[0] = '\0';
    }
    xQueueSend(appEventQueue, &ev, 0);
}

// ============================================================================
// SECTION 4: NVS CONFIGURATION STORAGE & CALIBRATION PERSISTENCE
// ============================================================================
struct TouchCalibrationData {
    uint16_t data[8];
    bool valid;
};

class ConfigManager {
private:
    Preferences prefsNet;
    Preferences prefsAuth;
    Preferences prefsCfg;

    String cachedSsid = "";
    String cachedPass = "";
    String cachedDeviceId = "6afc90a09b5d5190078ba835";
    String cachedApiKey = "rbt_TO6t2Ags3brXCBpzjGrWWMG6Bz3AVAto";
    uint8_t cachedVolume = DEFAULT_VOLUME;
    uint8_t cachedBrightness = DEFAULT_BRIGHTNESS;
    TouchCalibrationData cachedTouchCal = { { 240, 260, 240, 3750, 3750, 260, 3750, 3750 }, false };

public:
    static ConfigManager& getInstance() {
        static ConfigManager instance;
        return instance;
    }

    void begin() {
        prefsNet.begin("robot_net", false);
        if (prefsNet.isKey("ssid")) cachedSsid = prefsNet.getString("ssid", "KoMyo");
        if (prefsNet.isKey("pass")) cachedPass = prefsNet.getString("pass", "0995138020");
        prefsNet.end();

        prefsAuth.begin("robot_auth", false);
        cachedDeviceId = prefsAuth.getString("dev_id", "6afc90a09b5d5190078ba835");
        cachedApiKey = prefsAuth.getString("api_key", "rbt_TO6t2Ags3brXCBpzjGrWWMG6Bz3AVAto");
        prefsAuth.end();

        cachedDeviceId.trim();
        cachedApiKey.trim();

        prefsCfg.begin("robot_cfg", false);
        cachedVolume = prefsCfg.getUChar("volume", DEFAULT_VOLUME);
        cachedBrightness = prefsCfg.getUChar("bright", DEFAULT_BRIGHTNESS);
        if (prefsCfg.isKey("touch_cal")) {
            prefsCfg.getBytes("touch_cal", &cachedTouchCal, sizeof(cachedTouchCal));
        }
        prefsCfg.end();

        Serial.println("[ConfigManager] Storage Initialized.");
        Serial.printf("[ConfigManager] Wi-Fi SSID: %s\n", cachedSsid.length() ? cachedSsid.c_str() : "<NOT SET>");
        Serial.printf("[ConfigManager] Device ID: %s | API key: %s | Volume: %u%% | Brightness: %u%%\n",
                      cachedDeviceId.c_str(), cachedApiKey.length() ? "SET" : "MISSING", cachedVolume, cachedBrightness);
    }

    String getWifiSSID() const { return cachedSsid; }
    String getWifiPassword() const { return cachedPass; }
    void setWifiCredentials(const String& ssid, const String& pass) {
        cachedSsid = ssid;
        cachedPass = pass;
        prefsNet.begin("robot_net", false);
        prefsNet.putString("ssid", ssid);
        prefsNet.putString("pass", pass);
        prefsNet.end();
    }

    String getDeviceId() const { return cachedDeviceId; }
    String getDeviceApiKey() const { return cachedApiKey; }
    void setDeviceCredentials(const String& id, const String& key) {
        cachedDeviceId = id;
        cachedApiKey = key;
        cachedDeviceId.trim();
        cachedApiKey.trim();
        prefsAuth.begin("robot_auth", false);
        prefsAuth.putString("dev_id", cachedDeviceId);
        prefsAuth.putString("api_key", cachedApiKey);
        prefsAuth.end();
    }

    uint8_t getVolume() const { return cachedVolume; }
    void setVolume(uint8_t vol) {
        if (vol > MAX_VOLUME_LIMIT) vol = MAX_VOLUME_LIMIT;
        cachedVolume = vol;
        prefsCfg.begin("robot_cfg", false);
        prefsCfg.putUChar("volume", vol);
        prefsCfg.end();
    }

    uint8_t getBrightness() const { return cachedBrightness; }
    void setBrightness(uint8_t br) {
        if (br > 100) br = 100;
        cachedBrightness = br;
        prefsCfg.begin("robot_cfg", false);
        prefsCfg.putUChar("bright", br);
        prefsCfg.end();
    }

    TouchCalibrationData getTouchCal() const { return cachedTouchCal; }
    void saveTouchCal(const TouchCalibrationData& cal) {
        cachedTouchCal = cal;
        prefsCfg.begin("robot_cfg", false);
        prefsCfg.putBytes("touch_cal", &cachedTouchCal, sizeof(cachedTouchCal));
        prefsCfg.end();
        Serial.println("[ConfigManager] Touch calibration parameters saved to NVS.");
    }

    void clearAll() {
        prefsNet.begin("robot_net", false); prefsNet.clear(); prefsNet.end();
        prefsAuth.begin("robot_auth", false); prefsAuth.clear(); prefsAuth.end();
        prefsCfg.begin("robot_cfg", false); prefsCfg.clear(); prefsCfg.end();
    }
};

// ============================================================================
// SECTION 5: HARDWARE DISPLAY & XPT2046 TOUCH DRIVER (LovyanGFX Engine)
// ============================================================================
class LGFX_Memo : public lgfx::LGFX_Device {
    lgfx::Panel_ILI9341 _panel_instance;
    lgfx::Bus_SPI       _bus_instance;
    lgfx::Light_PWM     _light_instance;
    lgfx::Touch_XPT2046 _touch_instance;

public:
    LGFX_Memo() {
        // 1. SPI Bus (10 MHz write clock for rock-solid stability)
        {
            auto cfg = _bus_instance.config();
            cfg.spi_host   = SPI2_HOST;
            cfg.spi_mode   = 0;
            cfg.freq_write = 10000000;
            cfg.freq_read  = 5000000;
            cfg.spi_3wire  = false;
            cfg.use_lock   = true;
            cfg.dma_channel = 0;
            cfg.pin_sclk   = PIN_TFT_SCK;  // 12
            cfg.pin_mosi   = PIN_TFT_MOSI; // 11
            cfg.pin_miso   = PIN_TFT_MISO; // 13
            cfg.pin_dc     = PIN_TFT_DC;   // 9
            _bus_instance.config(cfg);
            _panel_instance.setBus(&_bus_instance);
        }

        // 2. ILI9341 LCD Panel (240x320 physical)
        {
            auto cfg = _panel_instance.config();
            cfg.pin_cs           = PIN_TFT_CS;  // 10
            cfg.pin_rst          = PIN_TFT_RST; // 14
            cfg.pin_busy         = -1;
            cfg.memory_width     = 240;
            cfg.memory_height    = 320;
            cfg.panel_width      = 240;
            cfg.panel_height     = 320;
            cfg.offset_x         = 0;
            cfg.offset_y         = 0;
            cfg.offset_rotation  = 0;
            cfg.dummy_read_pixel = 8;
            cfg.dummy_read_bits  = 1;
            cfg.readable         = true;
            cfg.invert           = false;
            cfg.rgb_order        = false;
            cfg.dlen_16bit       = false;
            cfg.bus_shared       = true;
            _panel_instance.config(cfg);
        }

        // 3. Backlight PWM (GPIO 4)
        {
            auto cfg = _light_instance.config();
            cfg.pin_bl      = PIN_TFT_BL; // 4
            cfg.invert      = false;
            cfg.freq        = 44100;
            cfg.pwm_channel = 0;
            _light_instance.config(cfg);
            _panel_instance.setLight(&_light_instance);
        }

        // 4. XPT2046 Resistive Touchscreen Controller
        {
            auto cfg = _touch_instance.config();
            cfg.x_min      = 200;
            cfg.x_max      = 3800;
            cfg.y_min      = 200;
            cfg.y_max      = 3800;
            cfg.pin_int    = PIN_TOUCH_IRQ; // 18
            cfg.bus_shared = true;
            cfg.spi_host   = SPI2_HOST;
            cfg.freq       = 1000000;       // 1 MHz for touch SPI noise immunity
            cfg.pin_sclk   = PIN_TFT_SCK;   // 12
            cfg.pin_mosi   = PIN_TFT_MOSI;  // 11
            cfg.pin_miso   = PIN_TFT_MISO;  // 13 (T_OUT / T_DO)
            cfg.pin_cs     = PIN_TOUCH_CS;  // 8 (T_CS)
            cfg.offset_rotation = DISPLAY_ROTATION;
            _touch_instance.config(cfg);
            _panel_instance.setTouch(&_touch_instance);
        }

        setPanel(&_panel_instance);
    }
};

LGFX_Memo tft;

// ============================================================================
// SECTION 6: GROBOT-STYLE SPRING ANIMATION EYE ENGINE
// ============================================================================
// Custom spring-kinematics eye engine inspired by Grobot Animations,
// natively implemented for LovyanGFX dirty-rectangle rendering.
class GrobotStyleEyeEngine {
private:
    float eyeWidth = 76.0f;
    float eyeHeight = 50.0f;
    float targetWidth = 76.0f;
    float targetHeight = 50.0f;

    float currentX = 0.0f;
    float currentY = 0.0f;
    float targetX = 0.0f;
    float targetY = 0.0f;
    float velX = 0.0f;
    float velY = 0.0f;
    float velW = 0.0f;
    float velH = 0.0f;

    const float springK = 0.22f;
    const float damping = 0.65f;

    RobotEmotion currentEmotion = RobotEmotion::NEUTRAL;
    unsigned long lastBlinkTime = 0;
    unsigned long blinkDuration = 140;
    bool isBlinking = false;
    unsigned long lastGazeShiftTime = 0;

    int lastLeftX = 0, lastLeftY = 0, lastLeftW = 0, lastLeftH = 0;
    int lastRightX = 0, lastRightY = 0, lastRightW = 0, lastRightH = 0;

public:
    void begin() {
        eyeWidth = targetWidth = 76.0f;
        eyeHeight = targetHeight = 50.0f;
        currentX = targetX = 0.0f;
        currentY = targetY = 0.0f;
        lastBlinkTime = millis();
        lastGazeShiftTime = millis();
    }

    void setEmotion(RobotEmotion emotion) {
        currentEmotion = emotion;
        switch (emotion) {
            case RobotEmotion::HAPPY:
                targetWidth = 76.0f; targetHeight = 36.0f;
                break;
            case RobotEmotion::EXCITED:
                targetWidth = 84.0f; targetHeight = 58.0f;
                break;
            case RobotEmotion::ANGRY:
                targetWidth = 74.0f; targetHeight = 38.0f;
                break;
            case RobotEmotion::SAD:
                targetWidth = 70.0f; targetHeight = 34.0f;
                break;
            case RobotEmotion::ANNOYED:
                targetWidth = 76.0f; targetHeight = 24.0f;
                break;
            case RobotEmotion::QUESTIONING:
                targetWidth = 78.0f; targetHeight = 46.0f;
                break;
            case RobotEmotion::WINK:
                targetWidth = 76.0f; targetHeight = 50.0f;
                break;
            default: // NEUTRAL, IDLE1..3
                targetWidth = 76.0f; targetHeight = 50.0f;
                break;
        }
    }

    void lookAt(float x, float y) {
        targetX = constrain(x, -25.0f, 25.0f);
        targetY = constrain(y, -15.0f, 15.0f);
    }

    void updatePhysics() {
        unsigned long now = millis();

        // 1. Natural eye blinking
        if (!isBlinking && (now - lastBlinkTime > 3600)) {
            isBlinking = true;
            lastBlinkTime = now;
        } else if (isBlinking && (now - lastBlinkTime > blinkDuration)) {
            isBlinking = false;
            lastBlinkTime = now + (rand() % 2500);
        }

        // 2. Autonomous gaze shifts
        if (now - lastGazeShiftTime > 2800) {
            lastGazeShiftTime = now;
            int r = rand() % 5;
            if (r == 0) lookAt(0, 0);
            else if (r == 1) lookAt(-18, 0);
            else if (r == 2) lookAt(18, 0);
            else if (r == 3) lookAt(0, -10);
            else lookAt(0, 8);
        }

        // 3. Spring kinematics
        float diffX = targetX - currentX;
        velX = (velX + diffX * springK) * damping;
        currentX += velX;

        float diffY = targetY - currentY;
        velY = (velY + diffY * springK) * damping;
        currentY += velY;

        float effTargetH = isBlinking ? 6.0f : targetHeight;
        float diffH = effTargetH - eyeHeight;
        velH = (velH + diffH * springK) * damping;
        eyeHeight += velH;

        float diffW = targetWidth - eyeWidth;
        velW = (velW + diffW * springK) * damping;
        eyeWidth += velW;
    }

    void render(bool forceFullRedraw = false) {
        updatePhysics();

        const uint16_t COLOR_BG = tft.color565(14, 40, 78);
        const uint16_t COLOR_EYE = TFT_WHITE;
        const uint16_t COLOR_GLOW = tft.color565(80, 190, 255);

        int centerX = 160 + (int)currentX;
        int centerY = 120 + (int)currentY;
        int spacing = 58;

        int leftX = centerX - spacing - (int)(eyeWidth / 2);
        int rightX = centerX + spacing - (int)(eyeWidth / 2);
        int eyeY = centerY - (int)(eyeHeight / 2);
        int curW = (int)eyeWidth;
        int curH = (int)eyeHeight;
        int cornerR = curH / 2;
        if (cornerR > 22) cornerR = 22;

        if (forceFullRedraw) {
            tft.fillRect(0, 24, 320, 240 - 24, COLOR_BG);
        } else {
            if (lastLeftW > 0) tft.fillRect(lastLeftX - 4, lastLeftY - 4, lastLeftW + 8, lastLeftH + 8, COLOR_BG);
            if (lastRightW > 0) tft.fillRect(lastRightX - 4, lastRightY - 4, lastRightW + 8, lastRightH + 8, COLOR_BG);
        }

        // Left Eye
        if (currentEmotion == RobotEmotion::WINK) {
            tft.fillRoundRect(leftX, centerY - 3, curW, 6, 3, COLOR_EYE);
        } else if (currentEmotion == RobotEmotion::ANGRY) {
            tft.fillRoundRect(leftX, eyeY, curW, curH, cornerR, COLOR_EYE);
            tft.fillTriangle(leftX - 2, eyeY - 2, leftX + curW + 2, eyeY - 2, leftX + curW + 2, eyeY + 14, COLOR_BG);
        } else if (currentEmotion == RobotEmotion::HAPPY) {
            tft.fillRoundRect(leftX, eyeY, curW, curH, cornerR, COLOR_EYE);
            tft.fillRoundRect(leftX, eyeY + (curH / 2), curW, curH / 2, cornerR, COLOR_BG);
        } else {
            tft.drawRoundRect(leftX - 2, eyeY - 2, curW + 4, curH + 4, cornerR + 2, COLOR_GLOW);
            tft.fillRoundRect(leftX, eyeY, curW, curH, cornerR, COLOR_EYE);
            if (!isBlinking && curH > 18) {
                tft.fillCircle(leftX + 16, eyeY + 12, 4, tft.color565(225, 245, 255));
            }
        }

        // Right Eye
        if (currentEmotion == RobotEmotion::ANGRY) {
            tft.fillRoundRect(rightX, eyeY, curW, curH, cornerR, COLOR_EYE);
            tft.fillTriangle(rightX - 2, eyeY - 2, rightX + curW + 2, eyeY - 2, rightX - 2, eyeY + 14, COLOR_BG);
        } else if (currentEmotion == RobotEmotion::HAPPY) {
            tft.fillRoundRect(rightX, eyeY, curW, curH, cornerR, COLOR_EYE);
            tft.fillRoundRect(rightX, eyeY + (curH / 2), curW, curH / 2, cornerR, COLOR_BG);
        } else {
            tft.drawRoundRect(rightX - 2, eyeY - 2, curW + 4, curH + 4, cornerR + 2, COLOR_GLOW);
            tft.fillRoundRect(rightX, eyeY, curW, curH, cornerR, COLOR_EYE);
            if (!isBlinking && curH > 18) {
                tft.fillCircle(rightX + 16, eyeY + 12, 4, tft.color565(225, 245, 255));
            }
        }

        lastLeftX = leftX; lastLeftY = eyeY; lastLeftW = curW; lastLeftH = curH;
        lastRightX = rightX; lastRightY = eyeY; lastRightW = curW; lastRightH = curH;
    }
};

GrobotStyleEyeEngine eyes;

// ============================================================================
// SECTION 7: CENTRALIZED UNIFIED I2S AUDIO MANAGER (MAX98357A + Octal PSRAM)
// ============================================================================
// ARCHITECTURE RULES (FINAL DEADLINE CERTIFIED):
// 1. Only ONE task (taskAudio on Core 1) writes to I2S_NUM_1 and drains RingBuffer.
// 2. Reliable Session Handshake: taskAudio flushes old audio before session is activated;
//    sends audio:ready acknowledgement so backend transmits PCM only to a clean buffer.
// 3. Command Queue Resilience: Bounded retries on xQueueSend; emergency atomic cancellation;
//    new sessions cannot start unless command is verified. Zero unsafe fallback writes.
// 4. Real Watermark Resume: taskAudio owns watermark state (pauses at 75%, resumes at 40%),
//    evaluating occupancy continuously during playback even when producer is paused.
// 5. Bounded Callback Execution: enqueuePcm() uses 2ms timeout without polling loops,
//    never stalling WebSocket network callback.
// 6. Strict 24kHz Format Validation: Non-24kHz metadata is strictly rejected.
// 7. Abnormal I2S Partial Write: Preserves 16-bit PCM alignment without duplicate byte resend.
// ============================================================================

enum class AudioCmdType : uint8_t {
    CMD_START_SESSION,
    CMD_CANCEL_SESSION,
    CMD_END_SESSION,
    CMD_FLUSH,
    CMD_SET_VOLUME,
    CMD_CHIME_REQUEST,
    CMD_SPEAKER_TEST
};

struct AudioCommand {
    AudioCmdType type;
    uint32_t generation;
    uint32_t sampleRate;
    uint8_t volume;
    bool isMp3;
    uint16_t initialBufferMs;
    char sessionId[64];
};

class AudioOutputDriver {
private:
    bool initialized = false;
    uint32_t currentSampleRate = AUDIO_OUT_SAMPLE_RATE; // Strictly fixed at 24000 Hz

    // Asynchronous command queue for serialized lifecycle management
    QueueHandle_t audioCmdQueue = nullptr;

    // FreeRTOS RingBuffer for rolling PCM audio streaming
    RingbufHandle_t audioRingBuf = nullptr;
    size_t ringBufCapacity = 0;

    // Atomic session lifecycle state
    std::atomic<uint32_t> activeSessionGeneration{0};
    std::atomic<bool> sessionActive{false};
    std::atomic<bool> sessionCancelled{false};
    std::atomic<bool> isAudioPlaying{false};
    std::atomic<uint8_t> currentVolume{DEFAULT_VOLUME};
    std::atomic<bool> emergencyCancelRequested{false};
    std::atomic<bool> sendReadyAckPending{false};
    std::atomic<bool> flowControlPauseNeeded{false};
    std::atomic<bool> flowControlResumeNeeded{false};

    // Internal playback state owned EXCLUSIVELY by taskAudio on Core 1:
    float gainMultiplier = 0.8f;
    int16_t scaleBuffer[AUDIO_CHUNK_SIZE];
    bool streamEnded = false;
    bool isMp3Stream = false;
    bool prebufferReached = false;
    bool recoveringUnderrun = false;
    uint16_t initialBufferOverrideMs = 0;
    bool isFlowPaused = false; // Watermark state owned strictly by taskAudio
    uint32_t playbackGeneration = 0;
    char activeSessionId[64] = {0};
    unsigned long lastPlayingTime = 0;

    // Non-blocking Cute Chime synthesizer state machine (Owned solely by taskAudio)
    struct CuteChimeSynth {
        bool running = false;
        size_t currentSample = 0;
        size_t totalSamples = 0;
        float phase = 0.0f;
    } chime;

    // Production telemetry (lock-free atomics for accurate diagnostic reporting)
    std::atomic<uint32_t> underrunCount{0};
    std::atomic<uint32_t> overflowCount{0};
    std::atomic<uint32_t> droppedFrames{0};
    std::atomic<uint32_t> i2sWriteErrors{0};
    std::atomic<uint32_t> i2sPartialWrites{0};
    std::atomic<uint32_t> unrecoverableDroppedBytes{0};
    std::atomic<uint32_t> cmdQueueFailures{0};
    std::atomic<uint32_t> ringItemsCheckedOut{0};
    std::atomic<uint32_t> ringItemsReturned{0};
    std::atomic<uint32_t> totalPcmBytesReceived{0};
    std::atomic<uint32_t> totalBytesPlayed{0};

    // Robust I2S write recovery: strictly bounded global attempts and elapsed time, sample-aligned
    size_t writeI2sWithRetry(const uint8_t* data, size_t totalBytes, uint8_t maxAttempts = 4, uint32_t maxElapsedMs = 60) {
        if (!data || totalBytes == 0) return 0;

        size_t bytesRemaining = (totalBytes / sizeof(int16_t)) * sizeof(int16_t);
        const uint8_t* ptr = data;
        uint8_t attempts = 0;
        const unsigned long startTime = millis();

        while (bytesRemaining > 0 && attempts < maxAttempts) {
            if (millis() - startTime >= maxElapsedMs) {
                break; // Global elapsed-time budget exhausted
            }

            attempts++;
            size_t written = 0;
            esp_err_t err = i2s_write(I2S_NUM_1, (const char*)ptr, bytesRemaining, &written, pdMS_TO_TICKS(20));

            if (err != ESP_OK) {
                i2sWriteErrors.fetch_add(1, std::memory_order_relaxed);
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }

            if (written > 0) {
                if (written < bytesRemaining) {
                    i2sPartialWrites.fetch_add(1, std::memory_order_relaxed);
                }

                // Handle partial write ending on an odd byte
                if ((written % sizeof(int16_t)) != 0) {
                    // One byte of a 16-bit sample was sent; the other is orphaned in the buffer.
                    // Advance past both bytes to maintain strict 16-bit PCM alignment.
                    bytesRemaining -= written;
                    ptr += written;
                    if (bytesRemaining > 0) {
                        ptr += 1;
                        bytesRemaining -= 1;
                        unrecoverableDroppedBytes.fetch_add(2, std::memory_order_relaxed);
                    }
                } else {
                    bytesRemaining -= written;
                    ptr += written;
                }
            } else {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }

        size_t totalWritten = totalBytes - bytesRemaining;
        if (bytesRemaining > 0) {
            unrecoverableDroppedBytes.fetch_add(bytesRemaining, std::memory_order_relaxed);
            Serial.printf("[AudioOutput] WARNING: I2S write incomplete. Lost %u bytes (total lost: %lu)\n",
                          (unsigned)bytesRemaining, (unsigned long)unrecoverableDroppedBytes.load());
        }
        totalBytesPlayed.fetch_add(totalWritten / 2, std::memory_order_relaxed);
        return totalWritten;
    }

public:
    static AudioOutputDriver& getInstance() {
        static AudioOutputDriver instance;
        return instance;
    }

    bool begin() {
        Serial.println("[AudioOutput] Initializing MAX98357A Central Audio Manager (I2S_NUM_1)...");

        if (!audioCmdQueue) {
            audioCmdQueue = xQueueCreate(32, sizeof(AudioCommand));
            if (!audioCmdQueue) {
                Serial.println("[AudioOutput] CRITICAL ERROR: Failed to create audioCmdQueue!");
                initialized = false;
                return false;
            }
        }

        i2s_config_t i2sConfig = {
            .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
            .sample_rate = AUDIO_OUT_SAMPLE_RATE, // Enforce 24kHz hardware rate
            .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
            .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
            .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
            .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
            .dma_buf_count = 4,
            .dma_buf_len = 128,
            .use_apll = false,
            .tx_desc_auto_clear = true,
            .fixed_mclk = 0
        };

        i2s_pin_config_t pinConfig = {
            .bck_io_num = PIN_I2S_SPK_BCLK,
            .ws_io_num = PIN_I2S_SPK_LRC,
            .data_out_num = PIN_I2S_SPK_DIN,
            .data_in_num = I2S_PIN_NO_CHANGE
        };

        esp_err_t err = i2s_driver_install(I2S_NUM_1, &i2sConfig, 0, NULL);
        if (err != ESP_OK) {
            Serial.printf("[AudioOutput] CRITICAL ERROR: i2s_driver_install failed with code 0x%X!\n", err);
            initialized = false;
            return false;
        }

        err = i2s_set_pin(I2S_NUM_1, &pinConfig);
        if (err != ESP_OK) {
            Serial.printf("[AudioOutput] CRITICAL ERROR: i2s_set_pin failed with code 0x%X!\n", err);
            i2s_driver_uninstall(I2S_NUM_1);
            initialized = false;
            return false;
        }

        i2s_zero_dma_buffer(I2S_NUM_1);

        if (!audioRingBuf) {
            if (psramFound()) {
                audioRingBuf = xRingbufferCreateWithCaps(AUDIO_RING_BUFFER_BYTES_PSRAM, RINGBUF_TYPE_BYTEBUF, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                if (audioRingBuf) {
                    ringBufCapacity = AUDIO_RING_BUFFER_BYTES_PSRAM;
                    Serial.printf("[AudioOutput] Allocated %u B RingBuffer in Octal PSRAM.\n", (unsigned)ringBufCapacity);
                } else {
                    Serial.println("[AudioOutput] WARNING: PSRAM allocation failed! Falling back to SRAM...");
                }
            }
            if (!audioRingBuf) {
                audioRingBuf = xRingbufferCreate(AUDIO_RING_BUFFER_BYTES_SRAM, RINGBUF_TYPE_BYTEBUF);
                if (audioRingBuf) {
                    ringBufCapacity = AUDIO_RING_BUFFER_BYTES_SRAM;
                    Serial.println("[AudioOutput] Allocated 32 KB RingBuffer in internal SRAM.");
                } else {
                    ringBufCapacity = 0;
                    Serial.println("[AudioOutput] CRITICAL ERROR: Failed to allocate RingBuffer in internal SRAM!");
                    i2s_driver_uninstall(I2S_NUM_1);
                    initialized = false;
                    return false;
                }
            }
        }

        setVolume(DEFAULT_VOLUME);
        initialized = true;
        Serial.println("[AudioOutput] Central Audio Manager initialized successfully (Hardware locked at 24000 Hz).");
        return true;
    }

    // Output sample rate is strictly 24kHz; verifies metadata
    void setSampleRate(uint32_t rate) {
        if (rate != AUDIO_OUT_SAMPLE_RATE && rate != 0) {
            Serial.printf("[AudioOutput] Note: Metadata sample rate %lu Hz. Output hardware is unified at %u Hz.\n",
                          (unsigned long)rate, (unsigned)AUDIO_OUT_SAMPLE_RATE);
        }
    }

    // Thread-safe volume control via command queue with bounded retry (Zero unsafe writes)
    void setVolume(uint8_t vol) {
        if (vol > MAX_VOLUME_LIMIT) vol = MAX_VOLUME_LIMIT;
        currentVolume.store(vol, std::memory_order_release);

        AudioCommand cmd;
        cmd.type = AudioCmdType::CMD_SET_VOLUME;
        cmd.generation = 0;
        cmd.sampleRate = 0;
        cmd.volume = vol;
        cmd.isMp3 = false;
        cmd.sessionId[0] = '\0';

        bool sent = false;
        for (int r = 0; r < 5; r++) {
            if (audioCmdQueue && xQueueSend(audioCmdQueue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE) {
                sent = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        if (!sent) {
            cmdQueueFailures.fetch_add(1, std::memory_order_relaxed);
            Serial.println("[AudioOutput] WARNING: CMD_SET_VOLUME queue send failed after 5 retries.");
        }
    }

    uint8_t getVolume() const {
        return currentVolume.load(std::memory_order_acquire);
    }

    // Start a new audio session with strict 24kHz validation & handshake initiation
    uint32_t startSession(const char* sessionId, uint32_t sRate = 24000, bool isMp3 = false, uint16_t initialBufferMs = 0) {
        // Fix 4: Strictly reject non-24kHz incoming audio metadata
        if (sRate != 0 && sRate != AUDIO_OUT_SAMPLE_RATE) {
            Serial.printf("[AudioOutput] ERROR: Rejected stream. Unsupported sample rate %lu Hz. Hardware is strictly %u Hz!\n",
                          (unsigned long)sRate, (unsigned)AUDIO_OUT_SAMPLE_RATE);
            return 0; // Explicit rejection!
        }

        uint32_t nextGen = activeSessionGeneration.fetch_add(1, std::memory_order_relaxed) + 1;
        if (nextGen == 0) {
            nextGen = activeSessionGeneration.fetch_add(1, std::memory_order_relaxed) + 1;
        }

        // Session is NOT active yet until taskAudio finishes clearing old audio!
        sessionActive.store(false, std::memory_order_release);
        sessionCancelled.store(false, std::memory_order_release);

        AudioCommand cmd;
        cmd.type = AudioCmdType::CMD_START_SESSION;
        cmd.generation = nextGen;
        cmd.sampleRate = sRate;
        cmd.volume = 0;
        cmd.isMp3 = isMp3;
        cmd.initialBufferMs = initialBufferMs;
        if (sessionId) {
            strncpy(cmd.sessionId, sessionId, sizeof(cmd.sessionId) - 1);
            cmd.sessionId[sizeof(cmd.sessionId) - 1] = '\0';
        } else {
            cmd.sessionId[0] = '\0';
        }

        // Fix 2: Bounded retry for command queue send
        bool sent = false;
        for (int r = 0; r < 5; r++) {
            if (audioCmdQueue && xQueueSend(audioCmdQueue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE) {
                sent = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        if (!sent) {
            cmdQueueFailures.fetch_add(1, std::memory_order_relaxed);
            Serial.println("[AudioOutput] CRITICAL: CMD_START_SESSION failed to queue! Session rejected.");
            return 0; // Rejected
        }

        return nextGen;
    }

    // Cancel active audio session immediately (interruption/barge-in) with emergency atomic guarantee
    void cancelSession(const char* reason = nullptr) {
        activeSessionGeneration.fetch_add(1, std::memory_order_relaxed);
        sessionCancelled.store(true, std::memory_order_release);
        sessionActive.store(false, std::memory_order_release);
        emergencyCancelRequested.store(true, std::memory_order_release);

        AudioCommand cmd;
        cmd.type = AudioCmdType::CMD_CANCEL_SESSION;
        cmd.generation = activeSessionGeneration.load(std::memory_order_relaxed);
        cmd.sampleRate = 0;
        cmd.volume = 0;
        cmd.isMp3 = false;
        cmd.sessionId[0] = '\0';

        bool sent = false;
        for (int r = 0; r < 5; r++) {
            if (audioCmdQueue && xQueueSend(audioCmdQueue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE) {
                sent = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        if (!sent) {
            cmdQueueFailures.fetch_add(1, std::memory_order_relaxed);
            Serial.println("[AudioOutput] WARNING: CMD_CANCEL_SESSION queue send failed! Emergency flag active.");
        }

        Serial.printf("[AudioOutput] Session CANCELLED: %s (new gen: %lu)\n",
                      reason ? reason : "user_interrupted", (unsigned long)cmd.generation);
    }

    // End active audio session cleanly (Zero unsafe writes)
    void endSession() {
        sessionActive.store(false, std::memory_order_release);

        AudioCommand cmd;
        cmd.type = AudioCmdType::CMD_END_SESSION;
        cmd.generation = activeSessionGeneration.load(std::memory_order_relaxed);
        cmd.sampleRate = 0;
        cmd.volume = 0;
        cmd.isMp3 = false;
        cmd.sessionId[0] = '\0';

        bool sent = false;
        for (int r = 0; r < 5; r++) {
            if (audioCmdQueue && xQueueSend(audioCmdQueue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE) {
                sent = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        if (!sent) {
            cmdQueueFailures.fetch_add(1, std::memory_order_relaxed);
            Serial.println("[AudioOutput] WARNING: CMD_END_SESSION queue send failed after 5 retries.");
        }
    }

    uint32_t getActiveGeneration() const {
        return activeSessionGeneration.load(std::memory_order_acquire);
    }

    const char* getActiveSessionId() const {
        return activeSessionId;
    }

    bool isPlaying() const {
        return isAudioPlaying.load(std::memory_order_acquire);
    }

    bool isReadyAckPending() const {
        return sendReadyAckPending.load(std::memory_order_acquire);
    }

    void clearReadyAck() {
        sendReadyAckPending.store(false, std::memory_order_release);
    }

    bool isFlowPausePending() const {
        return flowControlPauseNeeded.load(std::memory_order_acquire);
    }

    void clearFlowPause() {
        flowControlPauseNeeded.store(false, std::memory_order_release);
    }

    bool isFlowResumePending() const {
        return flowControlResumeNeeded.load(std::memory_order_acquire);
    }

    void clearFlowResume() {
        flowControlResumeNeeded.store(false, std::memory_order_release);
    }

    bool hasPsramAudioBuffer() const { return ringBufCapacity == AUDIO_RING_BUFFER_BYTES_PSRAM; }

    size_t getFreeBufferBytes() const {
        return audioRingBuf ? xRingbufferGetCurFreeSize(audioRingBuf) : 0;
    }

    // Bounded PCM enqueue: fast 2ms timeout, zero blocking polling loop
    bool enqueuePcm(const void* samples, size_t sampleCount, uint32_t streamGen) {
        if (!initialized || !audioRingBuf || !samples || sampleCount == 0) return false;

        // Disallow 0 or mismatched generation or cancelled or inactive session
        if (streamGen == 0 ||
            streamGen != activeSessionGeneration.load(std::memory_order_acquire) ||
            sessionCancelled.load(std::memory_order_acquire) ||
            !sessionActive.load(std::memory_order_acquire)) {
            droppedFrames.fetch_add(1, std::memory_order_relaxed);
            return false;
        }

        size_t bytes = sampleCount * sizeof(int16_t);

        // Bounded wait of 2ms: protects WebSocket callback from ever blocking
        BaseType_t res = xRingbufferSend(audioRingBuf, samples, bytes, pdMS_TO_TICKS(2));
        if (res != pdTRUE) {
            overflowCount.fetch_add(1, std::memory_order_relaxed);
            droppedFrames.fetch_add(1, std::memory_order_relaxed);
            return false;
        }

        totalPcmBytesReceived.fetch_add(bytes, std::memory_order_relaxed);
        return true;
    }

    void triggerCuteChime(bool speakerTest = false) {
        AudioCommand cmd;
        cmd.type = speakerTest ? AudioCmdType::CMD_SPEAKER_TEST : AudioCmdType::CMD_CHIME_REQUEST;
        cmd.generation = 0;
        cmd.sampleRate = 0;
        cmd.volume = 0;
        cmd.isMp3 = false;
        cmd.sessionId[0] = '\0';

        bool sent = false;
        for (int r = 0; r < 5; r++) {
            if (audioCmdQueue && xQueueSend(audioCmdQueue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE) {
                sent = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        if (!sent) {
            cmdQueueFailures.fetch_add(1, std::memory_order_relaxed);
        }
    }

    // Thread-safe flush request
    void stopAndFlush() {
        cancelSession("flush_stop");
    }

    bool isSessionActive() const { return sessionActive.load(std::memory_order_acquire); }

    // Thread-safe check if buffered audio or DMA is active
    bool hasBufferedAudio() const {
        if (!initialized) return false;
        if (isAudioPlaying.load(std::memory_order_acquire)) return true;
        if (audioRingBuf && ringBufCapacity > 0) {
            size_t freeBytes = xRingbufferGetCurFreeSize(audioRingBuf);
            if (ringBufCapacity > freeBytes && (ringBufCapacity - freeBytes) > 512) {
                return true;
            }
        }
        return false;
    }

    // ========================================================================
    // SINGLE DEDICATED AUDIO UPDATE METHOD (ONLY called by taskAudio on Core 1)
    // ========================================================================
    bool update() {
        if (!initialized) return false;

        // 0. Handle emergency atomic cancellation (if command queue was ever full)
        if (emergencyCancelRequested.exchange(false, std::memory_order_acq_rel)) {
            if (audioRingBuf) {
                size_t size = 0;
                void* itm = nullptr;
                while ((itm = xRingbufferReceiveUpTo(audioRingBuf, &size, 0, AUDIO_CHUNK_SIZE)) != nullptr) {
                    vRingbufferReturnItem(audioRingBuf, itm);
                }
            }
            i2s_zero_dma_buffer(I2S_NUM_1);
            streamEnded = false;
            prebufferReached = false;
            isFlowPaused = false;
            chime.running = false;
            activeSessionId[0] = '\0';
            isAudioPlaying.store(false, std::memory_order_release);
        }

        // 1. Process asynchronous commands from audioCmdQueue in strict FIFO order
        AudioCommand cmd;
        while (audioCmdQueue && xQueueReceive(audioCmdQueue, &cmd, 0) == pdTRUE) {
            switch (cmd.type) {
                case AudioCmdType::CMD_START_SESSION: {
                    // Flush any stale data from previous sessions completely before starting new playback
                    if (audioRingBuf) {
                        size_t size = 0;
                        void* itm = nullptr;
                        while ((itm = xRingbufferReceiveUpTo(audioRingBuf, &size, 0, AUDIO_CHUNK_SIZE)) != nullptr) {
                            vRingbufferReturnItem(audioRingBuf, itm);
                        }
                    }
                    i2s_zero_dma_buffer(I2S_NUM_1);
                    recoveringUnderrun = false;
                    playbackGeneration = cmd.generation;
                    streamEnded = false;
                    prebufferReached = false;
                    isMp3Stream = cmd.isMp3;
                    initialBufferOverrideMs = cmd.initialBufferMs;
                    isFlowPaused = false; // Reset flow pause state for new session
                    strncpy(activeSessionId, cmd.sessionId, sizeof(activeSessionId) - 1);
                    activeSessionId[sizeof(activeSessionId) - 1] = '\0';
                    chime.running = false;
                    isAudioPlaying.store(false, std::memory_order_release);

                    // Fix 1: Now that previous audio is 100% flushed, activate session & trigger handshake ack
                    sessionActive.store(true, std::memory_order_release);
                    sendReadyAckPending.store(true, std::memory_order_release);
                    break;
                }
                case AudioCmdType::CMD_CANCEL_SESSION:
                case AudioCmdType::CMD_FLUSH: {
                    if (audioRingBuf) {
                        size_t size = 0;
                        void* itm = nullptr;
                        while ((itm = xRingbufferReceiveUpTo(audioRingBuf, &size, 0, AUDIO_CHUNK_SIZE)) != nullptr) {
                            vRingbufferReturnItem(audioRingBuf, itm);
                        }
                    }
                    i2s_zero_dma_buffer(I2S_NUM_1);
                    streamEnded = false;
                    prebufferReached = false;
                    isFlowPaused = false; // Reset flow pause state
                    chime.running = false;
                    activeSessionId[0] = '\0';
                    isAudioPlaying.store(false, std::memory_order_release);
                    break;
                }
                case AudioCmdType::CMD_END_SESSION: {
                    streamEnded = true; // Signal update() that buffer may drain to completion
                    isFlowPaused = false; // Ensure flow pause state reset
                    break;
                }
                case AudioCmdType::CMD_SET_VOLUME: {
                    gainMultiplier = ((float)cmd.volume / 100.0f) * 0.95f;
                    break;
                }
                case AudioCmdType::CMD_CHIME_REQUEST:
                case AudioCmdType::CMD_SPEAKER_TEST: {
                    size_t curFree = audioRingBuf ? xRingbufferGetCurFreeSize(audioRingBuf) : 0;
                    size_t curBuffered = (ringBufCapacity > curFree) ? (ringBufCapacity - curFree) : 0;
                    if (!sessionActive.load(std::memory_order_acquire) && curBuffered == 0) {
                        chime.running = true;
                        chime.currentSample = 0;
                        chime.totalSamples = cmd.type == AudioCmdType::CMD_SPEAKER_TEST
                            ? AUDIO_OUT_SAMPLE_RATE * 3
                            : (AUDIO_OUT_SAMPLE_RATE * 650) / 1000;
                        chime.phase = 0.0f;
                    }
                    break;
                }
            }
        }

        // 2. Measure RingBuffer occupancy & Watermark Flow Control (Fix 1: Owned strictly by taskAudio!)
        size_t curFree = audioRingBuf ? xRingbufferGetCurFreeSize(audioRingBuf) : 0;
        size_t bufferedBytes = (ringBufCapacity > curFree) ? (ringBufCapacity - curFree) : 0;

        // Keep a larger reserve for library tracks while continuing to play locally.
        const bool libraryTrack = strncmp(activeSessionId, "playback_", 9) == 0;
        const size_t resumePercent = libraryTrack && ringBufCapacity == AUDIO_RING_BUFFER_BYTES_PSRAM ? 55 : 40;
        if (sessionActive.load(std::memory_order_acquire)) {
            if (!isFlowPaused && bufferedBytes >= (ringBufCapacity * 3) / 4) {
                isFlowPaused = true;
                flowControlPauseNeeded.store(true, std::memory_order_release);
            } else if (isFlowPaused && bufferedBytes <= (ringBufCapacity * resumePercent) / 100) {
                isFlowPaused = false;
                flowControlResumeNeeded.store(true, std::memory_order_release);
            }
        }

        // 3. SOURCE-PRIORITY POLICY:
        // Streaming Voice Audio takes precedence over Cute Chime.
        // If voice audio is buffered or session is active, Chime yields immediately!
        if ((bufferedBytes > 256 || sessionActive.load(std::memory_order_acquire)) && chime.running) {
            chime.running = false; // Gracefully yield to speech
        }

        // Let voice recover from a brief network gap without a long pause mid-sentence.
        if (!prebufferReached && ringBufCapacity) {
            constexpr size_t pcmBytesPerMs = AUDIO_OUT_SAMPLE_RATE * sizeof(int16_t) / 1000;
            const size_t initialMs = initialBufferOverrideMs ? initialBufferOverrideMs : (libraryTrack ? 900 : 1000);
            const size_t recoveryMs = libraryTrack ? 950 : 1200;
            const size_t target = std::min((recoveringUnderrun ? recoveryMs : initialMs) * pcmBytesPerMs,
                                           ringBufCapacity / 2);
            if (bufferedBytes >= target || streamEnded) prebufferReached = true;
        }

        // 5. Stream audio from RingBuffer to I2S (High-Priority Path with Guaranteed Item Return)
        bool hasData = false;
        if (audioRingBuf && prebufferReached && bufferedBytes > 0) {
            size_t itemSize = 0;
            void* item = xRingbufferReceiveUpTo(audioRingBuf, &itemSize, 0, (AUDIO_CHUNK_SIZE / 2) * sizeof(int16_t));

            if (item != nullptr) {
                ringItemsCheckedOut.fetch_add(1, std::memory_order_relaxed);

                if (itemSize > 0) {
                    hasData = true;
                    isAudioPlaying.store(true, std::memory_order_release);
                    lastPlayingTime = millis();

                    // If session was cancelled, skip physical write but STILL return item
                    if (!sessionCancelled.load(std::memory_order_acquire)) {
                        const int16_t* pcmIn = (const int16_t*)item;
                        size_t samples = itemSize / sizeof(int16_t);
                        if (samples > (AUDIO_CHUNK_SIZE / 2)) samples = (AUDIO_CHUNK_SIZE / 2);

                        for (size_t i = 0; i < samples; i++) {
                            int32_t val = (int32_t)(pcmIn[i] * gainMultiplier);
                            if (val > 30000) val = 30000;
                            if (val < -30000) val = -30000;
                            scaleBuffer[2*i] = scaleBuffer[2*i + 1] = (int16_t)val;
                        }

                        writeI2sWithRetry((const uint8_t*)scaleBuffer, 2 * samples * sizeof(int16_t));
                    }
                }

                // GUARANTEED ZERO-LOCK ITEM RETURN:
                // FreeRTOS RingBuffer API is intrinsically thread-safe.
                // Returned immediately with 0 retries and 0 mutex overhead.
                vRingbufferReturnItem(audioRingBuf, item);
                ringItemsReturned.fetch_add(1, std::memory_order_relaxed);

                return true;
            }
        } else if (isAudioPlaying.load(std::memory_order_acquire) && millis() - lastPlayingTime > 35) {
            isAudioPlaying.store(false, std::memory_order_release);
            if (prebufferReached && !streamEnded && sessionActive.load(std::memory_order_acquire)) {
                underrunCount.fetch_add(1, std::memory_order_relaxed);
                recoveringUnderrun = true;
                prebufferReached = false;
            }
        }

        // 6. Cute Chime synthesis state machine (Low-Priority Path)
        if (chime.running) {
            size_t batch = (chime.totalSamples - chime.currentSample < 128) ?
                           (chime.totalSamples - chime.currentSample) : 128;
            for (size_t b = 0; b < batch; b++) {
                float t = (float)(chime.currentSample + b) / (float)AUDIO_OUT_SAMPLE_RATE;
                float freq = 523.25f; // C5
                float noteProgress = 0.0f;

                if (t < 0.12f) { freq = 523.25f; noteProgress = t / 0.12f; }
                else if (t < 0.24f) { freq = 659.25f; noteProgress = (t - 0.12f) / 0.12f; }
                else if (t < 0.38f) { freq = 783.99f; noteProgress = (t - 0.24f) / 0.14f; }
                else { freq = 1046.50f; noteProgress = (t - 0.38f) / 0.27f; }

                float env = (noteProgress < 0.08f) ? (noteProgress / 0.08f) : powf(1.0f - ((noteProgress - 0.08f) / 0.92f), 2.0f);
                if (t >= 0.38f) freq += 12.0f * sinf(2.0f * PI * 6.5f * (t - 0.38f));

                chime.phase += 2.0f * PI * freq / (float)AUDIO_OUT_SAMPLE_RATE;
                if (chime.phase >= 2.0f * PI) chime.phase -= 2.0f * PI;

                float sample = (sinf(chime.phase) + 0.3f * sinf(2.0f * chime.phase)) * env * 18000.0f * ((float)currentVolume.load(std::memory_order_relaxed) / 100.0f);
                scaleBuffer[2*b] = scaleBuffer[2*b + 1] = (int16_t)sample;
            }
            chime.currentSample += batch;
            if (chime.currentSample >= chime.totalSamples) {
                chime.running = false;
            }

            writeI2sWithRetry((const uint8_t*)scaleBuffer, 2 * batch * sizeof(int16_t), 3, 30);
            isAudioPlaying.store(true, std::memory_order_release);
            lastPlayingTime = millis();
            return true;
        }

        return hasData;
    }

    // Production health, memory and recovery telemetry
    void printDiagnostics(TaskHandle_t audioHandle = nullptr) {
        size_t freeHeap = ESP.getFreeHeap();
        size_t minHeap = ESP.getMinFreeHeap();
        size_t maxBlock = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
        size_t freePsram = ESP.getFreePsram();
        size_t totalPsram = ESP.getPsramSize();
        size_t freeRing = audioRingBuf ? xRingbufferGetCurFreeSize(audioRingBuf) : 0;
        size_t occRing = (ringBufCapacity > freeRing) ? (ringBufCapacity - freeRing) : 0;
        UBaseType_t stackHighWater = audioHandle ? uxTaskGetStackHighWaterMark(audioHandle) : 0;

        Serial.printf("[AUDIO-DIAG] Heap: %u B (Min: %u B, MaxBlk: %u B) | PSRAM: %u/%u KB\n",
                      (unsigned)freeHeap, (unsigned)minHeap, (unsigned)maxBlock,
                      (unsigned)(freePsram / 1024), (unsigned)(totalPsram / 1024));
        Serial.printf("[AUDIO-DIAG] RingBuf (PSRAM): %u/%u B (%u%%) | Under: %lu | Over: %lu | Drop: %lu | ItemOut/Ret: %lu/%lu (Leak: %ld)\n",
                      (unsigned)occRing, (unsigned)ringBufCapacity,
                      ringBufCapacity > 0 ? (unsigned)((occRing * 100) / ringBufCapacity) : 0,
                      (unsigned long)underrunCount.load(), (unsigned long)overflowCount.load(), (unsigned long)droppedFrames.load(),
                      (unsigned long)ringItemsCheckedOut.load(), (unsigned long)ringItemsReturned.load(),
                      (long)(ringItemsCheckedOut.load() - ringItemsReturned.load()));
        Serial.printf("[AUDIO-DIAG] I2S: Partial: %lu | LostBytes: %lu | Played: %lu B | StackFree: %u words | CmdQueueDrops: %lu\n",
                      (unsigned long)i2sPartialWrites.load(), (unsigned long)unrecoverableDroppedBytes.load(),
                      (unsigned long)totalBytesPlayed.load(), (unsigned)stackHighWater, (unsigned long)cmdQueueFailures.load());
        const unsigned long now = millis();
        const uint32_t received = totalPcmBytesReceived.load(std::memory_order_relaxed);
        const uint32_t played = totalBytesPlayed.load(std::memory_order_relaxed);
        static unsigned long previousTime = 0;
        static uint32_t previousReceived = 0, previousPlayed = 0;
        if (previousTime && now != previousTime) {
            const unsigned long interval = now - previousTime;
            Serial.printf("[AUDIO-DIAG] Avg over %lu ms: RX %lu B/s | I2S %lu B/s | WriteErrors %lu\n",
                          interval, (unsigned long)((uint64_t)(received - previousReceived) * 1000 / interval),
                          (unsigned long)((uint64_t)(played - previousPlayed) * 1000 / interval),
                          (unsigned long)i2sWriteErrors.load());
        }
        previousTime = now;
        previousReceived = received;
        previousPlayed = played;
        Serial.printf("[AUDIO-DIAG] Session: %s (ActiveGen: %lu, PlayGen: %lu) | State: %s | Prebuf: %s | Flow: %s\n",
                      activeSessionId[0] ? activeSessionId : "<none>",
                      (unsigned long)activeSessionGeneration.load(),
                      (unsigned long)playbackGeneration,
                      sessionActive.load() ? "ACTIVE" : (chime.running ? "CHIME" : "IDLE"),
                      prebufferReached ? "READY" : "BUFFERING",
                      isFlowPaused ? "PAUSED" : "FLOWING");
    }
};

// ============================================================================
// SECTION 8: NON-BLOCKING I2S AUDIO INPUT (INMP441 Microphone Driver)
// ============================================================================
class AudioInputDriver {
private:
    bool initialized = false;
    volatile bool capturing = false;
    int32_t rawDmaBuffer[AUDIO_CHUNK_SIZE];
    unsigned micSlot = 0;

public:
    static AudioInputDriver& getInstance() {
        static AudioInputDriver instance;
        return instance;
    }

    bool begin() {
        Serial.println("[AudioInput] Initializing INMP441 (I2S_NUM_0)...");

        i2s_config_t i2sConfig = {
            .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
            .sample_rate = AUDIO_IN_SAMPLE_RATE,
            .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
            .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
            .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
            .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
            .dma_buf_count = AUDIO_DMA_BUF_COUNT,
            .dma_buf_len = AUDIO_DMA_BUF_LEN,
            .use_apll = false,
            .tx_desc_auto_clear = false,
            .fixed_mclk = 0
        };

        i2s_pin_config_t pinConfig = {
            .bck_io_num = PIN_I2S_MIC_SCK,
            .ws_io_num = PIN_I2S_MIC_WS,
            .data_out_num = I2S_PIN_NO_CHANGE,
            .data_in_num = PIN_I2S_MIC_SD
        };

        if (i2s_driver_install(I2S_NUM_0, &i2sConfig, 0, NULL) != ESP_OK) {
            Serial.println("[AudioInput] ERROR: i2s_driver_install failed!");
            return false;
        }

        if (i2s_set_pin(I2S_NUM_0, &pinConfig) != ESP_OK) {
            Serial.println("[AudioInput] ERROR: i2s_set_pin failed!");
            return false;
        }

        i2s_zero_dma_buffer(I2S_NUM_0);
        i2s_start(I2S_NUM_0);
        initialized = true;
        Serial.println("[AudioInput] INMP441 driver initialized successfully.");
        return true;
    }

    void startRecording() {
        if (!initialized) return;
        i2s_zero_dma_buffer(I2S_NUM_0);
        capturing = true;
    }

    void stopRecording() {
        capturing = false;
    }

    bool isRecording() const { return capturing; }

    // DMA read for dedicated capture task (32ms nominal chunk at 16kHz, 40ms timeout)
    size_t readPcmChunk(int16_t* outBuf, size_t maxSamples) {
        if (!initialized || !outBuf) return 0;

        size_t bytesRead = 0;
        esp_err_t err = i2s_read(I2S_NUM_0, rawDmaBuffer, 2 * std::min<size_t>(maxSamples, AUDIO_CHUNK_SIZE / 2) * sizeof(int32_t), &bytesRead, pdMS_TO_TICKS(40));
        if (err != ESP_OK || bytesRead == 0) return 0;

        size_t samples = bytesRead / (2 * sizeof(int32_t));
        if (samples == 0) return 0;
        // INMP441 drives one slot according to its L/R strap. Ignore DC on the unused slot.
        int32_t low[2] = {32767, 32767}, high[2] = {-32768, -32768};
        for (size_t i = 0; i < samples; ++i) {
            for (unsigned slot = 0; slot < 2; ++slot) {
                const int32_t value = rawDmaBuffer[2*i + slot] >> 16;
                low[slot] = std::min(low[slot], value);
                high[slot] = std::max(high[slot], value);
            }
        }
        const unsigned other = 1 - micSlot;
        const int32_t otherRange = high[other] - low[other];
        if (otherRange > 16 && otherRange > 4 * (high[micSlot] - low[micSlot])) micSlot = other;
        static unsigned long lastMicReport = 0;
        if (millis() - lastMicReport >= 5000) {
            Serial.printf("[Mic] slot=%u p-p=%ld/%ld\n", micSlot, (long)(high[0]-low[0]), (long)(high[1]-low[1]));
            lastMicReport = millis();
        }
        for (size_t i = 0; i < samples; i++) {
            int32_t s = rawDmaBuffer[2*i + micSlot] >> 14;
            if (s > 32767) s = 32767;
            if (s < -32768) s = -32768;
            outBuf[i] = (int16_t)s;
        }
        return samples;
    }
};

// ============================================================================
// SECTION 8b: LOCAL ESP-SR WAKENET9 WAKE-WORD DETECTION ENGINE
// ============================================================================
// ARCHITECTURE RULES (WAKENET9 / AUDIO INPUT INTEGRATION):
// 1. Zero cloud transmission during IDLE standby (privacy preserved).
// 2. Continuous 16kHz 16-bit Mono PCM audio feed from INMP441.
// 3. Circular pre-roll buffer (1024 samples = 64ms) captures speech immediately following wake word.
// 4. Speaker echo prevention: WakeNet processing is paused while speaker is actively playing.
// 5. Dual activation: TTP223 capacitive touch remains 100% active as alternative & fallback.
// ============================================================================

enum class WakeNetStatus {
    UNINITIALIZED,
    MODEL_PARTITION_NOT_FOUND,
    MODEL_LOADED,
    READY_ACTIVE_LISTENING,
    WAKE_DETECTED,
    PAUSED
};

class WakeWordEngine {
private:
    WakeNetStatus status = WakeNetStatus::UNINITIALIZED;
    bool modelLoaded = false;
    std::atomic<bool> activeListening{false};
    const char* activeModelName = WAKE_WORD_MODEL_NAME;
    const char* activeWakeWord = WAKE_WORD_PHRASE;

    srmodel_list_t* srModels = nullptr;
    const esp_wn_iface_t* wakenetIface = nullptr;
    model_iface_data_t* modelData = nullptr;
    int chunkSampleSize = 512;

    // Rolling circular pre-roll buffer (1024 samples = 64ms at 16kHz Mono)
    static const size_t PRE_ROLL_SAMPLES = 1024;
    int16_t preRollBuffer[PRE_ROLL_SAMPLES] = {0};
    size_t preRollHead = 0;
    bool validPreRoll = false;

    // Neural network sample accumulator
    int16_t accumBuffer[1024] = {0};
    size_t accumCount = 0;
    std::atomic<bool> resetAccum{false};

    unsigned long lastDetectionTime = 0;
    uint32_t totalDetections = 0;

public:
    static WakeWordEngine& getInstance() {
        static WakeWordEngine instance;
        return instance;
    }

    bool begin() {
        Serial.println("[WakeWord] Initializing ESP-SR WakeNet9 Engine...");

        // 1. Mount model partition from flash (offset 0xC10000)
        srModels = esp_srmodel_init("model");
        if (!srModels) {
            Serial.println("[WakeWord] WARNING: Flash 'model' partition not found or empty at 0xC10000.");
            Serial.println("[WakeWord] Running in TTP223 Head-Touch / Touchscreen mode (Flash 'srmodels.bin' to offset 0xC10000 to enable WakeNet9).");
            status = WakeNetStatus::MODEL_PARTITION_NOT_FOUND;
            return false;
        }

        char* foundName = esp_srmodel_filter(srModels, ESP_WN_PREFIX, "hiesp");
        if (foundName) {
            activeModelName = foundName;
            Serial.printf("[WakeWord] Model partition loaded: %s\n", activeModelName);
        }

        // 2. Obtain WakeNet interface handle for target model
        wakenetIface = esp_wn_handle_from_name(activeModelName);
        if (!wakenetIface) {
            wakenetIface = &esp_sr_wakenet9_quantized;
        }

        if (wakenetIface) {
            modelData = wakenetIface->create(activeModelName, DET_MODE_90);
            if (modelData) {
                chunkSampleSize = wakenetIface->get_samp_chunksize(modelData);
                char* word = wakenetIface->get_word_name(modelData, 1);
                if (word) activeWakeWord = word;
                // Keep the detection threshold calibrated for this model.

                modelLoaded = true;
                status = WakeNetStatus::READY_ACTIVE_LISTENING;
                activeListening = true;
                Serial.printf("[WakeWord] Model: %s (\"%s\") -> Wake-Word Detection READY.\n", activeModelName, activeWakeWord);
                return true;
            } else {
                Serial.println("[WakeWord] Initialization failed: WakeNet create returned NULL.");
                status = WakeNetStatus::MODEL_PARTITION_NOT_FOUND;
            }
        } else {
            Serial.println("[WakeWord] Initialization failed: wakenet handle not found.");
            status = WakeNetStatus::UNINITIALIZED;
        }
        return false;
    }

    void setListening(bool listening) {
        resetAccum.store(true, std::memory_order_release);
        activeListening.store(listening, std::memory_order_release);
        if (listening) {
            Serial.println("[WakeWord] State: IDLE_LISTEN (re-armed)");
        } else {
            Serial.println("[WakeWord] State: DISARMED");
        }
    }

    bool isListening() const { return activeListening.load(std::memory_order_acquire); }
    bool isModelLoaded() const { return modelLoaded; }
    void preservePreRoll() { validPreRoll = true; }
    bool hasPreRoll() const { return validPreRoll; }
    WakeNetStatus getStatus() const { return status; }
    const char* getActiveWakeWord() const { return activeWakeWord; }
    const char* getModelName() const { return activeModelName; }
    uint32_t getDetectionCount() const { return totalDetections; }

    size_t getPreRollAudio(int16_t* outBuf, size_t maxSamples) {
        if (!outBuf || maxSamples == 0 || !validPreRoll) return 0;
        size_t count = (maxSamples < PRE_ROLL_SAMPLES) ? maxSamples : PRE_ROLL_SAMPLES;
        for (size_t i = 0; i < count; i++) {
            size_t idx = (preRollHead + i) % PRE_ROLL_SAMPLES;
            outBuf[i] = preRollBuffer[idx];
        }
        validPreRoll = false;
        return count;
    }

    void remember(const int16_t* pcmSamples, size_t sampleCount) {
        // 1. Maintain rolling circular pre-roll buffer
        for (size_t i = 0; i < sampleCount; i++) {
            preRollBuffer[preRollHead] = pcmSamples[i];
            preRollHead = (preRollHead + 1) % PRE_ROLL_SAMPLES;
        }

    }

    bool processPcmChunk(const int16_t* pcmSamples, size_t sampleCount) {
        if (!activeListening.load(std::memory_order_acquire) || !modelLoaded || !modelData || !wakenetIface || !pcmSamples || sampleCount == 0) {
            return false;
        }

        if (resetAccum.exchange(false)) accumCount = 0;

        remember(pcmSamples, sampleCount);

        // 2. Feed samples to WakeNet neural inference engine
        size_t offset = 0;
        while (offset < sampleCount) {
            size_t needed = (chunkSampleSize > (int)accumCount) ? (chunkSampleSize - accumCount) : 0;
            size_t toCopy = (sampleCount - offset < needed) ? (sampleCount - offset) : needed;

            memcpy(accumBuffer + accumCount, pcmSamples + offset, toCopy * sizeof(int16_t));
            accumCount += toCopy;
            offset += toCopy;

            if (accumCount >= (size_t)chunkSampleSize) {
                wakenet_state_t result = wakenetIface->detect(modelData, accumBuffer);
                accumCount = 0;

                unsigned long now = millis();
                if (result == WAKENET_DETECTED && ((!totalDetections || now - lastDetectionTime >= WAKE_DETECTION_COOLDOWN_MS))) {
                    lastDetectionTime = now;
                    totalDetections++;
                    validPreRoll = false;
                    memset(preRollBuffer, 0, sizeof(preRollBuffer));
                    preRollHead = 0;
                    activeListening.store(false, std::memory_order_release);
                    status = WakeNetStatus::WAKE_DETECTED;

                    Serial.printf("[WakeWord] DETECTED: \"%s\"!\n", activeWakeWord);
                    return true;
                }
            }
        }
        return false;
    }
};

// ============================================================================
// SECTION 9: WI-FI MANAGER & WEB CAPTIVE PROVISIONING PORTAL
// ============================================================================
// CRITICAL: Declared BEFORE UIManager and DeviceManager so no compile error!
enum class WifiState {
    DISCONNECTED,
    CONNECTING,
    CONNECTED,
    PROVISIONING_PORTAL
};

class WiFiManagerDriver {
private:
    WifiState state = WifiState::DISCONNECTED;
    bool portalActive = false;
    std::unique_ptr<WebServer> server;
    std::unique_ptr<DNSServer> dnsServer;
    unsigned long lastReconnectAttempt = 0;
    unsigned long reconnectDelay = 2000;
    const unsigned long maxReconnectDelay = 30000;
    uint32_t failedAttempts = 0;
    bool sntpConfigured = false;
    bool sntpSynchronized = false;

    void handlePortalRoot() {
        if (!server) return;
        String curSsid = ConfigManager::getInstance().getWifiSSID();
        String curDevId = ConfigManager::getInstance().getDeviceId();

        String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
                      "<title>Memo Robot Setup</title><style>"
                      "body{font-family:-apple-system,BlinkMacSystemFont,Arial,sans-serif;background:#0d1b2a;color:#e0e1dd;margin:0;padding:20px;text-align:center;}"
                      ".card{background:#1b263b;max-width:380px;margin:20px auto;padding:24px;border-radius:12px;box-shadow:0 8px 24px rgba(0,0,0,0.6);border:1px solid #415a77;}"
                      "h2{margin-top:0;color:#00e5ff;}"
                      "label{display:block;text-align:left;margin:10px 0 4px;font-size:13px;color:#a8dadc;}"
                      "input{width:100%;box-sizing:border-box;padding:12px;border-radius:8px;border:1px solid #415a77;background:#0d1b2a;color:#fff;font-size:15px;margin-bottom:10px;}"
                      "button{width:100%;background:#00e676;color:#000;font-weight:bold;font-size:16px;padding:14px;border:none;border-radius:8px;cursor:pointer;margin-top:14px;box-shadow:0 4px 12px rgba(0,230,118,0.3);}"
                      "button:hover{background:#00c853;}"
                      ".info{font-size:12px;color:#8d99ae;margin-top:14px;line-height:1.5;}"
                      "</style></head><body><div class='card'>"
                      "<h2>🤖 Memo AI Robot Setup</h2>"
                      "<p style='color:#a8dadc;font-size:14px;'>Wi-Fi နှင့် Device အချက်အလက်များ ထည့်သွင်းပါ</p>"
                      "<form action='/save' method='POST'>"
                      "<label>Wi-Fi Network Name (SSID):</label>"
                      "<input type='text' name='ssid' value='" + curSsid + "' placeholder='အိမ်သုံး Wi-Fi အမည်' required>"
                      "<label>Wi-Fi Password:</label>"
                      "<input type='password' name='pass' placeholder='Wi-Fi စကားဝှက်'>"
                      "<label>Device ID:</label>"
                      "<input type='text' name='dev_id' value='" + curDevId + "' placeholder='Device ID' required>"
                      "<label>Device API Key:</label>"
                      "<input type='password' name='api_key' placeholder='rbt_... API Key'>"
                      "<button type='submit'>💾 Save & Connect (သိမ်းဆည်းမည်)</button>"
                      "</form>"
                      "<div class='info'>သိမ်းဆည်းပြီးပါက Robot သည် အလိုအလျောက် Restart ပြုလုပ်ကာ Wi-Fi သို့ ချိတ်ဆက်သွားမည်ဖြစ်ပါသည်။</div>"
                      "</div></body></html>";
        server->send(200, "text/html", html);
    }

    void handlePortalSave() {
        if (!server) return;
        String ssid = server->arg("ssid");
        String pass = server->arg("pass");
        String devId = server->arg("dev_id");
        String apiKey = server->arg("api_key");

        ssid.trim();
        pass.trim();
        devId.trim();
        apiKey.trim();

        if (ssid.length() > 0) {
            ConfigManager::getInstance().setWifiCredentials(ssid, pass);
            if (devId.length() > 0 && apiKey.length() > 0) {
                ConfigManager::getInstance().setDeviceCredentials(devId, apiKey);
            }

            String response = "<!DOCTYPE html><html><body style='background:#0d1b2a;color:#fff;text-align:center;font-family:Arial;padding:40px;'>"
                              "<h2 style='color:#00e676;'>✓ Credentials Saved!</h2>"
                              "<p>Wi-Fi: <b>" + ssid + "</b> သို့ စတင်ချိတ်ဆက်နေပါပြီ...</p>"
                              "<p>Memo Robot is restarting now...</p></body></html>";
            server->send(200, "text/html", response);
            delay(1500);
            ESP.restart();
        } else {
            server->send(400, "text/plain", "Error: Wi-Fi SSID cannot be empty.");
        }
    }

public:
    static WiFiManagerDriver& getInstance() {
        static WiFiManagerDriver instance;
        return instance;
    }

    bool isPortalActive() const { return portalActive; }
    bool isSntpSynced() const { return sntpSynchronized; }
    int8_t getRSSI() const { return (state == WifiState::CONNECTED) ? WiFi.RSSI() : 0; }
    String getIpAddress() const { return (state == WifiState::CONNECTED) ? WiFi.localIP().toString() : "0.0.0.0"; }
    bool isConnected() const { return (state == WifiState::CONNECTED && WiFi.status() == WL_CONNECTED); }

    void begin() {
        WiFi.mode(WIFI_STA);
        WiFi.setSleep(false); // Keep the Wi-Fi link responsive for the WebSocket/audio stream
        WiFi.setAutoReconnect(false); // Controlled backoff avoids driver thrashing
        WiFi.setTxPower(WIFI_POWER_17dBm);

        String ssid = ConfigManager::getInstance().getWifiSSID();
        String pass = ConfigManager::getInstance().getWifiPassword();
        Serial.printf("[WiFi] Stored credentials: SSID=%s, password=%s\n",
                      ssid.length() ? "YES" : "NO", pass.length() ? "YES" : "NO");
        if (ssid.length() > 0) {
            Serial.printf("[WiFi] Connecting to SSID: %s ...\n", ssid.c_str());
            WiFi.begin(ssid.c_str(), pass.c_str());
            state = WifiState::CONNECTING;
            lastReconnectAttempt = millis();
        } else {
            Serial.println("[WiFi] No credentials found. Starting Provisioning Portal...");
            startProvisioningPortal();
        }
    }

    void startProvisioningPortal() {
        if (state == WifiState::PROVISIONING_PORTAL) return;
        state = WifiState::PROVISIONING_PORTAL;
        portalActive = true;

        Serial.println("\n[WiFiPortal] Launching SoftAP & Web Provisioning Portal...");
        WiFi.disconnect(true);
        delay(100);
        WiFi.mode(WIFI_AP);
        WiFi.softAP(PROV_AP_SSID, PROV_AP_PASSWORD);

        dnsServer = std::unique_ptr<DNSServer>(new DNSServer());
        dnsServer->start(53, "*", WiFi.softAPIP());

        server = std::unique_ptr<WebServer>(new WebServer(80));
        server->on("/", HTTP_GET, [this]() { handlePortalRoot(); });
        server->on("/save", HTTP_POST, [this]() { handlePortalSave(); });
        server->onNotFound([this]() {
            server->sendHeader("Location", "/", true);
            server->send(302, "text/plain", "");
        });
        server->begin();

        Serial.printf("[WiFiPortal] AP SSID: ESP32-Robot-Setup | Web Portal: http://%s\n\n", WiFi.softAPIP().toString().c_str());
    }

    void stopProvisioningPortal() {
        if (state != WifiState::PROVISIONING_PORTAL) return;
        portalActive = false;
        if (server) { server->stop(); server.reset(); }
        if (dnsServer) { dnsServer->stop(); dnsServer.reset(); }
        WiFi.softAPdisconnect(true);
        state = WifiState::DISCONNECTED;
        begin();
    }

    void update() {
        unsigned long now = millis();

        if (state == WifiState::PROVISIONING_PORTAL) {
            if (dnsServer) dnsServer->processNextRequest();
            if (server) server->handleClient();
            return;
        }

        if (state == WifiState::CONNECTING) {
            if (WiFi.status() == WL_CONNECTED) {
                state = WifiState::CONNECTED;
                failedAttempts = 0;
                reconnectDelay = 2000;
                Serial.printf("[WiFi] Connected! IP: %s (RSSI: %d dBm). Syncing SNTP...\n",
                              WiFi.localIP().toString().c_str(), WiFi.RSSI());

                configTzTime("MMT-6:30", "pool.ntp.org", "time.google.com", "time.windows.com");
                sntpConfigured = true;
            } else if (now - lastReconnectAttempt > 8000) {
                // Fast 8-second connection timeout (prevents long hanging loops)
                state = WifiState::DISCONNECTED;
                failedAttempts++;
                lastReconnectAttempt = now;
                Serial.printf("[WiFi] Connection attempt %u timed out (8s limit).\n", failedAttempts);


            }
        } else if (state == WifiState::CONNECTED) {
            if (WiFi.status() != WL_CONNECTED) {
                state = WifiState::DISCONNECTED;
                sntpConfigured = false;
                sntpSynchronized = false;
                lastReconnectAttempt = now;
                Serial.println("[WiFi] Wi-Fi link lost! Scheduling reconnect...");
            } else {
                if (!sntpSynchronized) {
                    time_t tnow = time(nullptr);
                    if (tnow >= 1704067200) { // Jan 1, 2024 threshold
                        sntpSynchronized = true;
                        struct tm tinfo;
                        localtime_r(&tnow, &tinfo);
                        char tbuf[64];
                        strftime(tbuf, sizeof(tbuf), "%Y-%m-%d %H:%M:%S", &tinfo);
                        Serial.printf("[SNTP] Time synchronized successfully: %s MMT\n", tbuf);
                    }
                }
            }
        } else if (state == WifiState::DISCONNECTED) {
            if (now - lastReconnectAttempt > reconnectDelay) {
                lastReconnectAttempt = now;
                reconnectDelay = (reconnectDelay * 2 <= maxReconnectDelay) ? (reconnectDelay * 2) : maxReconnectDelay;

                String ssid = ConfigManager::getInstance().getWifiSSID();
                String pass = ConfigManager::getInstance().getWifiPassword();
                if (ssid.length() > 0) {
                    Serial.printf("[WiFi] Reconnecting to %s (backoff: %lu ms)...\n", ssid.c_str(), (unsigned long)reconnectDelay);
                    WiFi.disconnect(true);
                    delay(50);
                    WiFi.begin(ssid.c_str(), pass.c_str());
                    state = WifiState::CONNECTING;
                } else {
                    startProvisioningPortal();
                }
            }
        }
    }
};

// SECTION 10: TOUCHSCREEN UI ENGINE & MULTI-SCREEN NAVIGATION
// ============================================================================
class UIManager {
private:
    ActiveScreen currentScreen = ActiveScreen::SCREEN_FACE;
    bool forceRedraw = true;
    unsigned long lastStatusBarDraw = 0;
    String lastRfidScanned = "";

    static constexpr unsigned long RFID_CACHE_TTL_MS = 5 * 60 * 1000;
    RfidDisplayResult rfidCache[8]{};
    size_t nextRfidCacheSlot = 0;

    enum class RfidLookupState { Pending, Ready, Unavailable };
    struct RfidCardView {
        bool active = false;
        String uid = "";
        bool found = false;
        String itemKey = "";
        String nameEn = "";
        RfidLookupState lookup = RfidLookupState::Ready;
        unsigned long scannedAt = 0;
        bool logDraw = false;
        unsigned long displayStartTime = 0;
    } rfidCardView;

public:
    static UIManager& getInstance() {
        static UIManager instance;
        return instance;
    }

    void setScreen(ActiveScreen scr) {
        currentScreen = scr;
        forceRedraw = true;
    }

    ActiveScreen getScreen() const { return currentScreen; }

    void setLastRfid(const String& uid) {
        lastRfidScanned = uid;
    }

    void showRfidScan(const char* uid, unsigned long scannedAt, bool lookupQueued) {
        rfidCardView.active = true;
        rfidCardView.uid = uid;
        rfidCardView.lookup = lookupQueued ? RfidLookupState::Pending : RfidLookupState::Unavailable;
        rfidCardView.scannedAt = scannedAt;
        rfidCardView.displayStartTime = millis();
        rfidCardView.logDraw = false;
        lastRfidScanned = uid;
        for (const auto& cached : rfidCache) {
            if (cached.found && rfidCardView.uid == cached.uid &&
                millis() - cached.receivedAt < RFID_CACHE_TTL_MS) {
                applyRfidResult(cached, rfidCardView.displayStartTime);
                Serial.printf("[RFID] Cache hit uid=%s\n", uid);
                break;
            }
        }
        setScreen(ActiveScreen::SCREEN_RFID);
    }

    void applyRfidResult(const RfidDisplayResult& result, unsigned long now) {
        rfidCardView.lookup = RfidLookupState::Ready;
        rfidCardView.found = result.found;
        rfidCardView.itemKey = result.itemKey;
        rfidCardView.nameEn = result.nameEn;
        rfidCardView.displayStartTime = now;
        rfidCardView.logDraw = true;
        forceRedraw = true;
    }

    void rememberRfidResult(const RfidDisplayResult& result) {
        for (auto& cached : rfidCache) {
            if (strcmp(cached.uid, result.uid) == 0) {
                cached = result;
                return;
            }
        }
        if (result.found) {
            rfidCache[nextRfidCacheSlot] = result;
            nextRfidCacheSlot = (nextRfidCacheSlot + 1) % (sizeof(rfidCache) / sizeof(rfidCache[0]));
        }
    }

    void drawVectorGraphic(const String& item, int cx, int cy) {
        if (item == "apple") {
            // Apple body: two overlapping red circles + rounded core
            tft.fillCircle(cx - 10, cy + 2, 20, tft.color565(230, 40, 50));
            tft.fillCircle(cx + 10, cy + 2, 20, tft.color565(230, 40, 50));
            tft.fillRoundRect(cx - 18, cy - 12, 36, 30, 10, tft.color565(230, 40, 50));
            tft.fillCircle(cx, cy - 14, 5, tft.color565(25, 38, 65));
            tft.fillCircle(cx, cy + 22, 4, tft.color565(25, 38, 65));
            // Stem
            tft.fillRoundRect(cx - 2, cy - 28, 4, 15, 2, tft.color565(120, 70, 30));
            // Green leaf
            tft.fillCircle(cx + 9, cy - 22, 6, tft.color565(60, 190, 70));
            tft.fillTriangle(cx + 4, cy - 22, cx + 16, cy - 26, cx + 14, cy - 18, tft.color565(60, 190, 70));
            // Shine highlight
            tft.drawCircle(cx - 10, cy - 2, 12, TFT_WHITE);
        } else if (item == "elephant") {
            uint16_t cGrey = tft.color565(140, 175, 210);
            uint16_t cPink = tft.color565(255, 185, 205);
            // Big ears
            tft.fillCircle(cx - 22, cy - 4, 16, cGrey);
            tft.fillCircle(cx - 22, cy - 4, 10, cPink);
            tft.fillCircle(cx + 22, cy - 4, 16, cGrey);
            tft.fillCircle(cx + 22, cy - 4, 10, cPink);
            // Head
            tft.fillCircle(cx, cy, 22, cGrey);
            // Trunk
            tft.fillRoundRect(cx - 6, cy + 6, 12, 20, 6, cGrey);
            tft.fillCircle(cx + 5, cy + 24, 6, cGrey);
            // Tusks
            tft.fillTriangle(cx - 8, cy + 12, cx - 14, cy + 20, cx - 6, cy + 16, TFT_WHITE);
            tft.fillTriangle(cx + 8, cy + 12, cx + 14, cy + 20, cx + 6, cy + 16, TFT_WHITE);
            // Eyes
            tft.fillCircle(cx - 9, cy - 4, 3, TFT_BLACK);
            tft.fillCircle(cx - 8, cy - 5, 1, TFT_WHITE);
            tft.fillCircle(cx + 9, cy - 4, 3, TFT_BLACK);
            tft.fillCircle(cx + 10, cy - 5, 1, TFT_WHITE);
        } else if (item == "cat") {
            uint16_t cCat = tft.color565(255, 165, 50);
            uint16_t cEarPink = tft.color565(255, 185, 195);
            // Ears
            tft.fillTriangle(cx - 22, cy - 10, cx - 16, cy - 32, cx - 6, cy - 16, cCat);
            tft.fillTriangle(cx - 20, cy - 12, cx - 16, cy - 27, cx - 8, cy - 16, cEarPink);
            tft.fillTriangle(cx + 22, cy - 10, cx + 16, cy - 32, cx + 6, cy - 16, cCat);
            tft.fillTriangle(cx + 20, cy - 12, cx + 16, cy - 27, cx + 8, cy - 16, cEarPink);
            // Head
            tft.fillCircle(cx, cy, 22, cCat);
            // Eyes
            tft.fillCircle(cx - 9, cy - 3, 4, TFT_BLACK);
            tft.fillCircle(cx - 8, cy - 4, 1, TFT_WHITE);
            tft.fillCircle(cx + 9, cy - 3, 4, TFT_BLACK);
            tft.fillCircle(cx + 10, cy - 4, 1, TFT_WHITE);
            // Nose & whiskers
            tft.fillTriangle(cx - 3, cy + 4, cx + 3, cy + 4, cx, cy + 7, cEarPink);
            tft.drawFastHLine(cx - 24, cy + 4, 12, TFT_BLACK);
            tft.drawFastHLine(cx - 24, cy + 9, 12, TFT_BLACK);
            tft.drawFastHLine(cx + 12, cy + 4, 12, TFT_BLACK);
            tft.drawFastHLine(cx + 12, cy + 9, 12, TFT_BLACK);
        } else if (item == "dog") {
            uint16_t cDog = tft.color565(225, 160, 90);
            uint16_t cEar = tft.color565(160, 95, 45);
            uint16_t cMuzzle = tft.color565(250, 240, 225);
            // Floppy ears
            tft.fillRoundRect(cx - 26, cy - 16, 12, 30, 6, cEar);
            tft.fillRoundRect(cx + 14, cy - 16, 12, 30, 6, cEar);
            // Head
            tft.fillCircle(cx, cy, 22, cDog);
            // Muzzle
            tft.fillCircle(cx, cy + 8, 12, cMuzzle);
            // Nose & tongue
            tft.fillRoundRect(cx - 5, cy + 4, 10, 6, 3, TFT_BLACK);
            tft.fillRoundRect(cx - 3, cy + 14, 6, 8, 3, tft.color565(255, 120, 140));
            // Eyes
            tft.fillCircle(cx - 9, cy - 4, 4, TFT_BLACK);
            tft.fillCircle(cx - 8, cy - 5, 1, TFT_WHITE);
            tft.fillCircle(cx + 9, cy - 4, 4, TFT_BLACK);
            tft.fillCircle(cx + 10, cy - 5, 1, TFT_WHITE);
        } else if (item == "banana") {
            uint16_t cYellow = tft.color565(255, 215, 0);     // Vibrant banana yellow
            uint16_t cDarkYellow = tft.color565(210, 160, 20); // Banana shading / peel ridge
            uint16_t cStem = tft.color565(100, 130, 40);      // Green-brown stem
            uint16_t cTip = tft.color565(80, 50, 20);         // Dark tip
            
            // Realistic curved banana peel body (tilted smoothly from top-left to right)
            tft.fillCircle(cx - 16, cy - 8, 7, cYellow);
            tft.fillCircle(cx - 10, cy - 2, 9, cYellow);
            tft.fillCircle(cx - 3, cy + 5, 11, cYellow);
            tft.fillCircle(cx + 5, cy + 8, 12, cYellow);
            tft.fillCircle(cx + 14, cy + 6, 11, cYellow);
            tft.fillCircle(cx + 21, cy, 9, cYellow);
            tft.fillCircle(cx + 26, cy - 7, 7, cYellow);

            // Smooth inner curve cutout using card background
            tft.fillCircle(cx + 5, cy - 14, 18, tft.color565(25, 38, 65));
            tft.fillCircle(cx + 4, cy + 8, 11, cYellow); // restore belly roundness

            // Stem at top-left tip
            tft.fillRoundRect(cx - 24, cy - 15, 9, 6, 2, cStem);
            // Tip at upper-right
            tft.fillCircle(cx + 27, cy - 8, 3, cTip);

            // Shading & peel ridge lines along the curve
            tft.drawLine(cx - 12, cy + 4, cx + 4, cy + 14, cDarkYellow);
            tft.drawLine(cx + 4, cy + 14, cx + 22, cy + 5, cDarkYellow);
            tft.drawLine(cx - 8, cy - 1, cx + 5, cy + 6, cDarkYellow);
        } else if (item == "snake") {
            uint16_t cSnakeGreen = tft.color565(40, 180, 80);
            uint16_t cSnakeLight = tft.color565(110, 225, 130);
            uint16_t cTongue = tft.color565(240, 50, 70);
            // Coiled body S-curve
            tft.fillRoundRect(cx - 18, cy + 4, 38, 14, 7, cSnakeGreen);
            tft.fillCircle(cx - 10, cy + 8, 10, cSnakeGreen);
            tft.fillCircle(cx + 10, cy + 4, 11, cSnakeGreen);
            tft.fillRoundRect(cx - 22, cy - 8, 22, 14, 7, cSnakeGreen);
            // Head on upper right
            tft.fillCircle(cx + 12, cy - 10, 13, cSnakeGreen);
            tft.fillCircle(cx + 14, cy - 10, 8, cSnakeLight);
            // Forked red tongue
            tft.drawFastHLine(cx + 24, cy - 8, 8, cTongue);
            tft.drawPixel(cx + 31, cy - 10, cTongue);
            tft.drawPixel(cx + 31, cy - 6, cTongue);
            // Cute eyes
            tft.fillCircle(cx + 10, cy - 14, 3, TFT_BLACK);
            tft.fillCircle(cx + 11, cy - 15, 1, TFT_WHITE);
            tft.fillCircle(cx + 18, cy - 14, 3, TFT_BLACK);
            tft.fillCircle(cx + 19, cy - 15, 1, TFT_WHITE);
            // Belly pattern dots
            tft.fillCircle(cx - 8, cy + 8, 3, cSnakeLight);
            tft.fillCircle(cx + 6, cy + 6, 3, cSnakeLight);
        } else if (item == "worm") {
            uint16_t cWormPink = tft.color565(255, 135, 155);
            uint16_t cWormDark = tft.color565(225, 95, 120);
            // Wavy segmented body (series of overlapping soft round segments)
            tft.fillCircle(cx - 22, cy + 12, 6, cWormPink); // tail
            tft.fillCircle(cx - 14, cy + 4, 7, cWormPink);
            tft.fillCircle(cx - 6, cy + 10, 8, cWormPink);
            tft.fillCircle(cx + 4, cy + 2, 9, cWormPink);
            tft.fillCircle(cx + 12, cy - 8, 10, cWormPink);
            // Head
            tft.fillCircle(cx + 16, cy - 15, 12, cWormPink);
            // Segment ridges
            tft.drawCircle(cx - 6, cy + 10, 8, cWormDark);
            tft.drawCircle(cx + 4, cy + 2, 9, cWormDark);
            // Rosy cheeks
            tft.fillCircle(cx + 10, cy - 10, 3, tft.color565(255, 175, 190));
            tft.fillCircle(cx + 22, cy - 10, 3, tft.color565(255, 175, 190));
            // Cute eyes
            tft.fillCircle(cx + 12, cy - 17, 2, TFT_BLACK);
            tft.fillCircle(cx + 20, cy - 17, 2, TFT_BLACK);
            // Smile
            tft.drawPixel(cx + 15, cy - 12, cWormDark);
            tft.drawPixel(cx + 16, cy - 11, cWormDark);
            tft.drawPixel(cx + 17, cy - 12, cWormDark);
        } else {
            // Mystery card / Unknown
            tft.fillRoundRect(cx - 24, cy - 24, 48, 48, 8, tft.color565(60, 85, 125));
            tft.drawRoundRect(cx - 24, cy - 24, 48, 48, 8, tft.color565(255, 215, 64));
            tft.setTextDatum(textdatum_t::middle_center);
            tft.setTextSize(3);
            tft.setTextColor(tft.color565(255, 215, 64), tft.color565(60, 85, 125));
            tft.drawString("?", cx, cy);
        }
    }

    void drawMyanmarBitmap(const String& item, int x, int y, uint16_t color, uint16_t bg) {
        if (item == "apple") {
            tft.drawBitmap(x, y, MM_BITMAP_apple, MM_BITMAP_apple_W, MM_BITMAP_apple_H, color, bg);
        } else if (item == "elephant") {
            tft.drawBitmap(x, y, MM_BITMAP_elephant, MM_BITMAP_elephant_W, MM_BITMAP_elephant_H, color, bg);
        } else if (item == "cat") {
            tft.drawBitmap(x, y, MM_BITMAP_cat, MM_BITMAP_cat_W, MM_BITMAP_cat_H, color, bg);
        } else if (item == "dog") {
            tft.drawBitmap(x, y, MM_BITMAP_dog, MM_BITMAP_dog_W, MM_BITMAP_dog_H, color, bg);
        } else if (item == "banana") {
            tft.drawBitmap(x, y, MM_BITMAP_banana, MM_BITMAP_banana_W, MM_BITMAP_banana_H, color, bg);
        } else if (item == "snake") {
            tft.drawBitmap(x, y, MM_BITMAP_snake, MM_BITMAP_snake_W, MM_BITMAP_snake_H, color, bg);
        } else if (item == "worm") {
            tft.drawBitmap(x, y, MM_BITMAP_worm, MM_BITMAP_worm_W, MM_BITMAP_worm_H, color, bg);
        }
    }

    void drawStatusBar(bool wifiOk, int rssi, bool wsOk, uint8_t vol) {
        tft.fillRect(0, 0, 320, 22, tft.color565(8, 20, 42));
        tft.drawFastHLine(0, 22, 320, tft.color565(25, 55, 95));

        // Wi-Fi Status
        tft.setTextSize(1);
        tft.setCursor(6, 6);
        if (wifiOk) {
            tft.setTextColor(tft.color565(0, 230, 118), tft.color565(8, 20, 42));
            tft.printf("WiFi:%d dBm", rssi);
        } else {
            tft.setTextColor(tft.color565(255, 82, 82), tft.color565(8, 20, 42));
            tft.print("WiFi:OFF");
        }

        // Cloud WebSocket
        tft.fillCircle(145, 11, 4, wsOk ? tft.color565(0, 230, 118) : tft.color565(255, 171, 0));
        tft.setTextColor(tft.color565(210, 235, 255), tft.color565(8, 20, 42));
        tft.setCursor(155, 6);
        tft.print(wsOk ? "LIVE" : "OFF");

        // Volume & Menu Touch Hotspot
        tft.setTextColor(tft.color565(220, 235, 250), tft.color565(8, 20, 42));
        tft.setCursor(240, 6);
        tft.printf("V:%u%% [MENU]", vol);
    }

    void renderMouth(RobotState state, unsigned long now) {
        const uint16_t COLOR_BG = tft.color565(14, 40, 78);
        tft.fillRect(110, 165, 100, 36, COLOR_BG);

        const auto prompt = wakePrompt.load(std::memory_order_relaxed);
        if (state == RobotState::IDLE && prompt != WakePrompt::None) {
            tft.setTextSize(1);
            tft.setTextDatum(textdatum_t::middle_center);
            tft.setTextColor(TFT_WHITE, COLOR_BG);
            tft.drawString(prompt == WakePrompt::Speak ? "Speak now" : "Server offline", 160, 183);
        } else if (state == RobotState::SPEAKING) {
            int h = 8 + (int)(abs(sin(now / 90.0f)) * 24);
            int w = 48 + (int)(abs(cos(now / 140.0f)) * 16);
            tft.fillRoundRect(160 - (w / 2), 183 - (h / 2), w, h, 6, tft.color565(0, 229, 255));
        } else if (state == RobotState::LISTENING) {
            int pulse = 32 + (int)(sin(now / 100.0f) * 16);
            tft.fillRoundRect(160 - (pulse / 2), 180, pulse, 8, 4, tft.color565(0, 230, 118));
        } else if (state == RobotState::THINKING) {
            for (int i = 0; i < 3; i++) {
                int bounce = (int)(abs(sin((now + i * 140) / 160.0f)) * 8);
                tft.fillCircle(140 + i * 20, 183 - bounce, 5, tft.color565(255, 215, 64));
            }
        }
    }

    void drawButton(int x, int y, int w, int h, const char* label, uint16_t bgColor, uint16_t txtColor = TFT_WHITE) {
        tft.fillRoundRect(x, y, w, h, 6, bgColor);
        tft.drawRoundRect(x, y, w, h, 6, tft.color565(200, 220, 255));
        tft.setTextColor(txtColor, bgColor);
        tft.setTextDatum(textdatum_t::middle_center);
        tft.drawString(label, x + (w / 2), y + (h / 2));
    }

    void renderScreenMenu() {
        tft.fillRect(0, 24, 320, 216, tft.color565(12, 24, 48));
        tft.setTextColor(TFT_WHITE, tft.color565(12, 24, 48));
        tft.setTextDatum(textdatum_t::top_center);
        tft.drawString("--- MEMO ROBOT MENU ---", 160, 30);

        drawButton(15, 55, 90, 38, "EMOTIONS", tft.color565(30, 80, 160));
        drawButton(115, 55, 90, 38, "VOICE", tft.color565(0, 150, 100));
        drawButton(215, 55, 90, 38, "VOLUME", tft.color565(160, 100, 30));

        drawButton(15, 105, 90, 38, "WI-FI", tft.color565(40, 100, 120));
        drawButton(115, 105, 90, 38, "DIAGNOST", tft.color565(120, 60, 140));
        drawButton(215, 105, 90, 38, "RFID", tft.color565(150, 40, 70));

        drawButton(15, 155, 140, 38, "DEVICE INFO", tft.color565(50, 70, 90));
        drawButton(165, 155, 140, 38, "<- BACK (FACE)", tft.color565(30, 40, 60));

        tft.setTextDatum(textdatum_t::bottom_center);
        tft.setTextColor(tft.color565(160, 190, 220), tft.color565(12, 24, 48));
        tft.drawString("Tap any option to navigate", 160, 230);
    }

    void renderScreenEmotions() {
        tft.fillRect(0, 24, 320, 216, tft.color565(15, 30, 60));
        tft.setTextColor(TFT_WHITE, tft.color565(15, 30, 60));
        tft.setTextDatum(textdatum_t::top_center);
        tft.drawString("SELECT ROBOT EMOTION", 160, 30);

        drawButton(15, 55, 68, 34, "HAPPY", tft.color565(0, 180, 80));
        drawButton(90, 55, 68, 34, "ANGRY", tft.color565(220, 40, 40));
        drawButton(165, 55, 68, 34, "SAD", tft.color565(60, 100, 200));
        drawButton(240, 55, 68, 34, "EXCITED", tft.color565(230, 140, 0));

        drawButton(15, 100, 68, 34, "NEUTRAL", tft.color565(80, 100, 130));
        drawButton(90, 100, 68, 34, "WINK", tft.color565(180, 60, 180));
        drawButton(165, 100, 68, 34, "ANNOYED", tft.color565(150, 80, 40));
        drawButton(240, 100, 68, 34, "QUESTION", tft.color565(40, 140, 160));

        drawButton(80, 175, 160, 38, "<- BACK TO MENU", tft.color565(40, 50, 70));
    }

    void renderScreenVoice(RobotState state) {
        tft.fillRect(0, 24, 320, 216, tft.color565(10, 25, 50));
        tft.setTextColor(TFT_WHITE, tft.color565(10, 25, 50));
        tft.setTextDatum(textdatum_t::top_center);
        tft.drawString("GEMINI LIVE ASSISTANT", 160, 32);

        tft.setTextDatum(textdatum_t::middle_center);
        tft.drawString("Robot State:", 160, 70);

        uint16_t stateColor = tft.color565(0, 230, 118);
        if (state == RobotState::LISTENING) stateColor = tft.color565(0, 255, 180);
        else if (state == RobotState::THINKING) stateColor = tft.color565(255, 215, 64);
        else if (state == RobotState::SPEAKING) stateColor = tft.color565(0, 229, 255);

        tft.setTextColor(stateColor, tft.color565(10, 25, 50));
        tft.setTextSize(2);
        tft.drawString(robotStateToString(state), 160, 95);
        tft.setTextSize(1);

        if (state == RobotState::IDLE) {
            drawButton(60, 125, 200, 42, "TAP TO TALK (START)", tft.color565(0, 160, 90));
        } else {
            drawButton(60, 125, 200, 42, "STOP CONVERSATION", tft.color565(200, 50, 50));
        }

        drawButton(80, 185, 160, 34, "<- BACK TO MENU", tft.color565(40, 50, 70));
    }

    void renderScreenVolume(uint8_t vol, uint8_t bright) {
        tft.fillRect(0, 24, 320, 216, tft.color565(16, 28, 48));
        tft.setTextColor(TFT_WHITE, tft.color565(16, 28, 48));
        tft.setTextDatum(textdatum_t::top_center);
        tft.drawString("VOLUME & DISPLAY SETTINGS", 160, 30);

        tft.drawString("Speaker Volume:", 80, 60);
        drawButton(180, 48, 50, 32, "VOL -", tft.color565(180, 70, 30));
        drawButton(245, 48, 50, 32, "VOL +", tft.color565(30, 140, 80));

        tft.drawString("Screen Brightness:", 80, 100);
        drawButton(180, 88, 50, 32, "BRT -", tft.color565(180, 70, 30));
        drawButton(245, 88, 50, 32, "BRT +", tft.color565(30, 140, 80));

        // Touch Calibration Button
        drawButton(30, 136, 120, 34, "CALIBRATE", tft.color565(100, 70, 160));
        drawButton(170, 136, 120, 34, "CUTE CHIME", tft.color565(0, 140, 140));

        drawButton(80, 185, 160, 34, "<- BACK TO MENU", tft.color565(40, 50, 70));
    }

    void renderScreenWifi(bool wifiOk, const String& ip, int rssi, const String& ssid) {
        tft.fillRect(0, 24, 320, 216, tft.color565(12, 26, 44));
        tft.setTextColor(TFT_WHITE, tft.color565(12, 26, 44));
        tft.setTextDatum(textdatum_t::top_center);
        tft.drawString("WI-FI NETWORK STATUS", 160, 30);

        bool portalOn = WiFiManagerDriver::getInstance().isPortalActive();

        tft.setTextDatum(textdatum_t::top_left);
        if (portalOn) {
            tft.setTextColor(tft.color565(0, 255, 200), tft.color565(12, 26, 44));
            tft.setCursor(15, 60);
            tft.print("WEB PORTAL ACTIVE!");
            tft.setTextColor(TFT_WHITE, tft.color565(12, 26, 44));
            tft.setCursor(15, 80);
            tft.print("1. Connect Phone Wi-Fi to:");
            tft.setCursor(25, 100);
            tft.setTextColor(tft.color565(255, 215, 64), tft.color565(12, 26, 44));
            tft.print("ESP32-Robot-Setup");
            tft.setTextColor(TFT_WHITE, tft.color565(12, 26, 44));
            tft.setCursor(15, 120);
            tft.print("2. Open Browser: http://192.168.4.1");
        } else {
            tft.setCursor(20, 60);
            tft.printf("SSID: %s\n", ssid.c_str());
            tft.setCursor(20, 80);
            tft.printf("Status: %s\n", wifiOk ? "CONNECTED" : "DISCONNECTED");
            tft.setCursor(20, 100);
            tft.printf("IP Address: %s\n", ip.c_str());
            tft.setCursor(20, 120);
            tft.printf("Signal RSSI: %d dBm\n", rssi);
        }

        if (portalOn) {
            drawButton(20, 160, 130, 38, "STOP PORTAL", tft.color565(200, 50, 50));
        } else {
            drawButton(20, 160, 130, 38, "WEB PORTAL", tft.color565(0, 150, 120));
        }
        drawButton(170, 160, 130, 38, "<- BACK", tft.color565(40, 50, 70));
    }

    void renderScreenInfo() {
        tft.fillRect(0, 24, 320, 216, tft.color565(10, 20, 38));
        tft.setTextColor(TFT_WHITE, tft.color565(10, 20, 38));
        tft.setTextDatum(textdatum_t::top_center);
        tft.drawString("MEMO SYSTEM INFORMATION", 160, 30);

        tft.setTextDatum(textdatum_t::top_left);
        tft.setCursor(20, 55);
        tft.printf("Model: %s\n", HARDWARE_VERSION);
        tft.setCursor(20, 75);
        tft.printf("Firmware: %s v%s\n", FIRMWARE_NAME, FIRMWARE_VERSION);
        tft.setCursor(20, 95);
        tft.printf("Device ID: %s\n", ConfigManager::getInstance().getDeviceId().c_str());
        tft.setCursor(20, 115);
        tft.printf("SRAM Free: %lu KB | PSRAM: %lu KB\n", (unsigned long)(ESP.getFreeHeap() / 1024),
                   (unsigned long)(ESP.getFreePsram() / 1024));
        tft.setCursor(20, 135);
        tft.printf("Uptime: %lu seconds\n", (unsigned long)(millis() / 1000));

        drawButton(80, 175, 160, 38, "<- BACK TO MENU", tft.color565(40, 50, 70));
    }

    void renderScreenDiagnostics() {
        tft.fillRect(0, 24, 320, 216, tft.color565(8, 18, 34));
        tft.setTextColor(TFT_WHITE, tft.color565(8, 18, 34));
        tft.setTextDatum(textdatum_t::top_center);
        tft.drawString("HARDWARE SUBSYSTEM AUDIT", 160, 30);

        tft.setTextDatum(textdatum_t::top_left);
        tft.setCursor(15, 55);
        tft.print("- Flash / PSRAM: 16MB / 8MB OPI OK");
        tft.setCursor(15, 75);
        tft.print("- ILI9341 SPI Display: 10MHz ACTIVE");
        tft.setCursor(15, 95);
        tft.print("- XPT2046 Touch: SPI CS:8 IRQ:18 READY");
        tft.setCursor(15, 115);
        tft.print("- INMP441 Microphone: I2S0 16kHz OK");
        tft.setCursor(15, 135);
        tft.print("- MAX98357A Speaker: I2S1 24kHz OK");
        tft.setCursor(15, 155);
        tft.print("- Head Touch (GPIO 38): ACTIVE_HIGH");

        drawButton(80, 185, 160, 34, "<- BACK TO MENU", tft.color565(40, 50, 70));
    }

    void renderScreenRfid() {
        if (rfidCardView.active) {
            // Child-Friendly RFID Learning Card Display (Feature E)
            const uint16_t cCardBg = tft.color565(25, 38, 65);
            tft.fillRect(0, 24, 320, 216, tft.color565(14, 22, 38));
            tft.fillRoundRect(10, 28, 300, 206, 12, cCardBg);
            tft.drawRoundRect(10, 28, 300, 206, 12, tft.color565(70, 120, 190));

            if (rfidCardView.lookup != RfidLookupState::Ready) {
                tft.setTextDatum(textdatum_t::top_center);
                tft.setTextSize(2);
                tft.setTextColor(TFT_WHITE, cCardBg);
                tft.drawString(rfidCardView.lookup == RfidLookupState::Pending ?
                               "Reading card..." : "Card lookup unavailable", 160, 78);
                tft.setTextSize(1);
                tft.drawString(rfidCardView.uid.c_str(), 160, 119);
                return;
            }

            if (rfidCardView.found) {
                // Header Banner
                tft.setTextDatum(textdatum_t::top_center);
                tft.setTextSize(1);
                tft.setTextColor(tft.color565(0, 229, 255), cCardBg);
                tft.drawString("MEMO RFID LEARNING CARD", 160, 36);

                // Cute Vector Graphic (Center: x=75, y=125)
                drawVectorGraphic(rfidCardView.itemKey, 75, 125);

                // English Title
                tft.setTextDatum(textdatum_t::top_left);
                tft.setTextSize(2);
                tft.setTextColor(TFT_WHITE, cCardBg);
                tft.drawString(rfidCardView.nameEn.c_str(), 145, 62);

                // Crisp Myanmar Bitmap Title
                drawMyanmarBitmap(rfidCardView.itemKey, 145, 102, tft.color565(255, 215, 64), cCardBg);

                // Scanned UID Badge
                tft.fillRoundRect(145, 142, 155, 24, 4, tft.color565(16, 26, 45));
                tft.drawRoundRect(145, 142, 155, 24, 4, tft.color565(55, 85, 130));
                tft.setTextSize(1);
                tft.setTextColor(tft.color565(160, 195, 235), tft.color565(16, 26, 45));
                tft.drawString(("UID: " + rfidCardView.uid).c_str(), 152, 148);

                // Footer auto-close hint
                tft.setCursor(145, 182);
                tft.setTextColor(tft.color565(110, 140, 175), cCardBg);
                tft.print("Tap to close (6s)");
            } else {
                // Unregistered Card View
                tft.setTextDatum(textdatum_t::top_center);
                tft.setTextSize(1);
                tft.setTextColor(tft.color565(255, 171, 0), cCardBg);
                tft.drawString("NEW RFID CARD DETECTED", 160, 36);

                // Unknown Card Graphic
                drawVectorGraphic("unknown", 75, 125);

                // Registration details
                tft.setTextDatum(textdatum_t::top_left);
                tft.setTextSize(2);
                tft.setTextColor(tft.color565(255, 180, 50), cCardBg);
                tft.drawString("UNREGISTERED", 145, 62);

                tft.setTextSize(1);
                tft.setTextColor(TFT_WHITE, cCardBg);
                tft.drawString("Register in MongoDB:", 145, 102);

                tft.fillRoundRect(145, 125, 155, 24, 4, tft.color565(16, 26, 45));
                tft.drawRoundRect(145, 125, 155, 24, 4, tft.color565(180, 120, 40));
                tft.setTextColor(tft.color565(255, 220, 100), tft.color565(16, 26, 45));
                tft.drawString(("UID: " + rfidCardView.uid).c_str(), 152, 131);

                tft.setCursor(145, 160);
                tft.setTextColor(tft.color565(160, 180, 200), cCardBg);
                tft.print("Collection: rfid_cards");

                tft.setCursor(145, 185);
                tft.setTextColor(tft.color565(110, 140, 175), cCardBg);
                tft.print("Tap to return");
            }
        } else {
            // Diagnostic / Menu RFID Screen
            tft.fillRect(0, 24, 320, 216, tft.color565(18, 22, 42));
            tft.setTextColor(TFT_WHITE, tft.color565(18, 22, 42));
            tft.setTextDatum(textdatum_t::top_center);
            tft.drawString("PN532 RFID READER STATUS", 160, 30);

            tft.setTextDatum(textdatum_t::top_left);
            tft.setCursor(20, 60);
            tft.print("Interface: UART1 (GPIO 1/2) 115200");
            tft.setCursor(20, 85);
            tft.printf("Last Card UID: %s\n", lastRfidScanned.length() > 0 ? lastRfidScanned.c_str() : "[NO CARD DETECTED]");
            tft.setCursor(20, 110);
            tft.print("Mode: MIFARE Classic Educational");

            drawButton(80, 175, 160, 38, "<- BACK TO MENU", tft.color565(40, 50, 70));
        }
    }

    void runTouchCalibration() {
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setTextDatum(textdatum_t::top_center);
        tft.drawString("TOUCHSCREEN CALIBRATION", 160, 20);
        tft.drawString("Touch the crosshairs precisely", 160, 45);

        TouchCalibrationData cal;
        uint16_t colorFg = (uint16_t)TFT_WHITE;
        uint16_t colorBg = tft.color565(12, 24, 48);
        tft.calibrateTouch(cal.data, colorFg, colorBg, 10);
        cal.valid = true;
        ConfigManager::getInstance().saveTouchCal(cal);
        tft.setTouchCalibrate(cal.data);

        tft.fillScreen(tft.color565(0, 160, 80));
        tft.drawString("Calibration Complete!", 160, 110);
        delay(1000);
        forceRedraw = true;
    }

    void handleTouchClick(int x, int y, RobotState state) {
        if (y < 24) {
            setScreen(currentScreen == ActiveScreen::SCREEN_FACE ? ActiveScreen::SCREEN_MENU : ActiveScreen::SCREEN_FACE);
            return;
        }

        switch (currentScreen) {
            case ActiveScreen::SCREEN_FACE:
                setScreen(ActiveScreen::SCREEN_MENU);
                break;

            case ActiveScreen::SCREEN_MENU:
                if (x >= 15 && x <= 105 && y >= 55 && y <= 93) setScreen(ActiveScreen::SCREEN_EMOTIONS);
                else if (x >= 115 && x <= 205 && y >= 55 && y <= 93) setScreen(ActiveScreen::SCREEN_VOICE);
                else if (x >= 215 && x <= 305 && y >= 55 && y <= 93) setScreen(ActiveScreen::SCREEN_VOLUME);
                else if (x >= 15 && x <= 105 && y >= 105 && y <= 143) setScreen(ActiveScreen::SCREEN_WIFI);
                else if (x >= 115 && x <= 205 && y >= 105 && y <= 143) setScreen(ActiveScreen::SCREEN_DIAGNOSTICS);
                else if (x >= 215 && x <= 305 && y >= 105 && y <= 143) setScreen(ActiveScreen::SCREEN_RFID);
                else if (x >= 15 && x <= 155 && y >= 155 && y <= 193) setScreen(ActiveScreen::SCREEN_INFO);
                else if (x >= 165 && x <= 305 && y >= 155 && y <= 193) setScreen(ActiveScreen::SCREEN_FACE);
                break;

            case ActiveScreen::SCREEN_EMOTIONS:
                if (x >= 15 && x <= 83 && y >= 55 && y <= 89) eyes.setEmotion(RobotEmotion::HAPPY);
                else if (x >= 90 && x <= 158 && y >= 55 && y <= 89) eyes.setEmotion(RobotEmotion::ANGRY);
                else if (x >= 165 && x <= 233 && y >= 55 && y <= 89) eyes.setEmotion(RobotEmotion::SAD);
                else if (x >= 240 && x <= 308 && y >= 55 && y <= 89) eyes.setEmotion(RobotEmotion::EXCITED);
                else if (x >= 15 && x <= 83 && y >= 100 && y <= 134) eyes.setEmotion(RobotEmotion::NEUTRAL);
                else if (x >= 90 && x <= 158 && y >= 100 && y <= 134) eyes.setEmotion(RobotEmotion::WINK);
                else if (x >= 165 && x <= 233 && y >= 100 && y <= 134) eyes.setEmotion(RobotEmotion::ANNOYED);
                else if (x >= 240 && x <= 308 && y >= 100 && y <= 134) eyes.setEmotion(RobotEmotion::QUESTIONING);
                else if (x >= 80 && x <= 240 && y >= 175 && y <= 213) setScreen(ActiveScreen::SCREEN_MENU);
                break;

            case ActiveScreen::SCREEN_VOICE:
                if (x >= 60 && x <= 260 && y >= 125 && y <= 167) {
                    if (state == RobotState::IDLE) postEvent(EventType::VOICE_SESSION_START);
                    else postEvent(EventType::VOICE_SESSION_END);
                } else if (x >= 80 && x <= 240 && y >= 185 && y <= 219) {
                    setScreen(ActiveScreen::SCREEN_MENU);
                }
                break;

            case ActiveScreen::SCREEN_VOLUME: {
                uint8_t curV = ConfigManager::getInstance().getVolume();
                uint8_t curB = ConfigManager::getInstance().getBrightness();
                if (x >= 180 && x <= 230 && y >= 48 && y <= 80) {
                    uint8_t nv = (curV >= 5) ? (curV - 5) : 0;
                    ConfigManager::getInstance().setVolume(nv);
                    postEvent(EventType::CMD_SET_VOLUME, nv);
                    forceRedraw = true;
                } else if (x >= 245 && x <= 295 && y >= 48 && y <= 80) {
                    uint8_t nv = (curV + 5 <= 100) ? (curV + 5) : 100;
                    ConfigManager::getInstance().setVolume(nv);
                    postEvent(EventType::CMD_SET_VOLUME, nv);
                    forceRedraw = true;
                } else if (x >= 180 && x <= 230 && y >= 88 && y <= 120) {
                    uint8_t nb = (curB >= 10) ? (curB - 10) : 10;
                    ConfigManager::getInstance().setBrightness(nb);
                    tft.setBrightness((nb * 255) / 100);
                    forceRedraw = true;
                } else if (x >= 245 && x <= 295 && y >= 88 && y <= 120) {
                    uint8_t nb = (curB + 10 <= 100) ? (curB + 10) : 100;
                    ConfigManager::getInstance().setBrightness(nb);
                    tft.setBrightness((nb * 255) / 100);
                    forceRedraw = true;
                } else if (x >= 30 && x <= 150 && y >= 136 && y <= 170) {
                    runTouchCalibration();
                } else if (x >= 170 && x <= 290 && y >= 136 && y <= 170) {
                    AudioOutputDriver::getInstance().triggerCuteChime();
                } else if (x >= 80 && x <= 240 && y >= 185 && y <= 219) {
                    setScreen(ActiveScreen::SCREEN_MENU);
                }
                break;
            }

            case ActiveScreen::SCREEN_RFID:
                if (rfidCardView.active) {
                    rfidCardView.active = false;
                    setScreen(ActiveScreen::SCREEN_FACE);
                } else {
                    if (x >= 80 && x <= 240 && y >= 175 && y <= 213) {
                        setScreen(ActiveScreen::SCREEN_MENU);
                    }
                }
                break;

            case ActiveScreen::SCREEN_WIFI:
                if (x >= 20 && x <= 150 && y >= 160 && y <= 198) {
                    if (WiFiManagerDriver::getInstance().isPortalActive()) {
                        WiFiManagerDriver::getInstance().stopProvisioningPortal();
                    } else {
                        WiFiManagerDriver::getInstance().startProvisioningPortal();
                    }
                    forceRedraw = true;
                } else if (x >= 170 && x <= 300 && y >= 160 && y <= 198) {
                    setScreen(ActiveScreen::SCREEN_MENU);
                }
                break;

            default:
                if (x >= 80 && x <= 240 && y >= 175 && y <= 213) {
                    setScreen(ActiveScreen::SCREEN_MENU);
                }
                break;
        }
    }

    void update(RobotState state, bool wifiOk, int rssi, bool wsOk, uint8_t vol) {
        unsigned long now = millis();

        RfidDisplayResult result{};
        if (rfidDisplayQueue && xQueueReceive(rfidDisplayQueue, &result, 0) == pdTRUE) {
            rememberRfidResult(result);
            if (rfidCardView.active && rfidCardView.uid == result.uid &&
                static_cast<int32_t>(result.receivedAt - rfidCardView.scannedAt) >= 0) {
                if (rfidCardView.lookup != RfidLookupState::Ready || rfidCardView.found != result.found ||
                    rfidCardView.itemKey != result.itemKey || rfidCardView.nameEn != result.nameEn) {
                    applyRfidResult(result, now);
                }
                Serial.printf("[RFID] Result uid=%s scan-to-reply=%lu ms\n",
                              result.uid, result.receivedAt - rfidCardView.scannedAt);
            }
        }
        if (rfidCardView.active && rfidCardView.lookup == RfidLookupState::Pending &&
            now - rfidCardView.displayStartTime >= 4000) {
            rfidCardView.lookup = RfidLookupState::Unavailable;
            rfidCardView.displayStartTime = now;
            forceRedraw = true;
        }

        // Auto-return from RFID learning card to robot face after 6 seconds
        if (currentScreen == ActiveScreen::SCREEN_RFID && rfidCardView.active) {
            if (now - rfidCardView.displayStartTime >= 6000) {
                rfidCardView.active = false;
                setScreen(ActiveScreen::SCREEN_FACE);
            }
        }

        if (now - lastStatusBarDraw > 1000 || forceRedraw) {
            drawStatusBar(wifiOk, rssi, wsOk, vol);
            lastStatusBarDraw = now;
        }

        if (currentScreen == ActiveScreen::SCREEN_FACE) {
            eyes.render(forceRedraw);
            renderMouth(state, now);
        } else if (forceRedraw) {
            switch (currentScreen) {
                case ActiveScreen::SCREEN_MENU:        renderScreenMenu(); break;
                case ActiveScreen::SCREEN_EMOTIONS:    renderScreenEmotions(); break;
                case ActiveScreen::SCREEN_VOICE:       renderScreenVoice(state); break;
                case ActiveScreen::SCREEN_VOLUME:      renderScreenVolume(vol, ConfigManager::getInstance().getBrightness()); break;
                case ActiveScreen::SCREEN_WIFI:        renderScreenWifi(wifiOk, WiFiManagerDriver::getInstance().getIpAddress(), rssi, ConfigManager::getInstance().getWifiSSID()); break;
                case ActiveScreen::SCREEN_INFO:        renderScreenInfo(); break;
                case ActiveScreen::SCREEN_DIAGNOSTICS: renderScreenDiagnostics(); break;
                case ActiveScreen::SCREEN_RFID:
                    renderScreenRfid();
                    if (rfidCardView.logDraw) {
                        Serial.printf("[RFID] Drawn uid=%s scan-to-display=%lu ms\n",
                                      rfidCardView.uid.c_str(), millis() - rfidCardView.scannedAt);
                        rfidCardView.logDraw = false;
                    }
                    break;
                default: break;
            }
        }

        forceRedraw = false;
    }
};

// ============================================================================
// SECTION 11: HARDWARE SENSORS (TTP223 Head Touch, Buttons, PN532 RFID)
// ============================================================================
class SensorsDriver {
private:
    HardwareSerial nfcSerial;
    Adafruit_PN532* nfc = nullptr;
    bool nfcReady = false;

    // TTP223 Head touch state
    bool headTouched = false;

    // Physical volume buttons
    int lastBtnVolUp = HIGH;
    int lastBtnVolDown = HIGH;
    unsigned long lastBtnDebounce = 0;

    // RFID tag cache
    String lastScannedUid = "";
    unsigned long lastRfidScanTime = 0;

public:
    SensorsDriver() : nfcSerial(PN532_UART_NUM) {}

    void begin() {
        // 1. TTP223 Head Touch Sensor (Active HIGH, pull-down to eliminate floating noise)
        pinMode(PIN_TOUCH_SENSOR, INPUT_PULLDOWN);

        // 2. Physical Buttons (Active LOW, internal pull-up)
        pinMode(PIN_BTN_VOL_UP, INPUT_PULLUP);
        pinMode(PIN_BTN_VOL_DOWN, INPUT_PULLUP);

        // 3. PN532 RFID on UART1 (High Speed UART mode)
        // Explicitly set ESP32-S3 UART1 pins and construct Adafruit_PN532
        nfcSerial.setPins(PIN_PN532_RX, PIN_PN532_TX);
        nfcSerial.begin(115200, SERIAL_8N1, PIN_PN532_RX, PIN_PN532_TX);
        nfc = new Adafruit_PN532(0xFF, &nfcSerial); // 0xFF (-1) = no hardware reset line
        nfc->begin();
        nfcSerial.begin(115200, SERIAL_8N1, PIN_PN532_RX, PIN_PN532_TX); // Ensure pins are locked after begin()

        uint32_t ver = nfc->getFirmwareVersion();
        if (ver) {
            nfc->SAMConfig();
            nfc->setPassiveActivationRetries(0x01); // Non-blocking fast poll
            nfcReady = true;
            Serial.printf("[Sensors] PN532 RFID ready: Chip PN5%02X, Firmware %d.%d\n", (ver >> 24) & 0xFF, (ver >> 16) & 0xFF, (ver >> 8) & 0xFF);
        } else {
            nfcReady = false;
            Serial.println("[Sensors] NOTICE: PN532 RFID not responding on UART1 (Check SEL0=0, SEL1=0 for HSU).");
        }
    }

    void update() {
        unsigned long now = millis();

        // 1. TTP223 Head Touch Processing
        int touchReading = digitalRead(PIN_TOUCH_SENSOR);
        if (touchReading == HIGH && !headTouched) {
            headTouched = true;
            postEvent(EventType::TOUCH_SENSOR_PRESS);
            Serial.println("[Sensors] Head touch pressed.");
        } else if (touchReading == LOW && headTouched) {
            headTouched = false;
            postEvent(EventType::TOUCH_SENSOR_RELEASE);
            Serial.println("[Sensors] Head touch released.");
        }

        // 2. Physical Volume Buttons
        if (now - lastBtnDebounce > 50) {
            int rUp = digitalRead(PIN_BTN_VOL_UP);
            int rDown = digitalRead(PIN_BTN_VOL_DOWN);

            if (rUp == LOW && lastBtnVolUp == HIGH) {
                uint8_t curV = ConfigManager::getInstance().getVolume();
                uint8_t newV = (curV + VOLUME_STEP <= MAX_VOLUME_LIMIT) ? (curV + VOLUME_STEP) : MAX_VOLUME_LIMIT;
                ConfigManager::getInstance().setVolume(newV);
                postEvent(EventType::BUTTON_VOL_UP, newV);
                Serial.printf("[Buttons] Vol Up -> %u%%\n", newV);
                lastBtnDebounce = now;
            }
            if (rDown == LOW && lastBtnVolDown == HIGH) {
                uint8_t curV = ConfigManager::getInstance().getVolume();
                uint8_t newV = (curV >= VOLUME_STEP) ? (curV - VOLUME_STEP) : 0;
                ConfigManager::getInstance().setVolume(newV);
                postEvent(EventType::BUTTON_VOL_DOWN, newV);
                Serial.printf("[Buttons] Vol Down -> %u%%\n", newV);
                lastBtnDebounce = now;
            }

            lastBtnVolUp = rUp;
            lastBtnVolDown = rDown;
        }

        // 3. PN532 RFID Polling (Supports 4, 7, and 10-byte UIDs without buffer overflow)
        static uint8_t missedRfidPolls = 0;
        if (nfcReady && (now - lastRfidScanTime >= 100)) {
            lastRfidScanTime = now;
            uint8_t uid[12];
            uint8_t uidLength = 0;
            if (nfc->readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 20)) {
                char hexStr[32] = {0};
                for (uint8_t i = 0; i < uidLength && i < 12; i++) {
                    sprintf(hexStr + (i * 2), "%02X", uid[i]);
                }
                String cardUid = String(hexStr);
                missedRfidPolls = 0;
                if (cardUid != lastScannedUid) {
                    lastScannedUid = cardUid;
                    postEvent(EventType::RFID_CARD_DETECTED, static_cast<int>(millis()), cardUid.c_str());
                    Serial.printf("[Sensors] RFID Scanned UID (%d bytes): %s\n", uidLength, cardUid.c_str());
                }
            } else if (missedRfidPolls < 2 && ++missedRfidPolls == 2) {
                lastScannedUid = "";
            }
        }
    }
};

SensorsDriver sensors;

// ============================================================================
// SECTION 12: PRODUCTION TLS WEBSOCKET CLIENT (Fastify + Gemini Live)
// ============================================================================
class WebSocketClientManager {
private:
    uint32_t currentSessionGeneration = 0;
    String requestedVoiceId;
    String receivingAudioId;
    bool receivingAdpcm = false;
    uint8_t leftoverByte = 0;
    bool hasLeftoverByte = false;
    std::unique_ptr<WebsocketsClient> client{new WebsocketsClient()};
    bool wsConnected = false;
    bool authenticated = false;
    unsigned long lastReconnect = 0;
    unsigned long reconnectInterval = 2000;
    unsigned long lastHeartbeatTime = 0;
    unsigned long wsConnectedAt = 0;
    unsigned long lastCredentialWarning = 0;
    SemaphoreHandle_t wsMutex = nullptr;

public:
    static WebSocketClientManager& getInstance() {
        static WebSocketClientManager instance;
        return instance;
    }

    void begin() {
        if (!wsMutex) wsMutex = xSemaphoreCreateMutex();
        Serial.println("[WebSocket] Cloud WebSocket Manager initialized.");
    }

    bool isAuthenticated() const { return authenticated; }

    void resetAssembler() {
        hasLeftoverByte = false;
        leftoverByte = 0;
    }

    void handleAdpcmFrame(const uint8_t* raw, size_t length) {
        // Four-byte predictor/index/padding header, then one nibble per 24 kHz sample.
        if (length <= 4 || length > 2884 || raw[2] > 88 || raw[3] > 1) return;
        static constexpr int16_t steps[] = {
            7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31,
            34, 37, 41, 45, 50, 55, 60, 66, 73, 80, 88, 97, 107, 118, 130, 143,
            157, 173, 190, 209, 230, 253, 279, 307, 337, 371, 408, 449, 494, 544, 598, 658,
            724, 796, 876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066, 2272, 2499, 2749, 3024,
            3327, 3660, 4026, 4428, 4871, 5358, 5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487,
            12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
        };
        static constexpr int8_t indexDelta[] = {-1, -1, -1, -1, 2, 4, 6, 8};
        int predictor = static_cast<int16_t>(raw[0] | (raw[1] << 8));
        int index = raw[2];
        const size_t count = (length - 4) * 2 - raw[3];
        int16_t samples[512];
        size_t buffered = 0;
        auto& output = AudioOutputDriver::getInstance();
        for (size_t i = 0; i < count; ++i) {
            const uint8_t code = (raw[4 + (i >> 1)] >> ((i & 1) * 4)) & 15;
            const int step = steps[index];
            int change = step >> 3;
            if (code & 4) change += step;
            if (code & 2) change += step >> 1;
            if (code & 1) change += step >> 2;
            predictor = std::clamp(predictor + ((code & 8) ? -change : change), -32768, 32767);
            index = std::clamp(index + indexDelta[code & 7], 0, 88);
            samples[buffered++] = static_cast<int16_t>(predictor);
            if (buffered == 512) {
                if (!output.enqueuePcm(samples, buffered, currentSessionGeneration)) return;
                buffered = 0;
            }
        }
        if (buffered) output.enqueuePcm(samples, buffered, currentSessionGeneration);
    }

    void handleBinaryMessage(const uint8_t* raw, size_t rawLen) {
        if (!raw || rawLen == 0 || !currentSessionGeneration) return;
        if (receivingAdpcm) {
            handleAdpcmFrame(raw, rawLen);
            return;
        }

        size_t readOffset = 0;

        // Reconstruct odd byte left over from the previous frame of THIS session
        if (hasLeftoverByte && rawLen > 0) {
            uint16_t s = (uint16_t)leftoverByte | ((uint16_t)raw[0] << 8);
            int16_t sample = (int16_t)s;
            readOffset = 1;
            hasLeftoverByte = false;
            leftoverByte = 0;

            AudioOutputDriver::getInstance().enqueuePcm(&sample, 1, currentSessionGeneration);
        }

        // The ring buffer copies bytes, so audio frames do not need a temporary aligned buffer.
        const size_t fullSamples = (rawLen - readOffset) / sizeof(int16_t);
        if (fullSamples) {
            AudioOutputDriver::getInstance().enqueuePcm(raw + readOffset, fullSamples, currentSessionGeneration);
            readOffset += fullSamples * sizeof(int16_t);
        }

        // Save leftover byte if frame has odd length
        if (readOffset < rawLen) {
            leftoverByte = raw[readOffset];
            hasLeftoverByte = true;
        }
    }

    void connect() {
        if (!WiFiManagerDriver::getInstance().isConnected()) return;

        String devId = ConfigManager::getInstance().getDeviceId();
        String apiKey = ConfigManager::getInstance().getDeviceApiKey();
        if (devId.length() == 0 || apiKey.length() == 0) {
            const unsigned long now = millis();
            if (lastCredentialWarning == 0 || now - lastCredentialWarning >= 15000) {
                lastCredentialWarning = now;
                Serial.println("[WebSocket] BLOCKED: Device ID or API key is missing in NVS.");
                Serial.println("[WebSocket] Set credentials with PROV:AUTH:<deviceId>:<rawApiKey> or the Wi-Fi setup portal.");
            }
            return;
        }

        // TLS certificate validation requires a valid wall clock.
        if (!WiFiManagerDriver::getInstance().isSntpSynced()) {
            Serial.println("[WebSocket] Waiting for SNTP time synchronization before TLS...");
            return;
        }

        IPAddress serverIp;
        if (WiFi.hostByName(DEFAULT_WS_HOST, serverIp) != 1) {
            Serial.printf("[WebSocket] DNS lookup FAILED for %s\n", DEFAULT_WS_HOST);
            return;
        }
        Serial.printf("[WebSocket] DNS %s -> %s\n", DEFAULT_WS_HOST, serverIp.toString().c_str());
        Serial.printf("[WebSocket] Connecting to wss://%s:%d%s (device key present) ...\n",
                      DEFAULT_WS_HOST, DEFAULT_WS_PORT, DEFAULT_WS_PATH);

#ifndef WS_PROBE
#define WS_PROBE 0
#endif
#if WS_PROBE
        static bool probed = false;
        if (!probed) { probed = true; probeUpgrade(devId, apiKey); }
#endif

        if (xSemaphoreTake(wsMutex, pdMS_TO_TICKS(500))) {
            resetAssembler(); // Clear any stale incomplete sample
            client.reset(new WebsocketsClient());
            // Keep certificate and hostname verification enabled. Never use setInsecure().
            client->setCACert(PROD_ROOT_CA_BUNDLE);
            client->addHeader("X-Device-ID", devId);
            client->addHeader("X-Device-Key", apiKey);
            if (AudioOutputDriver::getInstance().hasPsramAudioBuffer()) {
                client->addHeader("X-Audio-Codec", "ima-adpcm");
            }

            client->onMessage([this](WebsocketsMessage msg) {
                if (msg.isBinary()) {
                    handleBinaryMessage((const uint8_t*)msg.c_str(), msg.length());
                } else if (msg.isText()) {
                    handleTextMessage(msg.data());
                }
            });

            client->onEvent([this](WebsocketsEvent event, String data) {
                if (event == WebsocketsEvent::ConnectionOpened) {
                    wsConnected = true;
                    wsConnectedAt = millis();
                    Serial.println("[WebSocket] TLS connection opened. Waiting for backend device:connected authentication...");
                } else if (event == WebsocketsEvent::ConnectionClosed) {
                    resetAssembler(); // Clear assembler on disconnect
                    requestedVoiceId = "";
                    receivingAudioId = "";
                    currentSessionGeneration = 0;
                    wsConnected = false;
                    authenticated = false;
                    wsConnectedAt = 0;
                    postEvent(EventType::WS_DISCONNECTED);
                    Serial.printf("[WebSocket] Connection closed. Reason: %s (code: %d)\n", data.c_str(), (int)client->getCloseReason());
                }
            });

            const uint32_t t0 = millis();
            bool ok = client->connectSecure(DEFAULT_WS_HOST, DEFAULT_WS_PORT, DEFAULT_WS_PATH);
            const uint32_t dt = millis() - t0;
            xSemaphoreGive(wsMutex);
            if (!ok) {
                wsConnected = false;
                authenticated = false;
                wsConnectedAt = 0;
                // NOTE: "Connection closed." printed just before this line means the TLS session WAS
                // established; the HTTP Upgrade then got no "HTTP/1.1 101" (too slow, or 4xx/5xx).
                Serial.printf("[WebSocket] Connect/Upgrade failed after %lu ms (free heap %u B). "
                              "If 'Connection closed' was printed above, TLS is OK - check the Upgrade response/latency.\n",
                              (unsigned long)dt, (unsigned)ESP.getFreeHeap());
            }
        }
    }

    // ---- TEMPORARY DIAGNOSTIC (set WS_PROBE to 0 to disable) ------------------------------
    // Sends the same Upgrade request ArduinoWebsockets sends (same headers, incl. Origin and
    // User-Agent) over a plain WiFiClientSecure and prints TLS time, time-to-first-byte and
    // the raw response head. The API key is never printed. Runs once per boot.
    void probeUpgrade(const String& devId, const String& apiKey) {
        WiFiClientSecure s;
        s.setCACert(PROD_ROOT_CA_BUNDLE);
        const uint32_t t0 = millis();
        if (!s.connect(DEFAULT_WS_HOST, DEFAULT_WS_PORT)) {
            Serial.printf("[Probe] TLS connect FAILED after %lu ms\n", (unsigned long)(millis() - t0));
            return;
        }
        const uint32_t tTls = millis() - t0;
        String req = String("GET ") + DEFAULT_WS_PATH + " HTTP/1.1\r\n";
        req += String("Host: ") + DEFAULT_WS_HOST + "\r\n";
        req += "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n";
        req += "X-Device-ID: " + devId + "\r\n";
        req += "X-Device-Key: " + apiKey + "\r\n";
        req += "Upgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Version: 13\r\n";
        req += "User-Agent: TinyWebsockets Client\r\n";
        req += "Origin: https://github.com/gilmaimon/TinyWebsockets\r\n\r\n";
        s.print(req);

        const uint32_t t1 = millis();
        uint32_t tFirst = 0;
        String resp;
        while (millis() - t1 < 10000 && resp.length() < 400) {
            while (s.available() && resp.length() < 400) {
                if (!tFirst) tFirst = millis() - t1;
                resp += (char)s.read();
            }
            if (resp.indexOf("\r\n\r\n") >= 0) break;
            if (!s.connected() && !s.available()) break;
            delay(5);
        }
        s.stop();
        resp.replace("\r", "");
        Serial.printf("[Probe] TLS %lu ms | first byte %lu ms after request (0 = none in 10 s) | %u bytes\n",
                      (unsigned long)tTls, (unsigned long)tFirst, (unsigned)resp.length());
        Serial.println("[Probe] ---- response head ----");
        Serial.println(resp);
        Serial.println("[Probe] -----------------------");
    }

    void handleTextMessage(const String& jsonStr) {
        JsonDocument doc;
        if (deserializeJson(doc, jsonStr)) return;

        const char* ev = doc["event"] | doc["type"] | "";
        Serial.printf("[WebSocket] Server event: %s\n", ev[0] ? ev : "<missing>");
        JsonObject data = doc["data"].as<JsonObject>();
        if (data.isNull()) data = doc["payload"].as<JsonObject>();

        if (strcmp(ev, "device:connected") == 0) {
            authenticated = true;
            wsConnectedAt = 0;
            reconnectInterval = 2000;
            Serial.println("[WebSocket] Handshake SUCCESS: device:connected acknowledged!");
            if (!data["volume"].isNull()) {
                uint8_t v = data["volume"];
                ConfigManager::getInstance().setVolume(v);
                AudioOutputDriver::getInstance().setVolume(v);
            }
            postEvent(EventType::WS_AUTHENTICATED);
        } else if (strcmp(ev, "audio:response:start") == 0) {
            resetAssembler(); // Clear odd-byte assembler on new session boundary
            uint32_t sRate = data["sampleRate"] | 24000;
            const char* sessId = data["sessionId"] | "";
            if (strncmp(sessId, "voice_", 6) == 0 && requestedVoiceId != sessId) {
                Serial.println("[WebSocket] Ignored stale voice response.");
                currentSessionGeneration = 0;
                receivingAudioId = "";
                return;
            }
            receivingAudioId = sessId;
            receivingAdpcm = strcmp(data["wireCodec"] | "", "ima-adpcm") == 0;
            Serial.printf("[AudioOutput] Wire codec: %s\n", receivingAdpcm ? "ima-adpcm" : "pcm");
            const char* srcType = data["source"] | data["type"] | "";
            bool isMp3 = (strcmp(srcType, "mp3") == 0 || (sessId && strstr(sessId, "mp3") != nullptr));
            const uint16_t initialBufferMs = (receivingAdpcm && strcmp(srcType, "quick_reply") == 0) ? 480 : 0;
            currentSessionGeneration = AudioOutputDriver::getInstance().startSession(sessId, sRate, isMp3, initialBufferMs);
            postEvent(EventType::VOICE_RESPONSE_START, (int)sRate, sessId);
            Serial.printf("[WebSocket] AI Voice response starting: %s (gen: %lu, %lu Hz, mp3=%d)\n",
                          sessId[0] ? sessId : "<none>", (unsigned long)currentSessionGeneration, (unsigned long)sRate, (int)isMp3);
        } else if (strcmp(ev, "audio:response:end") == 0) {
            const char* sessId = data["sessionId"] | "";
            if (!currentSessionGeneration || (*sessId && receivingAudioId != sessId)) return;
            currentSessionGeneration = 0;
            // Keep the speaker session ID until the next start or cancel so Stop also works during drain.
            resetAssembler(); // Clear odd-byte assembler on session end
            AudioOutputDriver::getInstance().endSession();
            postEvent(EventType::VOICE_RESPONSE_END);
            Serial.println("[WebSocket] AI Voice response finished.");
        } else if (strcmp(ev, "audio:cancel") == 0) {
            const char* stopId = data["sessionId"] | "";
            if (*stopId && (receivingAudioId != stopId ||
                (!currentSessionGeneration && !AudioOutputDriver::getInstance().hasBufferedAudio()))) return;
            receivingAudioId = "";
            resetAssembler(); // Clear odd-byte assembler on session cancel
            const char* reason = data["reason"] | "interrupted";
            AudioOutputDriver::getInstance().cancelSession(reason);
            currentSessionGeneration = 0; // Invalidate client session generation immediately
            postEvent(EventType::VOICE_AUDIO_CANCEL);
        } else if (strcmp(ev, "rfid:result") == 0) {
            const char* cardUid = data["cardUid"] | "";
            bool found = data["found"] | false;
            const char* itemKey = data["itemKey"] | "";
            const char* nameEn = data["nameEn"] | "";
            const char* nameMm = data["nameMm"] | "";
            Serial.printf("[WebSocket] RFID Result: uid=%s, found=%d, item=%s, en=%s, mm=%s\n",
                          cardUid, (int)found, itemKey, nameEn, nameMm);
            RfidDisplayResult result{};
            snprintf(result.uid, sizeof(result.uid), "%s", cardUid);
            snprintf(result.itemKey, sizeof(result.itemKey), "%s", itemKey);
            snprintf(result.nameEn, sizeof(result.nameEn), "%s", nameEn);
            result.found = found;
            result.receivedAt = millis();
            if (rfidDisplayQueue) xQueueOverwrite(rfidDisplayQueue, &result);
        } else if (strcmp(ev, "robot:command") == 0) {
            const char* cmd = data["type"] | "";
            if (strcmp(cmd, "set_volume") == 0) {
                uint8_t v = data["payload"]["volume"] | 80;
                ConfigManager::getInstance().setVolume(v);
                AudioOutputDriver::getInstance().setVolume(v);
            } else if (strcmp(cmd, "reboot") == 0) {
                ESP.restart();
            }
        }
    }

    bool sendAudioReadyAck(const char* sessId, uint32_t gen) {
        JsonDocument doc;
        doc["event"] = "audio:ready";
        JsonObject data = doc["data"].to<JsonObject>();
        data["sessionId"] = sessId ? sessId : "";
        data["generation"] = gen;
        data["sampleRate"] = 24000;
        data["status"] = "ready";

        String out;
        serializeJson(doc, out);
        if (xSemaphoreTake(wsMutex, pdMS_TO_TICKS(50))) {
            bool ok = (client->available() && authenticated) ? client->send(out) : false;
            xSemaphoreGive(wsMutex);
            Serial.printf("[WebSocket] Handshake ACK: Sent audio:ready for session %s (gen: %lu)\n",
                          sessId ? sessId : "<none>", (unsigned long)gen);
            return ok;
        }
        return false;
    }

    bool sendFlowControl(const char* status, const char* sessId, uint32_t gen) {
        JsonDocument doc;
        doc["event"] = "audio:flow";
        JsonObject data = doc["data"].to<JsonObject>();
        data["status"] = status;
        data["sessionId"] = sessId ? sessId : "";
        data["generation"] = gen;
        data["freeBytes"] = AudioOutputDriver::getInstance().getFreeBufferBytes();

        String out;
        serializeJson(doc, out);
        if (xSemaphoreTake(wsMutex, pdMS_TO_TICKS(50))) {
            bool ok = (client->available() && authenticated) ? client->send(out) : false;
            xSemaphoreGive(wsMutex);
            if (ok) {
                Serial.printf("[WebSocket] Watermark Flow Control: Sent audio:flow %s (gen: %lu, free: %u B)\n",
                              status, (unsigned long)gen, (unsigned)AudioOutputDriver::getInstance().getFreeBufferBytes());
            }
            return ok;
        }
        return false;
    }

    void update() {
        if (xSemaphoreTake(wsMutex, pdMS_TO_TICKS(20))) {
            if (wsConnected) client->poll(4);
            xSemaphoreGive(wsMutex);
        }

        // Handshake & Flow Control: Only clear flags upon confirmed send success (Critical Fix A & B)
        if (wsConnected && authenticated) {
            if (AudioOutputDriver::getInstance().isReadyAckPending()) {
                if (sendAudioReadyAck(AudioOutputDriver::getInstance().getActiveSessionId(),
                                      AudioOutputDriver::getInstance().getActiveGeneration())) {
                    AudioOutputDriver::getInstance().clearReadyAck();
                }
            }
            if (AudioOutputDriver::getInstance().isFlowPausePending()) {
                if (sendFlowControl("pause", AudioOutputDriver::getInstance().getActiveSessionId(),
                                    AudioOutputDriver::getInstance().getActiveGeneration())) {
                    AudioOutputDriver::getInstance().clearFlowPause();
                }
            }
            if (AudioOutputDriver::getInstance().isFlowResumePending()) {
                if (sendFlowControl("resume", AudioOutputDriver::getInstance().getActiveSessionId(),
                                    AudioOutputDriver::getInstance().getActiveGeneration())) {
                    AudioOutputDriver::getInstance().clearFlowResume();
                }
            }
        }

        unsigned long now = millis();
        if (wsConnected && !authenticated && wsConnectedAt != 0 && now - wsConnectedAt > 20000) {
            Serial.println("[WebSocket] No device:connected acknowledgement after 20 seconds; closing and retrying.");
            if (xSemaphoreTake(wsMutex, pdMS_TO_TICKS(100))) {
                client->close();
                xSemaphoreGive(wsMutex);
            }
            wsConnected = false;
            wsConnectedAt = 0;
        }

        if (!wsConnected && WiFiManagerDriver::getInstance().isConnected()) {
            if (now - lastReconnect > reconnectInterval) {
                lastReconnect = now;
                reconnectInterval = std::min<unsigned long>(reconnectInterval * 2, 8000);
                connect();
            }
        }

        // Heartbeat Telemetry (Strictly accurate measurements only, no fabricated battery level)
        if (authenticated && (now - lastHeartbeatTime > WS_HEARTBEAT_INTERVAL_MS)) {
            lastHeartbeatTime = now;
            sendHeartbeat();
        }
    }

    void sendHeartbeat() {
        JsonDocument doc;
        doc["event"] = "device:heartbeat";
        JsonObject data = doc["data"].to<JsonObject>();
        data["freeHeap"] = ESP.getFreeHeap();
        data["uptimeSeconds"] = millis() / 1000;
        data["wifiRSSI"] = WiFiManagerDriver::getInstance().getRSSI();
        data["cpuTemp"] = temperatureRead();

        String out;
        serializeJson(doc, out);
        if (xSemaphoreTake(wsMutex, pdMS_TO_TICKS(50))) {
            if (client->available()) client->send(out);
            xSemaphoreGive(wsMutex);
        }
    }

    bool sendAudioStart(const String& sessId, uint32_t sRate) {
        JsonDocument doc;
        doc["event"] = "audio:start";
        JsonObject data = doc["data"].to<JsonObject>();
        data["sessionId"] = sessId;
        data["format"] = "pcm_16bit";
        data["sampleRate"] = sRate;

        String out;
        serializeJson(doc, out);
        if (xSemaphoreTake(wsMutex, pdMS_TO_TICKS(50))) {
            bool ok = client->available() ? client->send(out) : false;
            if (ok) requestedVoiceId = sessId;
            xSemaphoreGive(wsMutex);
            return ok;
        }
        return false;
    }

    bool sendAudioChunk(const uint8_t* pcm, size_t len) {
        if (!pcm || len == 0) return false;
        if (xSemaphoreTake(wsMutex, pdMS_TO_TICKS(100))) {
            bool ok = (client->available() && authenticated) ? client->sendBinary((const char*)pcm, len) : false;
            xSemaphoreGive(wsMutex);
            return ok;
        }
        return false;
    }

    bool sendAudioEnd(uint32_t chunks) {
        JsonDocument doc;
        doc["event"] = "audio:end";
        JsonObject data = doc["data"].to<JsonObject>();
        data["totalChunks"] = chunks;

        String out;
        serializeJson(doc, out);
        if (xSemaphoreTake(wsMutex, pdMS_TO_TICKS(250))) {
            bool ok = client->available() ? client->send(out) : false;
            xSemaphoreGive(wsMutex);
            return ok;
        }
        return false;
    }

    bool sendTouchEvent(uint8_t pad, bool pressed) {
        JsonDocument doc;
        doc["event"] = "touch:event";
        JsonObject data = doc["data"].to<JsonObject>();
        data["pad"] = pad;
        data["pressed"] = pressed;

        String out;
        serializeJson(doc, out);
        if (xSemaphoreTake(wsMutex, pdMS_TO_TICKS(50))) {
            bool ok = client->available() ? client->send(out) : false;
            xSemaphoreGive(wsMutex);
            return ok;
        }
        return false;
    }

    bool sendRfidScan(const String& uid) {
        JsonDocument doc;
        doc["event"] = "rfid:scan";
        JsonObject data = doc["data"].to<JsonObject>();
        data["cardUid"] = uid;

        String out;
        serializeJson(doc, out);
        if (xSemaphoreTake(wsMutex, pdMS_TO_TICKS(50))) {
            bool ok = client->available() ? client->send(out) : false;
            xSemaphoreGive(wsMutex);
            return ok;
        }
        return false;
    }

    bool sendAudioCancel(const char* reason = "user_cancel") {
        JsonDocument doc;
        doc["event"] = "audio:cancel";
        JsonObject data = doc["data"].to<JsonObject>();
        data["reason"] = reason ? reason : "user_cancel";

        String out;
        serializeJson(doc, out);
        if (xSemaphoreTake(wsMutex, pdMS_TO_TICKS(50))) {
            requestedVoiceId = "";
            bool ok = client->available() ? client->send(out) : false;
            xSemaphoreGive(wsMutex);
            Serial.printf("[WebSocket] Sent audio:cancel (reason: %s)\n", reason ? reason : "user_cancel");
            return ok;
        }
        return false;
    }
};

// ============================================================================
// SECTION 13: DEVICE MANAGER & GEMINI LIVE VOICE PIPELINE FSM
// ============================================================================
class DeviceManager {
private:
    volatile RobotState currentState = RobotState::IDLE;
    bool waitingForPlaybackDrain = false;
    bool voiceReply = false;
    std::atomic<unsigned long> followupStarted{0};
    unsigned long drainCompleteTime = 0;
    String currentSessionId = "";
    volatile uint32_t audioChunkCount = 0;
    volatile unsigned long listeningStartTime = 0;
    volatile unsigned long thinkingStartTime = 0;

public:
    static DeviceManager& getInstance() {
        static DeviceManager instance;
        return instance;
    }

    bool followupOpen() const {
        const unsigned long start = followupStarted.load();
        return start && millis() - start < 8000;
    }

    RobotState getState() const { return currentState; }
    void incrementAudioChunkCount() { audioChunkCount++; }
    unsigned long getListeningStartTime() const { return listeningStartTime; }

    void setState(RobotState st) {
        if (currentState != st) {
            currentState = st;
            Serial.printf("[DeviceManager] State: %s\n", robotStateToString(st));

            // Fix A: Reliable WakeNet re-arming on every transition to IDLE; disarm during any active state
            if (st == RobotState::IDLE) {
                WakeWordEngine::getInstance().setListening(true);
            } else {
                WakeWordEngine::getInstance().setListening(false);
            }

            switch (st) {
                case RobotState::IDLE:
                    eyes.setEmotion(RobotEmotion::NEUTRAL);
                    break;
                case RobotState::LISTENING:
                    eyes.setEmotion(RobotEmotion::QUESTIONING);
                    break;
                case RobotState::THINKING:
                    eyes.setEmotion(RobotEmotion::IDLE2);
                    break;
                case RobotState::SPEAKING:
                    eyes.setEmotion(RobotEmotion::HAPPY);
                    break;
                case RobotState::ALERT:
                    eyes.setEmotion(RobotEmotion::EXCITED);
                    break;
                case RobotState::ERROR:
                    eyes.setEmotion(RobotEmotion::SAD);
                    break;
                default: break;
            }
        }
    }

    // Accepts call from IDLE or conversational SPEAKING state
    void startVoiceSession(bool followup = false) {
        if (currentState != RobotState::IDLE && currentState != RobotState::SPEAKING && currentState != RobotState::THINKING) return;
        if (!WebSocketClientManager::getInstance().isAuthenticated()) {
            Serial.println("[DeviceManager] Cannot start voice: WebSocket not authenticated.");
            if (currentState == RobotState::IDLE) WakeWordEngine::getInstance().setListening(true);
            return;
        }

        // 1. If speaker is still playing previous response, flush it so mic doesn't capture speaker echo
        if (AudioOutputDriver::getInstance().hasBufferedAudio()) {
            AudioOutputDriver::getInstance().stopAndFlush();
            waitingForPlaybackDrain = false;
        }

        followupStarted.store(0);
        waitingForPlaybackDrain = false;
        String devId = ConfigManager::getInstance().getDeviceId();
        if (!followup) currentSessionId = "voice_" + devId + "_" + String(millis());
        audioChunkCount = 0;
        listeningStartTime = millis();

        // 2. Verify WebSocket sendAudioStart succeeded before starting mic DMA
        bool started = WebSocketClientManager::getInstance().sendAudioStart(currentSessionId, AUDIO_IN_SAMPLE_RATE);
        if (!started) {
            Serial.println("[DeviceManager] Failed to send audio:start to server! Aborting voice session.");
            if (currentState == RobotState::IDLE) WakeWordEngine::getInstance().setListening(true);
            return;
        }

        AudioInputDriver::getInstance().startRecording();
        setState(RobotState::LISTENING);
        Serial.printf("[DeviceManager] Voice session started: %s\n", currentSessionId.c_str());
    }

    void endVoiceSession() {
        if (currentState != RobotState::LISTENING) return;

        AudioInputDriver::getInstance().stopRecording();
        if (audioChunkCount == 0) {
            // Clean up backend session cleanly even if no audio was captured
            WebSocketClientManager::getInstance().sendAudioCancel("empty_capture");
            setState(RobotState::IDLE);
            return;
        }

        if (!WebSocketClientManager::getInstance().isAuthenticated() ||
            !WebSocketClientManager::getInstance().sendAudioEnd(audioChunkCount)) {
            Serial.println("[DeviceManager] audio:end send failed.");
            setState(RobotState::IDLE);
            return;
        }
        thinkingStartTime = millis();
        setState(RobotState::THINKING);
        Serial.printf("[DeviceManager] Audio capture ended (%lu chunks sent). Waiting for Gemini Live...\n", (unsigned long)audioChunkCount);
    }

    void handleEvent(const AppEvent& ev) {
        switch (ev.type) {
            case EventType::TOUCH_SENSOR_PRESS:
                // Note: Do not trigger speaker chime here as it bleeds directly into the microphone!
                WebSocketClientManager::getInstance().sendTouchEvent(0, true);
                if (currentState == RobotState::IDLE) {
                    startVoiceSession();
                } else if (currentState == RobotState::LISTENING) {
                    endVoiceSession();
                } else if (currentState == RobotState::SPEAKING || currentState == RobotState::THINKING) {
                    // Fix B: Immediate barge-in interruption during SPEAKING/THINKING
                    Serial.println("[DeviceManager] Head-touch barge-in interrupt during SPEAKING/THINKING!");
                    AudioOutputDriver::getInstance().stopAndFlush();
                    waitingForPlaybackDrain = false;
                    WebSocketClientManager::getInstance().sendAudioCancel("touch_barge_in");
                    startVoiceSession();
                }
                break;

            case EventType::TOUCH_SENSOR_RELEASE:
                WebSocketClientManager::getInstance().sendTouchEvent(0, false);
                break;

            case EventType::PROV_PORTAL_START:
                WiFiManagerDriver::getInstance().startProvisioningPortal();
                UIManager::getInstance().setScreen(ActiveScreen::SCREEN_WIFI);
                break;

            case EventType::PROV_PORTAL_STOP:
                WiFiManagerDriver::getInstance().stopProvisioningPortal();
                break;

            case EventType::VOICE_SESSION_START:
                if (ev.intValue == 0 || followupOpen()) startVoiceSession(ev.intValue == 1);
                break;

            case EventType::VOICE_SESSION_END:
                if (!ev.intValue || static_cast<unsigned long>(ev.intValue) == listeningStartTime) endVoiceSession();
                break;

            case EventType::VOICE_RESPONSE_START:
                followupStarted.store(0);
                waitingForPlaybackDrain = false;
                voiceReply = currentSessionId == ev.text;
                if (voiceReply && currentState == RobotState::THINKING) {
                    Serial.printf("[VoiceLatency] End-to-response-start: %lu ms\n", millis() - thinkingStartTime);
                }
                setState(RobotState::SPEAKING);
                break;

            case EventType::VOICE_RESPONSE_END:
                // Wait for speaker hardware to finish draining RingBuffer before going to IDLE
                Serial.println("[DeviceManager] AI response stream finished. Waiting for speaker to finish playing buffer...");
                waitingForPlaybackDrain = true;
                drainCompleteTime = 0;
                break;

            case EventType::VOICE_AUDIO_CANCEL:
                voiceReply = false;
                waitingForPlaybackDrain = false;
                if (!AudioInputDriver::getInstance().isRecording()) setState(RobotState::IDLE);
                break;

            case EventType::WS_AUTHENTICATED:
                setState(RobotState::IDLE);
                break;

            case EventType::WS_DISCONNECTED:
                AudioOutputDriver::getInstance().stopAndFlush();
                followupStarted.store(0);
                waitingForPlaybackDrain = false;
                voiceReply = false;
                AudioInputDriver::getInstance().stopRecording();
                setState(RobotState::IDLE);
                break;

            case EventType::RFID_CARD_DETECTED: {
                AudioOutputDriver::getInstance().triggerCuteChime();
                auto& cloud = WebSocketClientManager::getInstance();
                const auto scannedAt = static_cast<unsigned long>(ev.intValue);
                const bool lookupQueued = rfidLookupQueue && cloud.isAuthenticated();
                UIManager::getInstance().showRfidScan(ev.text, scannedAt, lookupQueued);
                if (lookupQueued) xQueueOverwrite(rfidLookupQueue, &ev);
                break;
            }

            case EventType::BUTTON_VOL_UP:
            case EventType::BUTTON_VOL_DOWN:
            case EventType::CMD_SET_VOLUME:
                AudioOutputDriver::getInstance().setVolume((uint8_t)ev.intValue);
                break;

            default: break;
        }
    }

    void update() {
        AppEvent ev;
        while (xQueueReceive(appEventQueue, &ev, 0)) {
            handleEvent(ev);
        }

        // Handle clean speaker buffer drain before returning to IDLE
        if (waitingForPlaybackDrain) {
            if (!AudioOutputDriver::getInstance().hasBufferedAudio()) {
                if (drainCompleteTime == 0) {
                    drainCompleteTime = millis();
                } else if (millis() - drainCompleteTime >= 100) {
                    waitingForPlaybackDrain = false;
                    drainCompleteTime = 0;
                    setState(RobotState::IDLE);
                    if (voiceReply) followupStarted.store(millis());
                    voiceReply = false;
                    Serial.println("[DeviceManager] Speaker buffer completely drained. System is now IDLE.");
                }
            } else {
                drainCompleteTime = 0;
            }
        }

        // Note: Audio capture streaming is handled asynchronously in taskAudioCapture on Core 1!
        if (currentState == RobotState::THINKING) {
            // Thinking timeout (15 seconds)
            if (millis() - thinkingStartTime > 15000) {
                Serial.println("[DeviceManager] Response timeout. Returning to IDLE.");
                setState(RobotState::IDLE);
                WebSocketClientManager::getInstance().sendAudioCancel("response_timeout");
            }
        }
    }
};

// ============================================================================
// SECTION 14: FREERTOS DUAL-CORE TASKS
// ============================================================================
TaskHandle_t taskNetworkHandle = nullptr;
TaskHandle_t taskSensorHandle = nullptr;
TaskHandle_t taskAudioHandle = nullptr;

// Network & Cloud Task (Core 0, Priority 1 to prevent IDLE0 watchdog starvation)
void taskNetwork(void* parameter) {
    Serial.println("[FreeRTOS] Network Task running on Core 0.");
    for (;;) {
        WiFiManagerDriver::getInstance().update();
        if (WiFiManagerDriver::getInstance().isConnected() && !WiFiManagerDriver::getInstance().isPortalActive()) {
            auto& cloud = WebSocketClientManager::getInstance();
            AppEvent lookup{};
            if (cloud.isAuthenticated() && rfidLookupQueue &&
                xQueueReceive(rfidLookupQueue, &lookup, 0) == pdTRUE) {
                const bool sent = cloud.sendRfidScan(String(lookup.text));
                Serial.printf("[RFID] Lookup uid=%s sent=%d scan-to-send=%lu ms\n",
                              lookup.text, (int)sent, millis() - static_cast<unsigned long>(lookup.intValue));
            }
            cloud.update();
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// Sensor Polling Task (Core 1, Priority 2)
void taskSensors(void* parameter) {
    Serial.println("[FreeRTOS] Sensor Task running on Core 1.");
    for (;;) {
        sensors.update();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// Background Audio Output Task (Core 1, Priority 3)
void taskAudio(void* parameter) {
    Serial.println("[FreeRTOS] Audio Output Task running on Core 1.");
    for (;;) {
        bool hasData = AudioOutputDriver::getInstance().update();
        if (!hasData) {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
}

// Background Audio Input Capture & WakeNet Task (Core 0, Priority 2, dedicated DMA & neural inference)
static TaskHandle_t taskAudioCaptureHandle = NULL;
void taskAudioCapture(void* parameter) {
    Serial.println("[FreeRTOS] Audio Capture & WakeNet Task running on Core 0.");
    static int16_t pcmBuffer[AUDIO_CHUNK_SIZE / 2]; // Static allocation prevents stack overflow!

    for (;;) {
        size_t readSamples = AudioInputDriver::getInstance().readPcmChunk(pcmBuffer, AUDIO_CHUNK_SIZE / 2);
        if (readSamples == 0) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        RobotState curState = DeviceManager::getInstance().getState();

        static uint32_t ambient = 120;
        static bool awaitingOnset = false;
        static unsigned long wakeAt = 0;
        static unsigned long offlinePromptAt = 0;
        static unsigned onsetFrames = 0;
        static unsigned voicedFrames = 0;
        static unsigned long lastSpeech = 0, lastStart = 0;
        static bool endQueued = false;
        static bool speechSeen = false;
        int32_t sampleSum = 0;
        for (size_t i = 0; i < readSamples; ++i) sampleSum += pcmBuffer[i];
        const int32_t mean = sampleSum / static_cast<int32_t>(readSamples);
        uint32_t sum = 0;
        for (size_t i = 0; i < readSamples; ++i) sum += abs(pcmBuffer[i] - mean);
        const uint32_t level = sum / readSamples;
        const bool speech = level > std::max<uint32_t>(100, ambient * 2);
        const bool cloudReady = WebSocketClientManager::getInstance().isAuthenticated();
        if (curState != RobotState::IDLE || (!cloudReady && awaitingOnset)) {
            if (awaitingOnset && curState == RobotState::IDLE) WakeWordEngine::getInstance().setListening(true);
            awaitingOnset = false;
            wakePrompt.store(WakePrompt::None, std::memory_order_relaxed);
        } else if (wakePrompt.load(std::memory_order_relaxed) == WakePrompt::Offline &&
                   millis() - offlinePromptAt >= 3000) {
            wakePrompt.store(WakePrompt::None, std::memory_order_relaxed);
        }
        if (curState == RobotState::IDLE && !awaitingOnset && !DeviceManager::getInstance().followupOpen())
            ambient = (31 * ambient + std::min(level, ambient * 2 + 32)) / 32;
        if (curState != RobotState::IDLE || !DeviceManager::getInstance().followupOpen()) voicedFrames = 0;

        if (curState == RobotState::LISTENING && AudioInputDriver::getInstance().isRecording()) {
            const unsigned long started = DeviceManager::getInstance().getListeningStartTime();
            if (lastStart != started) {
                lastStart = started;
                lastSpeech = started;
                endQueued = false;
                speechSeen = WakeWordEngine::getInstance().hasPreRoll();
            }
            if (endQueued) continue;
            // 1. ACTIVE CONVERSATION:
            // Flush pre-roll buffer if session was just activated via Wake Word detection
            if (WakeWordEngine::getInstance().hasPreRoll()) {
                static int16_t preRollOut[1024];
                size_t preRollCount = WakeWordEngine::getInstance().getPreRollAudio(preRollOut, 1024);
                if (preRollCount > 0) {
                    WebSocketClientManager::getInstance().sendAudioChunk((const uint8_t*)preRollOut, preRollCount * sizeof(int16_t));
                    DeviceManager::getInstance().incrementAudioChunkCount();
                }
            }

            // Stream live microphone PCM to WebSocket
            bool sent = WebSocketClientManager::getInstance().sendAudioChunk((const uint8_t*)pcmBuffer, readSamples * sizeof(int16_t));
            if (sent) {
                DeviceManager::getInstance().incrementAudioChunkCount();
            }

            if (speech) {
                speechSeen = true;
                lastSpeech = millis();
            }
            if (millis() - lastSpeech >= (speechSeen ? 1200UL : 3000UL)) {
                Serial.println("[DeviceManager] Voice capture auto-complete. Posting end event.");
                AppEvent endEvent{};
                endEvent.type = EventType::VOICE_SESSION_END;
                endEvent.intValue = static_cast<int>(started);
                endQueued = xQueueSend(appEventQueue, &endEvent, 0) == pdTRUE;
            }
        } else if (curState == RobotState::IDLE) {
            // 2. STANDBY LOCAL WAKE-WORD DETECTION:
            // Zero microphone data is sent over the network during standby (privacy preserved).
            // Prevent speaker echo: pause WakeNet if speaker is actively playing audio!
            if (!AudioOutputDriver::getInstance().isPlaying()) {
                auto& wake = WakeWordEngine::getInstance();
                if (awaitingOnset) {
                    wake.remember(pcmBuffer, readSamples);
                    const unsigned long elapsed = millis() - wakeAt;
                    if (speech && (elapsed < 3000 || onsetFrames > 0)) ++onsetFrames;
                    else onsetFrames = 0;
                    if (onsetFrames >= 2) {
                        AppEvent start{};
                        start.type = EventType::VOICE_SESSION_START;
                        wake.preservePreRoll();
                        awaitingOnset = false;
                        wakePrompt.store(WakePrompt::None, std::memory_order_relaxed);
                        if (xQueueSend(appEventQueue, &start, 0) != pdTRUE) wake.setListening(true);
                    } else if (elapsed >= 3000 && onsetFrames == 0) {
                        awaitingOnset = false;
                        wakePrompt.store(WakePrompt::None, std::memory_order_relaxed);
                        wake.setListening(true);
                        Serial.println("[WakeWord] No speech within 3s; no cloud turn opened.");
                    }
                } else if (wake.processPcmChunk(pcmBuffer, readSamples)) {
                    if (cloudReady) {
                        wakeAt = millis();
                        awaitingOnset = true;
                        onsetFrames = 0;
                        wakePrompt.store(WakePrompt::Speak, std::memory_order_relaxed);
                        Serial.println("[WakeWord] Speak now (3s).");
                    } else {
                        offlinePromptAt = millis();
                        wakePrompt.store(WakePrompt::Offline, std::memory_order_relaxed);
                        wake.setListening(true);
                        Serial.println("[WakeWord] Wake detected; server offline.");
                    }
                } else if (cloudReady && DeviceManager::getInstance().followupOpen()) {
                    voicedFrames = speech ? voicedFrames + 1 : 0;
                    if (voicedFrames == 2) {
                        WakeWordEngine::getInstance().preservePreRoll();
                        postEvent(EventType::VOICE_SESSION_START, 1);
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// ============================================================================
// SECTION 15: SERIAL CLI PROVISIONING COMMANDS
// ============================================================================
void executeCliCommand(const String& cmd) {
    if (cmd == "AUDIO:TEST") {
        auto& speaker = AudioOutputDriver::getInstance();
        if (speaker.hasBufferedAudio() || speaker.isSessionActive()) {
            Serial.println("[CLI] Stop current audio before testing the speaker.");
        } else {
            Serial.println("[CLI] Playing a 3-second local speaker test.");
            speaker.triggerCuteChime(true);
        }
    } else if (cmd.startsWith("PROV:WIFI:")) {
        int firstColon = cmd.indexOf(':', 10);
        if (firstColon > 0) {
            String ssid = cmd.substring(10, firstColon);
            String pass = cmd.substring(firstColon + 1);
            ConfigManager::getInstance().setWifiCredentials(ssid, pass);
            Serial.printf("[CLI] Wi-Fi credentials updated for SSID: %s\n", ssid.c_str());
            WiFi.disconnect(true);
            WiFi.begin(ssid.c_str(), pass.c_str());
        }
    } else if (cmd.startsWith("PROV:AUTH:")) {
        int firstColon = cmd.indexOf(':', 10);
        if (firstColon > 0) {
            String id = cmd.substring(10, firstColon);
            String key = cmd.substring(firstColon + 1);
            ConfigManager::getInstance().setDeviceCredentials(id, key);
            Serial.printf("[CLI] Device credentials updated: ID=%s\n", id.c_str());
        }
    } else if (cmd == "PROV:PORTAL") {
        WiFiManagerDriver::getInstance().startProvisioningPortal();
        UIManager::getInstance().setScreen(ActiveScreen::SCREEN_WIFI);
        Serial.println("[CLI] Web Provisioning Portal started. Connect to Wi-Fi 'ESP32-Robot-Setup' and browse to http://192.168.4.1");
    } else if (cmd == "PROV:STATUS") {
        Serial.println("\n=======================================================");
        Serial.println("             DEVICE PROVISIONING STATUS                ");
        Serial.println("=======================================================");
        Serial.printf("Firmware:      %s v%s\n", FIRMWARE_NAME, FIRMWARE_VERSION);
        Serial.printf("Wi-Fi SSID:    %s\n", ConfigManager::getInstance().getWifiSSID().c_str());
        Serial.printf("Wi-Fi Status:  %s\n", WiFiManagerDriver::getInstance().isConnected() ? "CONNECTED" : "DISCONNECTED");
        Serial.printf("IP Address:    %s (RSSI: %d dBm)\n", WiFiManagerDriver::getInstance().getIpAddress().c_str(),
                      WiFiManagerDriver::getInstance().getRSSI());
        Serial.printf("SNTP Synced:   %s\n", WiFiManagerDriver::getInstance().isSntpSynced() ? "YES" : "NO");
        Serial.printf("Device ID:     %s\n", ConfigManager::getInstance().getDeviceId().c_str());
        Serial.printf("API Key:       %s\n", ConfigManager::getInstance().getDeviceApiKey().length() ? "SET" : "MISSING");
        Serial.printf("WebSocket:     %s\n", WebSocketClientManager::getInstance().isAuthenticated() ? "ONLINE" : "OFFLINE");
        Serial.printf("SRAM Free:     %lu KB\n", (unsigned long)(ESP.getFreeHeap() / 1024));
        Serial.printf("PSRAM Free:    %lu / %lu KB\n", (unsigned long)(ESP.getFreePsram() / 1024),
                      (unsigned long)(ESP.getPsramSize() / 1024));
        Serial.println("=======================================================\n");
    } else if (cmd == "PROV:CLEAR") {
        ConfigManager::getInstance().clearAll();
        Serial.println("[CLI] NVS credentials cleared.");
    } else if (cmd == "PROV:REBOOT") {
        Serial.println("[CLI] Rebooting ESP32...");
        delay(300);
        ESP.restart();
    } else if (cmd == "SCREEN:TOGGLE") {
        UIManager::getInstance().setScreen(
            UIManager::getInstance().getScreen() == ActiveScreen::SCREEN_FACE ? ActiveScreen::SCREEN_MENU : ActiveScreen::SCREEN_FACE
        );
        Serial.println("[CLI] Screen toggled.");
    }
}

void processSerialCli() {
    static String buffer = "";
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r' || c == '\n') {
            buffer.trim();
            if (buffer.length() > 0) {
                executeCliCommand(buffer);
                buffer = "";
            }
        } else {
            if (buffer.length() < 256) buffer += c;
        }
    }
}

// ============================================================================
// SECTION 16: ARDUINO SETUP & LOOP
// ============================================================================
// Production Brownout Protection: Hardware detector is kept ENABLED and ACTIVE.
// Never suppress BOD comparator or brownout resets in production firmware.

void setup() {
    // Start diagnostics before touching peripherals. Serial is mirrored to UART0
    // and native USB CDC when USB CDC On Boot is enabled.
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n[BOOT] Memo voice build 2026-11-23-r14");

    // Comprehensive ESP-IDF Reset Reason Diagnostics
    esp_reset_reason_t rst = esp_reset_reason();
    const char* rstDesc = "UNKNOWN RESET";
    switch (rst) {
        case ESP_RST_POWERON:   rstDesc = "POWER-ON RESET"; break;
        case ESP_RST_EXT:       rstDesc = "EXTERNAL PIN RESET"; break;
        case ESP_RST_SW:        rstDesc = "SOFTWARE RESTART"; break;
        case ESP_RST_PANIC:     rstDesc = "PANIC / GURU MEDITATION / EXCEPTION"; break;
        case ESP_RST_INT_WDT:   rstDesc = "INTERRUPT WATCHDOG RESET"; break;
        case ESP_RST_TASK_WDT:  rstDesc = "TASK WATCHDOG RESET"; break;
        case ESP_RST_WDT:       rstDesc = "OTHER WATCHDOG RESET"; break;
        case ESP_RST_DEEPSLEEP: rstDesc = "DEEP SLEEP WAKE RESET"; break;
        case ESP_RST_BROWNOUT:  rstDesc = "BROWNOUT RESET (Voltage Sag Detected!)"; break;
        case ESP_RST_SDIO:      rstDesc = "SDIO RESET"; break;
        default:                rstDesc = "OTHER / UNKNOWN RESET"; break;
    }
    Serial.printf("[BOOT] Reset reason code: %d (%s) | CPU Freq: %u MHz\n", (int)rst, rstDesc, (unsigned)ESP.getCpuFreqMHz());
    Serial.println("[BOOT] Hardware Brownout Protection: ENABLED and ACTIVE.");
    if (rst == ESP_RST_BROWNOUT) {
        Serial.println("[WARNING] Previous reset was caused by BROWNOUT!");
        Serial.println("[WARNING] Check V2775 expansion board 9V power supply and verify 3.3V power rails under load.");
    }
    Serial.flush();

    // 1. CRITICAL: Immediately assert Touch CS (GPIO 8) HIGH and Touch IRQ INPUT_PULLUP
    // to prevent XPT2046 from driving the shared SPI bus while the ILI9341 display boots!
    pinMode(PIN_TOUCH_CS, OUTPUT);
    digitalWrite(PIN_TOUCH_CS, HIGH);
    pinMode(PIN_TOUCH_IRQ, INPUT_PULLUP);

    // 2. Drive Onboard WS2812 RGB LED (GPIO 48) LOW
    pinMode(PIN_RGB_LED, OUTPUT);
    digitalWrite(PIN_RGB_LED, LOW);

    Serial.println("[BOOT] Early GPIO initialization complete.");
    Serial.println("\n=======================================================");
    Serial.printf("  🤖 %s v%s\n", FIRMWARE_NAME, FIRMWARE_VERSION);
    Serial.printf("  Hardware: %s\n", HARDWARE_VERSION);
    Serial.println("=======================================================");

    // 3. Diagnostics: Flash & Octal PSRAM
    uint32_t flashSize = 0;
    esp_flash_get_size(NULL, &flashSize);
    Serial.printf("[Boot] Flash Memory: %lu MB (Mode: QIO 80MHz)\n", (unsigned long)(flashSize / (1024 * 1024)));

    if (psramInit()) {
        Serial.printf("[Boot] Octal PSRAM: DETECTED! Total: %lu KB, Free: %lu KB\n", (unsigned long)(ESP.getPsramSize() / 1024),
                      (unsigned long)(ESP.getFreePsram() / 1024));
    } else {
        Serial.println("[Boot] WARNING: Octal PSRAM initialization failed!");
    }

    // 4. Initialize Event Queue & NVS
    appEventQueue = xQueueCreate(32, sizeof(AppEvent));
    rfidDisplayQueue = xQueueCreate(1, sizeof(RfidDisplayResult));
    rfidLookupQueue = xQueueCreate(1, sizeof(AppEvent));
    ConfigManager::getInstance().begin();

    // 5. Initialize Display & Touch Calibration
    Serial.println("[Display] Initializing ILI9341 & XPT2046 with LovyanGFX...");
    tft.init();
    tft.setRotation(DISPLAY_ROTATION); // 320x240 Landscape mode
    tft.setBrightness((ConfigManager::getInstance().getBrightness() * 255) / 100);

    TouchCalibrationData touchCal = ConfigManager::getInstance().getTouchCal();
    if (touchCal.valid) {
        tft.setTouchCalibrate(touchCal.data);
        Serial.println("[Display] Loaded custom touch calibration from NVS.");
    } else {
        Serial.println("[Display] Using factory default touch calibration.");
    }

    // Hardware RGB diagnostic self-test
    Serial.println("[Display] Running hardware RGB diagnostic self-test...");
    tft.fillScreen(TFT_RED);
    delay(300);
    tft.fillScreen(TFT_GREEN);
    delay(300);
    tft.fillScreen(TFT_BLUE);
    delay(300);
    tft.fillScreen(TFT_BLACK);
    delay(100);
    Serial.println("[Display] Display self-test complete.");

    // 6. Initialize Audio & Sensor Subsystems
    AudioInputDriver::getInstance().begin();
    AudioInputDriver::getInstance().startRecording();

    // Initialize Local ESP-SR WakeNet9 Engine (Flash partition 'model' at 0xC10000)
    WakeWordEngine::getInstance().begin();
    if (!AudioOutputDriver::getInstance().begin()) {
        Serial.println("[Setup] CRITICAL ERROR: Audio Output Driver failed to initialize! Audio output will be offline.");
    }
    sensors.begin();
    eyes.begin();

    // 7. Initialize Wi-Fi Manager & SNTP
    WiFiManagerDriver::getInstance().begin();

    // 8. Initialize Cloud WebSocket Manager
    WebSocketClientManager::getInstance().begin();

    // 9. Launch FreeRTOS Tasks. Report each failure instead of silently staying offline.
    BaseType_t netTaskResult = xTaskCreatePinnedToCore(taskNetwork, "net_task", 16384, NULL, 1, &taskNetworkHandle, 0);
    BaseType_t sensorTaskResult = xTaskCreatePinnedToCore(taskSensors, "sensor_task", 4096, NULL, 1, &taskSensorHandle, 1);
    BaseType_t audioTaskResult = xTaskCreatePinnedToCore(taskAudio, "audio_task", 10240, NULL, 2, &taskAudioHandle, 1);
    BaseType_t captureTaskResult = xTaskCreatePinnedToCore(taskAudioCapture, "audio_cap", 16384, NULL, 2, &taskAudioCaptureHandle, 0);

    Serial.printf("[BOOT] Task creation: network=%s sensors=%s audio=%s capture=%s\n",
                  netTaskResult == pdPASS ? "OK" : "FAILED",
                  sensorTaskResult == pdPASS ? "OK" : "FAILED",
                  audioTaskResult == pdPASS ? "OK" : "FAILED",
                  captureTaskResult == pdPASS ? "OK" : "FAILED");
    Serial.println("[Boot] Memo AI Robot firmware initialized successfully!");
    Serial.println("Commands: 'PROV:STATUS', 'PROV:PORTAL', 'SCREEN:TOGGLE'");
}

void loop() {
    // 1. Process Serial CLI Provisioning Commands
    processSerialCli();

    // 2. Update Device Manager State Machine
    DeviceManager::getInstance().update();

    // 3. Process Touchscreen Input (Calibrated LovyanGFX coordinates)
    int32_t touchX = 0, touchY = 0;
    static bool touchWasPressed = false;
    if (tft.getTouch(&touchX, &touchY)) {
        if (!touchWasPressed) {
            touchWasPressed = true;
            UIManager::getInstance().handleTouchClick(touchX, touchY, DeviceManager::getInstance().getState());
        }
    } else {
        touchWasPressed = false;
    }

    // 4. Update UI & Animated Eyes Engine (30 FPS Target)
    static unsigned long lastUiFrame = 0;
    unsigned long now = millis();
    if (now - lastUiFrame >= 33) {
        lastUiFrame = now;
        bool wifiOk = WiFiManagerDriver::getInstance().isConnected();
        int rssi = WiFiManagerDriver::getInstance().getRSSI();
        bool wsOk = WebSocketClientManager::getInstance().isAuthenticated();
        uint8_t vol = ConfigManager::getInstance().getVolume();

        UIManager::getInstance().update(DeviceManager::getInstance().getState(), wifiOk, rssi, wsOk, vol);
    }

    // 5. Periodic Audio Pipeline & Memory Diagnostics (every 10 seconds)
    static unsigned long lastDiagLogTime = 0;
    if (now - lastDiagLogTime >= 10000) {
        lastDiagLogTime = now;
        AudioOutputDriver::getInstance().printDiagnostics(taskAudioHandle);
    }

    vTaskDelay(pdMS_TO_TICKS(5));
}
