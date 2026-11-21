import os

source_path = r'C:\Users\USER\Desktop\Memo_Arduino\final_deadline_candidate\final_deadline_candidate.ino'
dest_dir1 = r'C:\Users\USER\Desktop\Memo_Arduino\MEMO_ESP32_FINAL'
dest_dir2 = r'C:\Users\USER\Desktop\MEMO_ESP32_FINAL'

os.makedirs(dest_dir1, exist_ok=True)
os.makedirs(dest_dir2, exist_ok=True)

target_file1 = os.path.join(dest_dir1, 'MEMO_ESP32_FINAL.ino')
target_file2 = os.path.join(dest_dir2, 'MEMO_ESP32_FINAL.ino')

with open(source_path, 'r', encoding='utf-8') as f:
    code = f.read()

# 1. Update title block and includes
header_end_idx = code.find('#include <Arduino.h>')
assert header_end_idx > 0, "include Arduino.h not found"

new_header = '''/**
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

'''

code = new_header + code[header_end_idx:]

# 2. Update Configuration Section (ENABLE_ESP_SR_WAKENET 1)
old_wakenet_def = '''// WakeNet9 Status: Explicitly marked unsupported in standard Arduino IDE build
// (Voice assistant activation is handled via TTP223 head touch and Touchscreen UI)
#define ENABLE_ESP_SR_WAKENET    0'''

new_wakenet_def = '''// ============================================================================
// ESP-SR WakeNet9 Local Wake-Word Engine Configuration
// ============================================================================
// Built-in official test model in flash partition: "wn9_hiesp" ("Hi ESP")
// Final production target: "Hey Memo" (requires custom-trained model artifact "wn9_heymemo")
#define ENABLE_ESP_SR_WAKENET      1
#define WAKE_WORD_MODEL_NAME       "wn9_hiesp"
#define WAKE_WORD_PHRASE           "Hi ESP (Temp Test - Target: Hey Memo)"
#define WAKE_DETECTION_COOLDOWN_MS 3000'''

assert old_wakenet_def in code, "old_wakenet_def not found"
code = code.replace(old_wakenet_def, new_wakenet_def)

# 3. Add WakeNetStatus and WakeWordEngine before SECTION 9
sec8_end_marker = '''        return samples;
    }
};

// ============================================================================
// SECTION 9: WI-FI MANAGER & WEB CAPTIVE PROVISIONING PORTAL'''

wakenet_engine_code = '''        return samples;
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
    bool activeListening = false;
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
            Serial.printf("[WakeWord] Model partition loaded: %s\\n", activeModelName);
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
                wakenetIface->set_det_threshold(modelData, 0.65f, 1);

                modelLoaded = true;
                status = WakeNetStatus::READY_ACTIVE_LISTENING;
                activeListening = true;
                Serial.printf("[WakeWord] Model: %s (\\\"%s\\\") -> Wake-Word Detection READY.\\n", activeModelName, activeWakeWord);
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
        activeListening = listening;
        accumCount = 0;
        if (listening) {
            Serial.println("[WakeWord] State: IDLE_LISTEN");
        }
    }

    bool isListening() const { return activeListening; }
    bool isModelLoaded() const { return modelLoaded; }
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

    bool processPcmChunk(const int16_t* pcmSamples, size_t sampleCount) {
        if (!activeListening || !modelLoaded || !modelData || !wakenetIface || !pcmSamples || sampleCount == 0) {
            return false;
        }

        // 1. Maintain rolling circular pre-roll buffer
        for (size_t i = 0; i < sampleCount; i++) {
            preRollBuffer[preRollHead] = pcmSamples[i];
            preRollHead = (preRollHead + 1) % PRE_ROLL_SAMPLES;
        }

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
                if (result == WAKENET_DETECTED && (now - lastDetectionTime > WAKE_DETECTION_COOLDOWN_MS)) {
                    lastDetectionTime = now;
                    totalDetections++;
                    validPreRoll = true;
                    activeListening = false;
                    status = WakeNetStatus::WAKE_DETECTED;

                    Serial.printf("[WakeWord] DETECTED: \\\"%s\\\"! Launching Gemini Voice Session...\\n", activeWakeWord);
                    postEvent(EventType::VOICE_SESSION_START);
                    return true;
                }
            }
        }
        return false;
    }
};

// ============================================================================
// SECTION 9: WI-FI MANAGER & WEB CAPTIVE PROVISIONING PORTAL'''

assert sec8_end_marker in code, "sec8_end_marker not found"
code = code.replace(sec8_end_marker, wakenet_engine_code)

# 4. Update checkAndClearReadyAck / checkAndClearFlowPause in AudioOutputDriver & ADD isPlaying()
old_ack_methods = '''    bool checkAndClearReadyAck() {
        return sendReadyAckPending.exchange(false, std::memory_order_acq_rel);
    }

    bool checkAndClearFlowPause() {
        return flowControlPauseNeeded.exchange(false, std::memory_order_acq_rel);
    }

    bool checkAndClearFlowResume() {
        return flowControlResumeNeeded.exchange(false, std::memory_order_acq_rel);
    }'''

new_ack_methods = '''    bool isPlaying() const {
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
    }'''

assert old_ack_methods in code, "old_ack_methods not found"
code = code.replace(old_ack_methods, new_ack_methods)

# 5. Update WebSocketClientManager sendFlowControl to include sessionId and generation
old_send_flow = '''    bool sendFlowControl(const char* status) {
        JsonDocument doc;
        doc["event"] = "audio:flow";
        JsonObject data = doc["data"].to<JsonObject>();
        data["status"] = status;
        data["freeBytes"] = AudioOutputDriver::getInstance().getFreeBufferBytes();

        String out;
        serializeJson(doc, out);
        if (xSemaphoreTake(wsMutex, pdMS_TO_TICKS(50))) {
            bool ok = (client.available() && authenticated) ? client.send(out) : false;
            xSemaphoreGive(wsMutex);
            Serial.printf("[WebSocket] Watermark Flow Control: Sent audio:flow %s (free: %u B)\\n",
                          status, (unsigned)AudioOutputDriver::getInstance().getFreeBufferBytes());
            return ok;
        }
        return false;
    }'''

new_send_flow = '''    bool sendFlowControl(const char* status, const char* sessId, uint32_t gen) {
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
            bool ok = (client.available() && authenticated) ? client.send(out) : false;
            xSemaphoreGive(wsMutex);
            if (ok) {
                Serial.printf("[WebSocket] Watermark Flow Control: Sent audio:flow %s (gen: %lu, free: %u B)\\n",
                              status, (unsigned long)gen, (unsigned)AudioOutputDriver::getInstance().getFreeBufferBytes());
            }
            return ok;
        }
        return false;
    }'''

assert old_send_flow in code, "old_send_flow not found"
code = code.replace(old_send_flow, new_send_flow)

# 6. Update WebSocketClientManager::update() to use confirmed send before clearing flags
old_ws_flow_check = '''        // Handshake: Send audio:ready acknowledgement as soon as taskAudio finishes buffer flush
        if (wsConnected && authenticated) {
            if (AudioOutputDriver::getInstance().checkAndClearReadyAck()) {
                sendAudioReadyAck(AudioOutputDriver::getInstance().getActiveSessionId(),
                                  AudioOutputDriver::getInstance().getActiveGeneration());
            }
            if (AudioOutputDriver::getInstance().checkAndClearFlowPause()) {
                sendFlowControl("pause");
            }
            if (AudioOutputDriver::getInstance().checkAndClearFlowResume()) {
                sendFlowControl("resume");
            }
        }'''

new_ws_flow_check = '''        // Handshake & Flow Control: Only clear flags upon confirmed send success (Critical Fix A & B)
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
        }'''

assert old_ws_flow_check in code, "old_ws_flow_check not found"
code = code.replace(old_ws_flow_check, new_ws_flow_check)

# 7. Update taskAudioCapture to support WakeNet Standby & Pre-Roll Delivery
old_task_capture = '''// Background Audio Input Capture Task (Core 1, Priority 3, dedicated DMA streamer)
static TaskHandle_t taskAudioCaptureHandle = NULL;
void taskAudioCapture(void* parameter) {
    Serial.println("[FreeRTOS] Audio Capture Task running on Core 0.");
    static int16_t pcmBuffer[AUDIO_CHUNK_SIZE / 2]; // Static allocation prevents stack overflow!

    for (;;) {
        if (DeviceManager::getInstance().getState() == RobotState::LISTENING && 
            AudioInputDriver::getInstance().isRecording()) {
            
            size_t readSamples = AudioInputDriver::getInstance().readPcmChunk(pcmBuffer, AUDIO_CHUNK_SIZE / 2);
            if (readSamples > 0) {
                bool sent = WebSocketClientManager::getInstance().sendAudioChunk((const uint8_t*)pcmBuffer, readSamples * sizeof(int16_t));
                if (sent) {
                    DeviceManager::getInstance().incrementAudioChunkCount();
                }
            }

            // Utterance timeout (4.0s auto-stop if not tapped to end)
            if (millis() - DeviceManager::getInstance().getListeningStartTime() > 4000) {
                Serial.println("[DeviceManager] Voice capture auto-complete (4.0s utterance reached). Posting end event.");
                postEvent(EventType::VOICE_SESSION_END);
            }
            vTaskDelay(pdMS_TO_TICKS(2)); // Feed watchdog and yield to lower priority tasks on Core 1
        } else {
            vTaskDelay(pdMS_TO_TICKS(20)); // Yield to other tasks when not actively recording
        }
    }
}'''

new_task_capture = '''// Background Audio Input Capture & WakeNet Task (Core 0, Priority 2, dedicated DMA & neural inference)
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

        if (curState == RobotState::LISTENING && AudioInputDriver::getInstance().isRecording()) {
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

            // Utterance timeout (4.0s auto-stop if not tapped to end)
            if (millis() - DeviceManager::getInstance().getListeningStartTime() > 4000) {
                Serial.println("[DeviceManager] Voice capture auto-complete (4.0s utterance reached). Posting end event.");
                postEvent(EventType::VOICE_SESSION_END);
            }
        } else if (curState == RobotState::IDLE) {
            // 2. STANDBY LOCAL WAKE-WORD DETECTION:
            // Zero microphone data is sent over the network during standby (privacy preserved).
            // Prevent speaker echo: pause WakeNet if speaker is actively playing audio!
            if (!AudioOutputDriver::getInstance().isPlaying()) {
                WakeWordEngine::getInstance().processPcmChunk(pcmBuffer, readSamples);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}'''

assert old_task_capture in code, "old_task_capture not found"
code = code.replace(old_task_capture, new_task_capture)

# 8. Update DeviceManager::update() to re-arm WakeWordEngine when returning to IDLE
old_dev_state_check = '''        if (currentState == RobotState::THINKING) {
            // Thinking timeout (15 seconds)
            if (millis() - thinkingStartTime > 15000) {
                Serial.println("[DeviceManager] Response timeout. Returning to IDLE.");
                setState(RobotState::IDLE);
            }
        }'''

new_dev_state_check = '''        if (currentState == RobotState::THINKING) {
            // Thinking timeout (15 seconds)
            if (millis() - thinkingStartTime > 15000) {
                Serial.println("[DeviceManager] Response timeout. Returning to IDLE.");
                setState(RobotState::IDLE);
                WakeWordEngine::getInstance().setListening(true);
            }
        }'''

assert old_dev_state_check in code, "old_dev_state_check not found"
code = code.replace(old_dev_state_check, new_dev_state_check)

# 9. Update setup() to initialize WakeWordEngine
old_setup_wake = '''    // 6. Initialize Audio & Sensor Subsystems
    AudioInputDriver::getInstance().begin();'''

new_setup_wake = '''    // 6. Initialize Audio & Sensor Subsystems
    AudioInputDriver::getInstance().begin();
    AudioInputDriver::getInstance().startRecording();

    // Initialize Local ESP-SR WakeNet9 Engine (Flash partition 'model' at 0xC10000)
    WakeWordEngine::getInstance().begin();'''

assert old_setup_wake in code, "old_setup_wake not found"
code = code.replace(old_setup_wake, new_setup_wake)

# 10. Write the output to both destinations
with open(target_file1, 'w', encoding='utf-8') as f:
    f.write(code)

with open(target_file2, 'w', encoding='utf-8') as f:
    f.write(code)

print("Successfully generated MEMO_ESP32_FINAL.ino in both destinations with isPlaying() method!")
print("Destination 1:", target_file1)
print("Destination 2:", target_file2)
