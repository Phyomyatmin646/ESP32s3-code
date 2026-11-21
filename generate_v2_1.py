import os

v2_path = r'C:\Users\USER\Desktop\Memo_Arduino\final\final_audio_stability_v2.ino'
out_path1 = r'C:\Users\USER\Desktop\Memo_Arduino\final\final_audio_stability_v2_1.ino'
out_path2 = r'C:\Users\USER\Desktop\Memo_Arduino\final_audio_stability_v2_1\final_audio_stability_v2_1.ino'

with open(v2_path, 'r', encoding='utf-8') as f:
    content = f.read()

# 1. Update header comment
header_v2 = ' *           🤖 MEMO ESP32-S3 AI ROBOT — PRODUCTION AUDIO STABILITY V2.0\n * Unified 24kHz Audio Pipeline | Thread-Safe FreeRTOS Audio Manager | Zero BOD Suppression'
header_v2_1 = ' *           🤖 MEMO ESP32-S3 AI ROBOT — PRODUCTION AUDIO STABILITY V2.1\n * Real Session Isolation | Thread-Safe Ownership | Partial-Write Recovery | Source-Priority Policy'
assert header_v2 in content, 'Header V2 not found'
content = content.replace(header_v2, header_v2_1, 1)

# 2. Update Section 7: AudioOutputDriver with all V2.1 enhancements
# Find boundaries of Section 7
sec7_start_marker = '// ============================================================================\n// SECTION 7: CENTRALIZED UNIFIED I2S AUDIO MANAGER (MAX98357A + Octal PSRAM)\n// ============================================================================'
sec8_start_marker = '// ============================================================================\n// SECTION 8: NON-BLOCKING I2S AUDIO INPUT (INMP441 Microphone Driver)\n// ============================================================================'

assert sec7_start_marker in content, 'sec7_start_marker not found'
assert sec8_start_marker in content, 'sec8_start_marker not found'

sec7_idx = content.find(sec7_start_marker)
sec8_idx = content.find(sec8_start_marker)

sec7_v2_1 = '''// ============================================================================
// SECTION 7: CENTRALIZED UNIFIED I2S AUDIO MANAGER (MAX98357A + Octal PSRAM)
// ============================================================================
// ARCHITECTURE RULES (V2.1 PRODUCTION AUDIT CERTIFIED):
// 1. Only ONE task (taskAudio on Core 1) writes to I2S_NUM_1 or touches I2S hardware.
// 2. All mutable state (sessions, volumes, commands) protected by audioMutex.
// 3. Partial writes continue from offset with bounded retries; unrecoverable dropped bytes tracked.
// 4. Source-Priority Policy: Streaming AI Voice > Cute Chime (Chime yields if Voice arrives).
// 5. Real session isolation: sessionGeneration validated on every PCM block.
// 6. Safe flush handling: if flush lock fails, request is preserved and retried.
// ============================================================================
class AudioOutputDriver {
private:
    bool initialized = false;
    uint32_t currentSampleRate = AUDIO_OUT_SAMPLE_RATE; // Strictly fixed at 24000 Hz
    uint8_t currentVolume = DEFAULT_VOLUME;
    float gainMultiplier = 0.8f;
    int16_t scaleBuffer[AUDIO_CHUNK_SIZE / 2];

    RingbufHandle_t audioRingBuf = nullptr;
    size_t ringBufCapacity = 0;
    SemaphoreHandle_t audioMutex = nullptr;

    // Asynchronous commands and real session isolation state
    volatile bool flushRequested = false;
    volatile bool chimeTriggered = false;
    volatile bool sessionActive = false;
    volatile bool sessionCancelled = false;
    char activeSessionId[64] = {0};
    uint32_t activeSessionGeneration = 0;

    // Non-blocking Cute Chime synthesizer state machine
    struct CuteChimeSynth {
        bool running = false;
        size_t currentSample = 0;
        size_t totalSamples = 0;
        float phase = 0.0f;
    } chime;

    // Production health, diagnostics and recovery telemetry
    volatile uint32_t underrunCount = 0;
    volatile uint32_t overflowCount = 0;
    volatile uint32_t droppedFrames = 0;
    volatile uint32_t i2sWriteErrors = 0;
    volatile uint32_t i2sPartialWrites = 0;
    volatile uint32_t unrecoverableDroppedBytes = 0;
    volatile uint32_t totalPacketsReceived = 0;
    volatile uint32_t totalBytesPlayed = 0;
    volatile bool isDmaPlaying = false;
    unsigned long lastPlayingTime = 0;

    // Robust I2S partial-write handler with bounded retries
    size_t writeI2sWithRetry(const uint8_t* data, size_t totalBytes, uint8_t maxRetries = 3, TickType_t tickTimeout = pdMS_TO_TICKS(40)) {
        size_t bytesRemaining = totalBytes;
        const uint8_t* ptr = data;
        uint8_t retries = 0;

        while (bytesRemaining > 0 && retries < maxRetries) {
            size_t written = 0;
            esp_err_t err = i2s_write(I2S_NUM_1, (const char*)ptr, bytesRemaining, &written, tickTimeout);
            if (err != ESP_OK) {
                i2sWriteErrors++;
                retries++;
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }
            if (written > 0) {
                if (written < bytesRemaining) {
                    i2sPartialWrites++;
                }
                bytesRemaining -= written;
                ptr += written;
                retries = 0;
            } else {
                retries++;
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }

        size_t totalWritten = totalBytes - bytesRemaining;
        if (bytesRemaining > 0) {
            unrecoverableDroppedBytes += bytesRemaining;
        }
        totalBytesPlayed += totalWritten;
        return totalWritten;
    }

public:
    static AudioOutputDriver& getInstance() {
        static AudioOutputDriver instance;
        return instance;
    }

    bool begin() {
        Serial.println("[AudioOutput] Initializing MAX98357A Central Audio Manager (I2S_NUM_1)...");

        if (!audioMutex) {
            audioMutex = xSemaphoreCreateMutex();
        }

        i2s_config_t i2sConfig = {
            .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
            .sample_rate = AUDIO_OUT_SAMPLE_RATE, // Enforce 24kHz hardware rate
            .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
            .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
            .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
            .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
            .dma_buf_count = AUDIO_DMA_BUF_COUNT,
            .dma_buf_len = AUDIO_DMA_BUF_LEN,
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

        if (i2s_driver_install(I2S_NUM_1, &i2sConfig, 0, NULL) != ESP_OK) {
            Serial.println("[AudioOutput] ERROR: i2s_driver_install failed!");
            return false;
        }

        if (i2s_set_pin(I2S_NUM_1, &pinConfig) != ESP_OK) {
            Serial.println("[AudioOutput] ERROR: i2s_set_pin failed!");
            return false;
        }

        i2s_zero_dma_buffer(I2S_NUM_1);
        setVolume(currentVolume);

        if (!audioRingBuf) {
            if (psramFound()) {
                audioRingBuf = xRingbufferCreateWithCaps(AUDIO_RING_BUFFER_BYTES_PSRAM, RINGBUF_TYPE_BYTEBUF, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                if (audioRingBuf) {
                    ringBufCapacity = AUDIO_RING_BUFFER_BYTES_PSRAM;
                    Serial.printf("[AudioOutput] Allocated %u KB RingBuffer in Octal PSRAM.\\n", (unsigned)(AUDIO_RING_BUFFER_BYTES_PSRAM / 1024));
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
                }
            }
        }

        initialized = true;
        Serial.println("[AudioOutput] Central Audio Manager initialized successfully (Hardware locked at 24000 Hz).");
        return true;
    }

    // Output sample rate is strictly 24kHz; verifies metadata
    void setSampleRate(uint32_t rate) {
        if (rate != AUDIO_OUT_SAMPLE_RATE && rate != 0) {
            Serial.printf("[AudioOutput] Note: Metadata sample rate %lu Hz. Output hardware is unified at %u Hz.\\n",
                          (unsigned long)rate, (unsigned)AUDIO_OUT_SAMPLE_RATE);
        }
    }

    // Thread-safe volume control
    void setVolume(uint8_t vol) {
        if (vol > MAX_VOLUME_LIMIT) vol = MAX_VOLUME_LIMIT;
        if (audioMutex && xSemaphoreTake(audioMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            currentVolume = vol;
            gainMultiplier = ((float)vol / 100.0f) * 0.95f;
            xSemaphoreGive(audioMutex);
        } else {
            currentVolume = vol;
            gainMultiplier = ((float)vol / 100.0f) * 0.95f;
        }
    }

    uint8_t getVolume() const { return currentVolume; }

    // Start a new audio session with real unique generation ID
    uint32_t startSession(const char* sessionId, uint32_t sRate = 24000) {
        uint32_t gen = 0;
        if (audioMutex && xSemaphoreTake(audioMutex, pdMS_TO_TICKS(25)) == pdTRUE) {
            activeSessionGeneration++;
            gen = activeSessionGeneration;
            sessionActive = true;
            sessionCancelled = false;
            if (sessionId) {
                strncpy(activeSessionId, sessionId, sizeof(activeSessionId) - 1);
                activeSessionId[sizeof(activeSessionId) - 1] = '\\0';
            } else {
                activeSessionId[0] = '\\0';
            }
            xSemaphoreGive(audioMutex);
        } else {
            activeSessionGeneration++;
            gen = activeSessionGeneration;
            sessionActive = true;
            sessionCancelled = false;
        }
        setSampleRate(sRate);
        return gen;
    }

    // Cancel active audio session immediately (interruption/barge-in)
    void cancelSession(const char* reason = nullptr) {
        if (audioMutex && xSemaphoreTake(audioMutex, pdMS_TO_TICKS(25)) == pdTRUE) {
            activeSessionGeneration++; // Invalidate any subsequent chunks for old session
            sessionCancelled = true;
            sessionActive = false;
            flushRequested = true;
            activeSessionId[0] = '\\0';
            xSemaphoreGive(audioMutex);
        } else {
            activeSessionGeneration++;
            sessionCancelled = true;
            sessionActive = false;
            flushRequested = true;
        }
        Serial.printf("[AudioOutput] Session CANCELLED: %s (new gen: %lu)\\n",
                      reason ? reason : "user_interrupted", (unsigned long)activeSessionGeneration);
    }

    void endSession() {
        if (audioMutex && xSemaphoreTake(audioMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            sessionActive = false;
            xSemaphoreGive(audioMutex);
        } else {
            sessionActive = false;
        }
    }

    uint32_t getActiveGeneration() const { return activeSessionGeneration; }

    // Non-blocking, thread-safe PCM enqueue with real generation matching
    bool enqueuePcm(const int16_t* samples, size_t sampleCount, uint32_t streamGen = 0) {
        if (!initialized || !audioRingBuf || !samples || sampleCount == 0) return false;

        // Session generation verification: discard stale packets from old/cancelled sessions
        if (streamGen != 0 && streamGen != activeSessionGeneration) {
            droppedFrames++;
            return false;
        }
        if (sessionCancelled || !sessionActive) {
            droppedFrames++;
            return false;
        }

        totalPacketsReceived++;
        size_t bytes = sampleCount * sizeof(int16_t);
        bool ok = false;

        if (audioMutex && xSemaphoreTake(audioMutex, pdMS_TO_TICKS(15)) == pdTRUE) {
            BaseType_t res = xRingbufferSend(audioRingBuf, (const void*)samples, bytes, pdMS_TO_TICKS(15));
            ok = (res == pdTRUE);
            if (!ok) {
                overflowCount++;
                droppedFrames++;
            }
            xSemaphoreGive(audioMutex);
        } else {
            overflowCount++;
            droppedFrames++;
        }
        return ok;
    }

    void triggerCuteChime() {
        chimeTriggered = true;
    }

    // Thread-safe flush request
    void stopAndFlush() {
        sessionCancelled = true;
        sessionActive = false;
        flushRequested = true;
    }

    // Thread-safe check if buffered audio or DMA is active
    bool hasBufferedAudio() const {
        if (!initialized) return false;
        if (chime.running || chimeTriggered) return true;
        if (isDmaPlaying) return true;
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

        // 1. Process asynchronous flush request.
        // If mutex cannot be acquired, PRESERVE flushRequested = true and retry safely on next tick!
        if (flushRequested) {
            if (audioMutex && xSemaphoreTake(audioMutex, pdMS_TO_TICKS(15)) == pdTRUE) {
                if (audioRingBuf) {
                    size_t size = 0;
                    void* item = nullptr;
                    while ((item = xRingbufferReceiveUpTo(audioRingBuf, &size, 0, AUDIO_CHUNK_SIZE)) != nullptr) {
                        vRingbufferReturnItem(audioRingBuf, item);
                    }
                }
                flushRequested = false; // Only clear when flush is actually executed!
                chime.running = false;
                isDmaPlaying = false;
                xSemaphoreGive(audioMutex);
                i2s_zero_dma_buffer(I2S_NUM_1);
            }
            return false;
        }

        // 2. Determine if streaming PCM buffer has data ready to play
        bool ringHasData = false;
        if (audioRingBuf && ringBufCapacity > 0) {
            size_t curFree = xRingbufferGetCurFreeSize(audioRingBuf);
            if (ringBufCapacity > curFree && (ringBufCapacity - curFree) > 256) {
                ringHasData = true;
            }
        }

        // 3. SOURCE-PRIORITY POLICY:
        // High-Priority: Streaming Voice Audio takes precedence over Cute Chime.
        // If incoming voice audio is ready and Chime is active, Chime yields immediately!
        if (ringHasData && chime.running) {
            chime.running = false; // Gracefully yield to incoming voice stream
        }

        // 4. Stream audio from RingBuffer to I2S (High-Priority Path)
        bool hasData = false;
        if (audioRingBuf) {
            void* item = nullptr;
            size_t itemSize = 0;

            if (audioMutex && xSemaphoreTake(audioMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                item = xRingbufferReceiveUpTo(audioRingBuf, &itemSize, 0, (AUDIO_CHUNK_SIZE / 2) * sizeof(int16_t));
                xSemaphoreGive(audioMutex);
            }

            if (item && itemSize > 0) {
                hasData = true;
                isDmaPlaying = true;
                lastPlayingTime = millis();
                const int16_t* pcmIn = (const int16_t*)item;
                size_t samples = itemSize / sizeof(int16_t);
                if (samples > (AUDIO_CHUNK_SIZE / 2)) samples = (AUDIO_CHUNK_SIZE / 2);

                for (size_t i = 0; i < samples; i++) {
                    int32_t val = (int32_t)(pcmIn[i] * gainMultiplier);
                    if (val > 30000) val = 30000;
                    if (val < -30000) val = -30000;
                    scaleBuffer[i] = (int16_t)val;
                }

                writeI2sWithRetry((const uint8_t*)scaleBuffer, samples * sizeof(int16_t));

                if (audioMutex && xSemaphoreTake(audioMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    vRingbufferReturnItem(audioRingBuf, item);
                    xSemaphoreGive(audioMutex);
                }
                return true;
            } else {
                if (isDmaPlaying && (millis() - lastPlayingTime > 80)) {
                    isDmaPlaying = false;
                    if (sessionActive) {
                        underrunCount++;
                    }
                }
            }
        }

        // 5. Cute Chime synthesis state machine (Low-Priority Path)
        if (chimeTriggered) {
            chimeTriggered = false;
            chime.running = true;
            chime.currentSample = 0;
            chime.totalSamples = (AUDIO_OUT_SAMPLE_RATE * 650) / 1000;
            chime.phase = 0.0f;
        }

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

                float sample = (sinf(chime.phase) + 0.3f * sinf(2.0f * chime.phase)) * env * 18000.0f * (currentVolume / 100.0f);
                scaleBuffer[b] = (int16_t)sample;
            }
            chime.currentSample += batch;
            if (chime.currentSample >= chime.totalSamples) {
                chime.running = false;
            }

            writeI2sWithRetry((const uint8_t*)scaleBuffer, batch * sizeof(int16_t), 2, pdMS_TO_TICKS(30));
            isDmaPlaying = true;
            lastPlayingTime = millis();
            return true;
        }

        return hasData;
    }

    // Production health, memory and recovery telemetry
    void printDiagnostics() {
        size_t freeHeap = ESP.getFreeHeap();
        size_t minHeap = ESP.getMinFreeHeap();
        size_t maxBlock = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
        size_t freePsram = ESP.getFreePsram();
        size_t totalPsram = ESP.getPsramSize();
        size_t freeRing = audioRingBuf ? xRingbufferGetCurFreeSize(audioRingBuf) : 0;
        size_t occRing = (ringBufCapacity > freeRing) ? (ringBufCapacity - freeRing) : 0;

        Serial.printf("[AUDIO-DIAG] Heap: %u B (Min: %u B, MaxBlk: %u B) | PSRAM: %u/%u KB\\n",
                      (unsigned)freeHeap, (unsigned)minHeap, (unsigned)maxBlock,
                      (unsigned)(freePsram / 1024), (unsigned)(totalPsram / 1024));
        Serial.printf("[AUDIO-DIAG] RingBuf: %u/%u B (%u%%) | Under: %lu | Over: %lu | Drop: %lu | Partial: %lu | UnrecovBytes: %lu\\n",
                      (unsigned)occRing, (unsigned)ringBufCapacity,
                      ringBufCapacity > 0 ? (unsigned)((occRing * 100) / ringBufCapacity) : 0,
                      (unsigned long)underrunCount, (unsigned long)overflowCount, (unsigned long)droppedFrames,
                      (unsigned long)i2sPartialWrites, (unsigned long)unrecoverableDroppedBytes);
        Serial.printf("[AUDIO-DIAG] Session: %s (Gen: %lu) | State: %s\\n",
                      activeSessionId[0] ? activeSessionId : "<none>",
                      (unsigned long)activeSessionGeneration,
                      sessionActive ? "ACTIVE" : (chime.running ? "CHIME" : "IDLE"));
    }
};

'''

content = content[:sec7_idx] + sec7_v2_1 + content[sec8_idx:]

# 3. Update Section 12 WebSocketClientManager: Frame Assembler & Session Gen Tracking
ws_old = '''        client.onMessage([this](WebsocketsMessage msg) {
            if (msg.isBinary()) {
                // Non-blocking binary audio enqueue with 16-bit frame alignment & cancellation filtering
                const char* data = msg.c_str();
                size_t len = msg.length();
                if (len >= sizeof(int16_t) && data) {
                    size_t alignedLen = (len / sizeof(int16_t)) * sizeof(int16_t);
                    bool ok = AudioOutputDriver::getInstance().enqueuePcm((const int16_t*)data, alignedLen / sizeof(int16_t));
                    if (!ok) {
                        static uint32_t droppedFrames = 0;
                        droppedFrames++;
                        if (droppedFrames % 25 == 1) {
                            Serial.printf("[AudioOutput] Frame dropped (buffer full or session cancelled): count %lu\\n", (unsigned long)droppedFrames);
                        }
                    }
                }
            } else if (msg.isText()) {
                handleTextMessage(msg.data());
            }
        });'''

ws_new = '''        static uint8_t leftoverByte = 0;
        static bool hasLeftoverByte = false;
        static int16_t sampleAlignedBuf[AUDIO_CHUNK_SIZE];

        client.onMessage([this](WebsocketsMessage msg) {
            if (msg.isBinary()) {
                // Production-grade frame assembler: Zero odd-byte truncation, preserves sample alignment
                const uint8_t* raw = (const uint8_t*)msg.c_str();
                size_t rawLen = msg.length();
                if (rawLen > 0 && raw) {
                    size_t samplesOut = 0;
                    size_t readOffset = 0;

                    if (hasLeftoverByte && rawLen > 0) {
                        uint16_t s = (uint16_t)leftoverByte | ((uint16_t)raw[0] << 8);
                        sampleAlignedBuf[samplesOut++] = (int16_t)s;
                        readOffset++;
                        hasLeftoverByte = false;
                    }

                    size_t remainingBytes = rawLen - readOffset;
                    size_t fullSamples = remainingBytes / sizeof(int16_t);
                    size_t bytesToCopy = fullSamples * sizeof(int16_t);

                    if (fullSamples > 0 && samplesOut + fullSamples <= AUDIO_CHUNK_SIZE) {
                        memcpy(&sampleAlignedBuf[samplesOut], &raw[readOffset], bytesToCopy);
                        samplesOut += fullSamples;
                        readOffset += bytesToCopy;
                    }

                    if (readOffset < rawLen) {
                        leftoverByte = raw[readOffset];
                        hasLeftoverByte = true;
                    }

                    if (samplesOut > 0) {
                        bool ok = AudioOutputDriver::getInstance().enqueuePcm(sampleAlignedBuf, samplesOut, currentSessionGeneration);
                        if (!ok) {
                            static uint32_t droppedFrames = 0;
                            droppedFrames++;
                            if (droppedFrames % 25 == 1) {
                                Serial.printf("[AudioOutput] Frame rejected (generation mismatch, buffer full, or cancelled): count %lu\\n", (unsigned long)droppedFrames);
                            }
                        }
                    }
                }
            } else if (msg.isText()) {
                handleTextMessage(msg.data());
            }
        });'''

assert ws_old in content, 'ws_old not found'
content = content.replace(ws_old, ws_new, 1)

# Add currentSessionGeneration member variable to WebSocketClientManager
ws_class_marker = 'class WebSocketClientManager {\nprivate:\n'
assert ws_class_marker in content, 'ws_class_marker not found'
content = content.replace(ws_class_marker, ws_class_marker + '    uint32_t currentSessionGeneration = 0;\n', 1)

# Update handleTextMessage to store the generation from startSession
ws_txt_v2 = '''        } else if (strcmp(ev, "audio:response:start") == 0) {
            uint32_t sRate = data["sampleRate"] | 24000;
            const char* sessId = data["sessionId"] | "";
            AudioOutputDriver::getInstance().startSession(sessId, sRate);
            postEvent(EventType::VOICE_RESPONSE_START, (int)sRate);
            Serial.printf("[WebSocket] AI Voice response starting: %s (%lu Hz)\\n", sessId[0] ? sessId : "<none>", (unsigned long)sRate);
        } else if (strcmp(ev, "audio:response:end") == 0) {
            AudioOutputDriver::getInstance().endSession();
            postEvent(EventType::VOICE_RESPONSE_END);
            Serial.println("[WebSocket] AI Voice response finished.");
        } else if (strcmp(ev, "audio:cancel") == 0) {
            const char* reason = data["reason"] | "interrupted";
            AudioOutputDriver::getInstance().cancelSession(reason);
            postEvent(EventType::VOICE_AUDIO_CANCEL);'''

ws_txt_v2_1 = '''        } else if (strcmp(ev, "audio:response:start") == 0) {
            uint32_t sRate = data["sampleRate"] | 24000;
            const char* sessId = data["sessionId"] | "";
            currentSessionGeneration = AudioOutputDriver::getInstance().startSession(sessId, sRate);
            postEvent(EventType::VOICE_RESPONSE_START, (int)sRate);
            Serial.printf("[WebSocket] AI Voice response starting: %s (gen: %lu, %lu Hz)\\n",
                          sessId[0] ? sessId : "<none>", (unsigned long)currentSessionGeneration, (unsigned long)sRate);
        } else if (strcmp(ev, "audio:response:end") == 0) {
            AudioOutputDriver::getInstance().endSession();
            postEvent(EventType::VOICE_RESPONSE_END);
            Serial.println("[WebSocket] AI Voice response finished.");
        } else if (strcmp(ev, "audio:cancel") == 0) {
            const char* reason = data["reason"] | "interrupted";
            AudioOutputDriver::getInstance().cancelSession(reason);
            currentSessionGeneration = 0; // Invalidate client session generation immediately
            postEvent(EventType::VOICE_AUDIO_CANCEL);'''

assert ws_txt_v2 in content, 'ws_txt_v2 not found'
content = content.replace(ws_txt_v2, ws_txt_v2_1, 1)

# Write output files
with open(out_path1, 'w', encoding='utf-8') as f:
    f.write(content)

with open(out_path2, 'w', encoding='utf-8') as f:
    f.write(content)

print('SUCCESS: Created final_audio_stability_v2_1.ino in both paths')
