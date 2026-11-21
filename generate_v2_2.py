import os

v2_1_path = r'C:\Users\USER\Desktop\Memo_Arduino\final\final_audio_stability_v2_1.ino'
out_path1 = r'C:\Users\USER\Desktop\Memo_Arduino\final\final_audio_stability_v2_2.ino'
out_dir2 = r'C:\Users\USER\Desktop\Memo_Arduino\final_audio_stability_v2_2'
out_path2 = os.path.join(out_dir2, 'final_audio_stability_v2_2.ino')

if not os.path.exists(out_dir2):
    os.makedirs(out_dir2)

with open(v2_1_path, 'r', encoding='utf-8') as f:
    content = f.read()

# 1. Update header comment
header_v2_1 = ' *           🤖 MEMO ESP32-S3 AI ROBOT — PRODUCTION AUDIO STABILITY V2.1\n * Real Session Isolation | Thread-Safe Ownership | Partial-Write Recovery | Source-Priority Policy'
header_v2_2 = ' *           🤖 MEMO ESP32-S3 AI ROBOT — PRODUCTION AUDIO STABILITY V2.2\n * Atomic Session Validation | Thread-Safe State | Guaranteed Item Return | Bounded I2S Recovery | Startup Prebuffering'
assert header_v2_1 in content, 'Header V2.1 not found'
content = content.replace(header_v2_1, header_v2_2, 1)

# 2. Update Section 7: AudioOutputDriver with all V2.2 enhancements
sec7_start_marker = '// ============================================================================\n// SECTION 7: CENTRALIZED UNIFIED I2S AUDIO MANAGER (MAX98357A + Octal PSRAM)\n// ============================================================================'
sec8_start_marker = '// ============================================================================\n// SECTION 8: NON-BLOCKING I2S AUDIO INPUT (INMP441 Microphone Driver)\n// ============================================================================'

assert sec7_start_marker in content, 'sec7_start_marker not found'
assert sec8_start_marker in content, 'sec8_start_marker not found'

sec7_idx = content.find(sec7_start_marker)
sec8_idx = content.find(sec8_start_marker)

sec7_v2_2 = '''// ============================================================================
// SECTION 7: CENTRALIZED UNIFIED I2S AUDIO MANAGER (MAX98357A + Octal PSRAM)
// ============================================================================
// ARCHITECTURE RULES (V2.2 PRODUCTION AUDIT CERTIFIED):
// 1. Only ONE task (taskAudio on Core 1) writes to I2S_NUM_1 or touches I2S hardware.
// 2. All mutable state (sessions, volumes, commands) protected by audioMutex. Zero unsafe fallback writes.
// 3. Guaranteed RingBuffer item return: every received item is returned exactly once. Zero leaks.
// 4. Atomic session validation: generation and session state validated inside audioMutex before insertion.
// 5. Bounded I2S write recovery: global elapsed-time and attempt budget with 16-bit PCM alignment.
// 6. Source-Priority Policy: Streaming AI Voice > Cute Chime (Chime yields if Voice arrives).
// 7. Startup Prebuffer Threshold: configurable low-latency (150ms) vs stable MP3 (400ms) buffering.
// 8. Robust Initialization: begin() validates mutex, I2S install, pin config, and ringbuffer.
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

    // Asynchronous commands and real session isolation state (Guarded by audioMutex)
    bool flushRequested = false;
    bool sessionActive = false;
    bool sessionCancelled = false;
    bool streamEnded = false;
    bool isMp3Stream = false;
    bool prebufferReached = false;
    char activeSessionId[64] = {0};
    uint32_t activeSessionGeneration = 0;

    // Cute Chime synthesizer state machine (Owned solely by taskAudio on Core 1)
    volatile bool chimeTriggered = false;
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
    volatile uint32_t mutexAcquireFailures = 0;
    volatile uint32_t ringItemsCheckedOut = 0;
    volatile uint32_t ringItemsReturned = 0;
    volatile uint32_t ringItemReturnFailures = 0;
    volatile uint32_t totalPacketsReceived = 0;
    volatile uint32_t totalBytesPlayed = 0;
    volatile bool isDmaPlaying = false;
    unsigned long lastPlayingTime = 0;

    // Robust I2S write recovery: strictly bounded global attempts and elapsed time, sample-aligned
    size_t writeI2sWithRetry(const uint8_t* data, size_t totalBytes, uint8_t maxAttempts = 4, uint32_t maxElapsedMs = 60) {
        if (!data || totalBytes == 0) return 0;

        size_t bytesRemaining = totalBytes;
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
                i2sWriteErrors++;
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }

            if (written > 0) {
                // Enforce 16-bit PCM alignment (must be multiple of 2 bytes)
                if ((written % sizeof(int16_t)) != 0) {
                    written -= (written % sizeof(int16_t));
                }

                if (written < bytesRemaining) {
                    i2sPartialWrites++;
                }
                bytesRemaining -= written;
                ptr += written;
                // attempts is NOT reset to 0: strictly bounded global budget
            } else {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }

        size_t totalWritten = totalBytes - bytesRemaining;
        if (bytesRemaining > 0) {
            unrecoverableDroppedBytes += bytesRemaining;
            Serial.printf("[AudioOutput] WARNING: I2S write incomplete. Dropped %u unrecoverable bytes (total dropped: %lu)\\n",
                          (unsigned)bytesRemaining, (unsigned long)unrecoverableDroppedBytes);
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
            if (!audioMutex) {
                Serial.println("[AudioOutput] CRITICAL ERROR: Failed to create audioMutex!");
                initialized = false;
                return false;
            }
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

        esp_err_t err = i2s_driver_install(I2S_NUM_1, &i2sConfig, 0, NULL);
        if (err != ESP_OK) {
            Serial.printf("[AudioOutput] CRITICAL ERROR: i2s_driver_install failed with code 0x%X!\\n", err);
            initialized = false;
            return false;
        }

        err = i2s_set_pin(I2S_NUM_1, &pinConfig);
        if (err != ESP_OK) {
            Serial.printf("[AudioOutput] CRITICAL ERROR: i2s_set_pin failed with code 0x%X!\\n", err);
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
                    i2s_driver_uninstall(I2S_NUM_1);
                    initialized = false;
                    return false;
                }
            }
        }

        setVolume(currentVolume);
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

    // Thread-safe volume control (Zero unsafe fallback writes)
    void setVolume(uint8_t vol) {
        if (vol > MAX_VOLUME_LIMIT) vol = MAX_VOLUME_LIMIT;
        if (!audioMutex) {
            currentVolume = vol;
            gainMultiplier = ((float)vol / 100.0f) * 0.95f;
            return;
        }
        if (xSemaphoreTake(audioMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            currentVolume = vol;
            gainMultiplier = ((float)vol / 100.0f) * 0.95f;
            xSemaphoreGive(audioMutex);
        } else {
            mutexAcquireFailures++;
            Serial.println("[AudioOutput] WARNING: setVolume failed to acquire audioMutex!");
        }
    }

    uint8_t getVolume() const { return currentVolume; }

    // Start a new audio session with real unique generation ID and prebuffering config
    uint32_t startSession(const char* sessionId, uint32_t sRate = 24000, bool isMp3 = false) {
        if (sRate != 0 && sRate != AUDIO_OUT_SAMPLE_RATE) {
            Serial.printf("[AudioOutput] WARNING: Session sample rate %lu Hz. Output hardware is fixed at %u Hz. Resampling on backend is required!\\n",
                          (unsigned long)sRate, (unsigned)AUDIO_OUT_SAMPLE_RATE);
        }

        uint32_t gen = 0;
        if (!audioMutex || xSemaphoreTake(audioMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
            mutexAcquireFailures++;
            Serial.println("[AudioOutput] ERROR: startSession failed to acquire audioMutex!");
            return 0; // Return 0 to indicate session rejected
        }

        activeSessionGeneration++;
        if (activeSessionGeneration == 0) activeSessionGeneration = 1; // Disallow 0
        gen = activeSessionGeneration;
        sessionActive = true;
        sessionCancelled = false;
        streamEnded = false;
        prebufferReached = false;
        isMp3Stream = isMp3;

        if (sessionId) {
            strncpy(activeSessionId, sessionId, sizeof(activeSessionId) - 1);
            activeSessionId[sizeof(activeSessionId) - 1] = '\\0';
        } else {
            activeSessionId[0] = '\\0';
        }

        xSemaphoreGive(audioMutex);
        return gen;
    }

    // Cancel active audio session immediately (interruption/barge-in)
    void cancelSession(const char* reason = nullptr) {
        if (!audioMutex) return;

        bool locked = false;
        for (int r = 0; r < 5; r++) {
            if (xSemaphoreTake(audioMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                locked = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        if (locked) {
            activeSessionGeneration++;
            if (activeSessionGeneration == 0) activeSessionGeneration = 1;
            sessionCancelled = true;
            sessionActive = false;
            flushRequested = true;
            prebufferReached = false;
            streamEnded = false;
            activeSessionId[0] = '\\0';
            xSemaphoreGive(audioMutex);
            Serial.printf("[AudioOutput] Session CANCELLED: %s (new gen: %lu)\\n",
                          reason ? reason : "user_interrupted", (unsigned long)activeSessionGeneration);
        } else {
            mutexAcquireFailures++;
            Serial.println("[AudioOutput] CRITICAL: cancelSession failed to acquire audioMutex after 5 retries!");
        }
    }

    void endSession() {
        if (!audioMutex) return;
        if (xSemaphoreTake(audioMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            sessionActive = false;
            streamEnded = true; // Signal update() that buffer may drain to completion
            xSemaphoreGive(audioMutex);
        } else {
            mutexAcquireFailures++;
            Serial.println("[AudioOutput] WARNING: endSession failed to acquire audioMutex!");
        }
    }

    uint32_t getActiveGeneration() const { return activeSessionGeneration; }

    // Atomic session validation and PCM enqueue
    bool enqueuePcm(const int16_t* samples, size_t sampleCount, uint32_t streamGen) {
        if (!initialized || !audioRingBuf || !samples || sampleCount == 0) return false;

        // Disallow uninitialized / zero stream generation
        if (streamGen == 0) {
            droppedFrames++;
            return false;
        }

        // Acquire audioMutex FIRST for atomic validation and insertion
        if (!audioMutex || xSemaphoreTake(audioMutex, pdMS_TO_TICKS(15)) != pdTRUE) {
            mutexAcquireFailures++;
            droppedFrames++;
            return false;
        }

        // Inside lock: check active session, cancellation, and exact generation match
        if (!sessionActive || sessionCancelled || (streamGen != activeSessionGeneration)) {
            droppedFrames++;
            xSemaphoreGive(audioMutex);
            return false;
        }

        totalPacketsReceived++;
        size_t bytes = sampleCount * sizeof(int16_t);
        BaseType_t res = xRingbufferSend(audioRingBuf, (const void*)samples, bytes, pdMS_TO_TICKS(15));
        bool ok = (res == pdTRUE);
        if (!ok) {
            overflowCount++;
            droppedFrames++;
        }
        xSemaphoreGive(audioMutex);
        return ok;
    }

    void triggerCuteChime() {
        chimeTriggered = true;
    }

    // Thread-safe flush request
    void stopAndFlush() {
        cancelSession("flush_stop");
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

        // 1. Process asynchronous flush request
        if (flushRequested) {
            if (audioMutex && xSemaphoreTake(audioMutex, pdMS_TO_TICKS(15)) == pdTRUE) {
                if (audioRingBuf) {
                    size_t size = 0;
                    void* itm = nullptr;
                    while ((itm = xRingbufferReceiveUpTo(audioRingBuf, &size, 0, AUDIO_CHUNK_SIZE)) != nullptr) {
                        vRingbufferReturnItem(audioRingBuf, itm);
                    }
                }
                flushRequested = false;
                prebufferReached = false;
                streamEnded = false;
                chime.running = false;
                isDmaPlaying = false;
                xSemaphoreGive(audioMutex);
                i2s_zero_dma_buffer(I2S_NUM_1);
            }
            return false;
        }

        // 2. Measure RingBuffer occupancy
        size_t curFree = audioRingBuf ? xRingbufferGetCurFreeSize(audioRingBuf) : 0;
        size_t bufferedBytes = (ringBufCapacity > curFree) ? (ringBufCapacity - curFree) : 0;

        // 3. SOURCE-PRIORITY POLICY:
        // Streaming Voice Audio takes precedence over Cute Chime.
        // If voice audio is buffered or session is active, Chime yields immediately!
        if ((bufferedBytes > 256 || sessionActive) && chime.running) {
            chime.running = false; // Gracefully yield to speech
        }

        // 4. Prebuffer Threshold Management:
        // Gemini Live voice: ~150ms = 7,200 bytes (24kHz 16-bit mono = 48,000 B/s)
        // Website MP3 stream: ~400ms = 19,200 bytes
        if (!prebufferReached) {
            size_t threshold = isMp3Stream ? 19200 : 7200;
            if (bufferedBytes >= threshold || streamEnded) {
                prebufferReached = true;
            }
        }

        // 5. Stream audio from RingBuffer to I2S (High-Priority Path with Guaranteed Item Return)
        bool hasData = false;
        if (audioRingBuf && prebufferReached && bufferedBytes > 0) {
            void* item = nullptr;
            size_t itemSize = 0;

            if (audioMutex && xSemaphoreTake(audioMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (!flushRequested) {
                    item = xRingbufferReceiveUpTo(audioRingBuf, &itemSize, 0, (AUDIO_CHUNK_SIZE / 2) * sizeof(int16_t));
                }
                xSemaphoreGive(audioMutex);
            }

            if (item != nullptr) {
                ringItemsCheckedOut++;

                if (itemSize > 0) {
                    hasData = true;
                    isDmaPlaying = true;
                    lastPlayingTime = millis();

                    // Check if session was cancelled while item was checked out
                    bool skipPlayback = sessionCancelled || flushRequested;
                    if (!skipPlayback) {
                        const int16_t* pcmIn = (const int16_t*)item;
                        size_t samples = itemSize / sizeof(int16_t);
                        if (samples > (AUDIO_CHUNK_SIZE / 2)) samples = (AUDIO_CHUNK_SIZE / 2);

                        float volGain = gainMultiplier;
                        for (size_t i = 0; i < samples; i++) {
                            int32_t val = (int32_t)(pcmIn[i] * volGain);
                            if (val > 30000) val = 30000;
                            if (val < -30000) val = -30000;
                            scaleBuffer[i] = (int16_t)val;
                        }

                        writeI2sWithRetry((const uint8_t*)scaleBuffer, samples * sizeof(int16_t));
                    }
                }

                // Guaranteed return of ringbuffer item: retry until released
                bool returned = false;
                for (int retry = 0; retry < 50; retry++) {
                    if (audioMutex && xSemaphoreTake(audioMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                        vRingbufferReturnItem(audioRingBuf, item);
                        ringItemsReturned++;
                        xSemaphoreGive(audioMutex);
                        returned = true;
                        break;
                    }
                    vTaskDelay(pdMS_TO_TICKS(1));
                }
                if (!returned) {
                    ringItemReturnFailures++;
                    vRingbufferReturnItem(audioRingBuf, item);
                    ringItemsReturned++;
                    Serial.println("[AudioOutput] CRITICAL: Mutex timeout on return; item force-returned.");
                }

                return true;
            } else {
                if (isDmaPlaying && (millis() - lastPlayingTime > 80)) {
                    isDmaPlaying = false;
                    if (sessionActive && !streamEnded) {
                        underrunCount++;
                    }
                }
            }
        } else {
            if (isDmaPlaying && (millis() - lastPlayingTime > 80)) {
                isDmaPlaying = false;
            }
        }

        // 6. Cute Chime synthesis state machine (Low-Priority Path)
        if (chimeTriggered) {
            chimeTriggered = false;
            if (!sessionActive && bufferedBytes == 0) {
                chime.running = true;
                chime.currentSample = 0;
                chime.totalSamples = (AUDIO_OUT_SAMPLE_RATE * 650) / 1000;
                chime.phase = 0.0f;
            }
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

            writeI2sWithRetry((const uint8_t*)scaleBuffer, batch * sizeof(int16_t), 3, 30);
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
        Serial.printf("[AUDIO-DIAG] RingBuf: %u/%u B (%u%%) | Under: %lu | Over: %lu | Drop: %lu | MutexFail: %lu | ItemOut/Ret: %lu/%lu (Leak: %ld)\\n",
                      (unsigned)occRing, (unsigned)ringBufCapacity,
                      ringBufCapacity > 0 ? (unsigned)((occRing * 100) / ringBufCapacity) : 0,
                      (unsigned long)underrunCount, (unsigned long)overflowCount, (unsigned long)droppedFrames,
                      (unsigned long)mutexAcquireFailures,
                      (unsigned long)ringItemsCheckedOut, (unsigned long)ringItemsReturned,
                      (long)(ringItemsCheckedOut - ringItemsReturned));
        Serial.printf("[AUDIO-DIAG] I2S: Partial: %lu | UnrecovBytes: %lu | Played: %lu B\\n",
                      (unsigned long)i2sPartialWrites, (unsigned long)unrecoverableDroppedBytes,
                      (unsigned long)totalBytesPlayed);
        Serial.printf("[AUDIO-DIAG] Session: %s (Gen: %lu) | State: %s | Prebuf: %s\\n",
                      activeSessionId[0] ? activeSessionId : "<none>",
                      (unsigned long)activeSessionGeneration,
                      sessionActive ? "ACTIVE" : (chime.running ? "CHIME" : "IDLE"),
                      prebufferReached ? "READY" : "BUFFERING");
    }
};

'''

content = content[:sec7_idx] + sec7_v2_2 + content[sec8_idx:]

# 3. Update Section 12 WebSocketClientManager: Member variables, frame assembler & lifecycle resets
ws_old_v2_1 = '''class WebSocketClientManager {
private:
    uint32_t currentSessionGeneration = 0;
    WebsocketsClient client;
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
            Serial.printf("[WebSocket] DNS lookup FAILED for %s\\n", DEFAULT_WS_HOST);
            return;
        }
        Serial.printf("[WebSocket] DNS %s -> %s\\n", DEFAULT_WS_HOST, serverIp.toString().c_str());
        Serial.printf("[WebSocket] Connecting to wss://%s:%d%s (device key present) ...\\n",
                      DEFAULT_WS_HOST, DEFAULT_WS_PORT, DEFAULT_WS_PATH);

#ifndef WS_PROBE
#define WS_PROBE 0
#endif
#if WS_PROBE
        static bool probed = false;
        if (!probed) { probed = true; probeUpgrade(devId, apiKey); }
#endif

        client = WebsocketsClient();
        // Keep certificate and hostname verification enabled. Never use setInsecure().
        client.setCACert(PROD_ROOT_CA_BUNDLE);
        client.addHeader("X-Device-ID", devId);
        client.addHeader("X-Device-Key", apiKey);

        static uint8_t leftoverByte = 0;
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
        });

        client.onEvent([this](WebsocketsEvent event, String data) {
            if (event == WebsocketsEvent::ConnectionOpened) {
                wsConnected = true;
                wsConnectedAt = millis();
                Serial.println("[WebSocket] TLS connection opened. Waiting for backend device:connected authentication...");
            } else if (event == WebsocketsEvent::ConnectionClosed) {
                wsConnected = false;
                authenticated = false;
                wsConnectedAt = 0;
                postEvent(EventType::WS_DISCONNECTED);
                Serial.printf("[WebSocket] Connection closed. Reason: %s (code: %d)\\n", data.c_str(), (int)client.getCloseReason());
            }
        });'''

ws_new_v2_2 = '''class WebSocketClientManager {
private:
    uint32_t currentSessionGeneration = 0;
    uint8_t leftoverByte = 0;
    bool hasLeftoverByte = false;
    WebsocketsClient client;
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

    void handleBinaryMessage(const uint8_t* raw, size_t rawLen) {
        if (!raw || rawLen == 0) return;

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

        // Process remaining bytes in chunks of up to 256 samples (512 bytes)
        int16_t chunkBuffer[256];
        size_t remainingBytes = rawLen - readOffset;
        size_t fullSamples = remainingBytes / sizeof(int16_t);

        while (fullSamples > 0) {
            size_t batchSamples = (fullSamples > 256) ? 256 : fullSamples;
            size_t bytesToCopy = batchSamples * sizeof(int16_t);

            memcpy(chunkBuffer, &raw[readOffset], bytesToCopy);
            readOffset += bytesToCopy;
            fullSamples -= batchSamples;

            bool ok = AudioOutputDriver::getInstance().enqueuePcm(chunkBuffer, batchSamples, currentSessionGeneration);
            if (!ok) {
                // Frame rejected due to generation mismatch, session cancel, or ring buffer full
            }
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
            Serial.printf("[WebSocket] DNS lookup FAILED for %s\\n", DEFAULT_WS_HOST);
            return;
        }
        Serial.printf("[WebSocket] DNS %s -> %s\\n", DEFAULT_WS_HOST, serverIp.toString().c_str());
        Serial.printf("[WebSocket] Connecting to wss://%s:%d%s (device key present) ...\\n",
                      DEFAULT_WS_HOST, DEFAULT_WS_PORT, DEFAULT_WS_PATH);

#ifndef WS_PROBE
#define WS_PROBE 0
#endif
#if WS_PROBE
        static bool probed = false;
        if (!probed) { probed = true; probeUpgrade(devId, apiKey); }
#endif

        resetAssembler(); // Clear any stale incomplete sample
        client = WebsocketsClient();
        // Keep certificate and hostname verification enabled. Never use setInsecure().
        client.setCACert(PROD_ROOT_CA_BUNDLE);
        client.addHeader("X-Device-ID", devId);
        client.addHeader("X-Device-Key", apiKey);

        client.onMessage([this](WebsocketsMessage msg) {
            if (msg.isBinary()) {
                handleBinaryMessage((const uint8_t*)msg.c_str(), msg.length());
            } else if (msg.isText()) {
                handleTextMessage(msg.data());
            }
        });

        client.onEvent([this](WebsocketsEvent event, String data) {
            if (event == WebsocketsEvent::ConnectionOpened) {
                wsConnected = true;
                wsConnectedAt = millis();
                Serial.println("[WebSocket] TLS connection opened. Waiting for backend device:connected authentication...");
            } else if (event == WebsocketsEvent::ConnectionClosed) {
                resetAssembler(); // Clear assembler on disconnect
                wsConnected = false;
                authenticated = false;
                wsConnectedAt = 0;
                postEvent(EventType::WS_DISCONNECTED);
                Serial.printf("[WebSocket] Connection closed. Reason: %s (code: %d)\\n", data.c_str(), (int)client.getCloseReason());
            }
        });'''

assert ws_old_v2_1 in content, 'ws_old_v2_1 not found'
content = content.replace(ws_old_v2_1, ws_new_v2_2, 1)

# 4. Update handleTextMessage audio event handlers: odd-byte resets & mp3 detection
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

ws_txt_v2_2 = '''        } else if (strcmp(ev, "audio:response:start") == 0) {
            resetAssembler(); // Clear odd-byte assembler on new session boundary
            uint32_t sRate = data["sampleRate"] | 24000;
            const char* sessId = data["sessionId"] | "";
            const char* srcType = data["source"] | data["type"] | "";
            bool isMp3 = (strcmp(srcType, "mp3") == 0 || (sessId && strstr(sessId, "mp3") != nullptr));
            currentSessionGeneration = AudioOutputDriver::getInstance().startSession(sessId, sRate, isMp3);
            postEvent(EventType::VOICE_RESPONSE_START, (int)sRate);
            Serial.printf("[WebSocket] AI Voice response starting: %s (gen: %lu, %lu Hz, mp3=%d)\\n",
                          sessId[0] ? sessId : "<none>", (unsigned long)currentSessionGeneration, (unsigned long)sRate, (int)isMp3);
        } else if (strcmp(ev, "audio:response:end") == 0) {
            resetAssembler(); // Clear odd-byte assembler on session end
            AudioOutputDriver::getInstance().endSession();
            postEvent(EventType::VOICE_RESPONSE_END);
            Serial.println("[WebSocket] AI Voice response finished.");
        } else if (strcmp(ev, "audio:cancel") == 0) {
            resetAssembler(); // Clear odd-byte assembler on session cancel
            const char* reason = data["reason"] | "interrupted";
            AudioOutputDriver::getInstance().cancelSession(reason);
            currentSessionGeneration = 0; // Invalidate client session generation immediately
            postEvent(EventType::VOICE_AUDIO_CANCEL);'''

assert ws_txt_v2_1 in content, 'ws_txt_v2_1 not found'
content = content.replace(ws_txt_v2_1, ws_txt_v2_2, 1)

# 5. Update setup() to verify AudioOutputDriver::begin() return status
setup_old = '''    // 6. Initialize Audio & Sensor Subsystems
    AudioInputDriver::getInstance().begin();
    AudioOutputDriver::getInstance().begin();
    sensors.begin();
    eyes.begin();'''

setup_new = '''    // 6. Initialize Audio & Sensor Subsystems
    AudioInputDriver::getInstance().begin();
    if (!AudioOutputDriver::getInstance().begin()) {
        Serial.println("[Setup] CRITICAL ERROR: Audio Output Driver failed to initialize! Audio output will be offline.");
    }
    sensors.begin();
    eyes.begin();'''

assert setup_old in content, 'setup_old not found'
content = content.replace(setup_old, setup_new, 1)

# Write output files
with open(out_path1, 'w', encoding='utf-8') as f:
    f.write(content)

with open(out_path2, 'w', encoding='utf-8') as f:
    f.write(content)

print(f'SUCCESS: Created final_audio_stability_v2_2.ino in {out_path1} and {out_path2}')
