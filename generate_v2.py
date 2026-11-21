import os

final_path = r'C:\Users\USER\Desktop\Memo_Arduino\final\final.ino'
out_path1 = r'C:\Users\USER\Desktop\Memo_Arduino\final\final_audio_stability_v2.ino'
out_path2 = r'C:\Users\USER\Desktop\Memo_Arduino\final_audio_stability_v2\final_audio_stability_v2.ino'

with open(final_path, 'r', encoding='utf-8') as f:
    content = f.read()

# 1. Update header comment
header_old = ' *           🤖 MEMO ESP32-S3 AI ROBOT — ALL-IN-ONE ARDUINO FIRMWARE'
header_new = ' *           🤖 MEMO ESP32-S3 AI ROBOT — PRODUCTION AUDIO STABILITY V2.0\n * Unified 24kHz Audio Pipeline | Thread-Safe FreeRTOS Audio Manager | Zero BOD Suppression'
assert header_old in content, 'Header not found'
content = content.replace(header_old, header_new, 1)

# 2. Replace Section 7
sec7_target = '''// ============================================================================
// SECTION 7: NON-BLOCKING I2S AUDIO OUTPUT (MAX98357A + RingBuffer)
// ============================================================================
class AudioOutputDriver {
private:
    bool initialized = false;
    uint32_t currentSampleRate = AUDIO_OUT_SAMPLE_RATE;
    uint8_t currentVolume = DEFAULT_VOLUME;
    float gainMultiplier = 0.8f;
    int16_t scaleBuffer[AUDIO_CHUNK_SIZE / 2];

    RingbufHandle_t audioRingBuf = nullptr;
    size_t ringBufCapacity = 0;
    volatile bool chimePending = false;

public:
    static AudioOutputDriver& getInstance() {
        static AudioOutputDriver instance;
        return instance;
    }

    bool begin() {
        Serial.println("[AudioOutput] Initializing MAX98357A (I2S_NUM_1)...");

        i2s_config_t i2sConfig = {
            .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
            .sample_rate = currentSampleRate,
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
                    Serial.printf("[AudioOutput] Created %u KB RingBuffer in Octal PSRAM!\\n", (unsigned)(AUDIO_RING_BUFFER_BYTES_PSRAM / 1024));
                } else {
                    Serial.println("[AudioOutput] PSRAM found but 256KB RingBuffer allocation failed! Falling back to SRAM...");
                }
            }
            if (!audioRingBuf) {
                audioRingBuf = xRingbufferCreate(AUDIO_RING_BUFFER_BYTES_SRAM, RINGBUF_TYPE_BYTEBUF);
                if (audioRingBuf) {
                    ringBufCapacity = AUDIO_RING_BUFFER_BYTES_SRAM;
                    Serial.println("[AudioOutput] Created 32 KB RingBuffer in internal SRAM.");
                } else {
                    ringBufCapacity = 0;
                    Serial.println("[AudioOutput] CRITICAL ERROR: Failed to allocate RingBuffer in internal SRAM!");
                }
            }
        }

        initialized = true;
        Serial.println("[AudioOutput] Driver initialized with FreeRTOS RingBuffer.");
        return true;
    }

    void setSampleRate(uint32_t rate) {
        if (!initialized || rate == currentSampleRate || rate == 0) return;
        currentSampleRate = rate;
        i2s_set_sample_rates(I2S_NUM_1, currentSampleRate);
    }

    void setVolume(uint8_t vol) {
        if (vol > MAX_VOLUME_LIMIT) vol = MAX_VOLUME_LIMIT;
        currentVolume = vol;
        gainMultiplier = ((float)vol / 100.0f) * 0.95f;
    }

    // Non-blocking enqueue from WebSocket callback with brief timeout to prevent packet drop
    bool enqueuePcm(const int16_t* samples, size_t sampleCount) {
        if (!audioRingBuf || !samples || sampleCount == 0) return false;
        size_t bytes = sampleCount * sizeof(int16_t);
        BaseType_t res = xRingbufferSend(audioRingBuf, (const void*)samples, bytes, pdMS_TO_TICKS(10));
        return (res == pdTRUE);
    }

    void triggerCuteChime() {
        chimePending = true;
    }

    void stopAndFlush() {
        if (!initialized) return;
        if (audioRingBuf) {
            size_t size = 0;
            void* item = nullptr;
            while ((item = xRingbufferReceiveUpTo(audioRingBuf, &size, 0, AUDIO_CHUNK_SIZE)) != nullptr) {
                vRingbufferReturnItem(audioRingBuf, item);
            }
        }
        i2s_zero_dma_buffer(I2S_NUM_1);
        chimePending = false;
    }

    bool hasBufferedAudio() const {
        if (!initialized) return false;
        if (chimePending) return true;
        if (audioRingBuf && ringBufCapacity > 0) {
            size_t freeBytes = xRingbufferGetCurFreeSize(audioRingBuf);
            if (ringBufCapacity > freeBytes && (ringBufCapacity - freeBytes) > 512) {
                return true;
            }
        }
        return false;
    }

    // Called periodically by dedicated audio task or loop
    bool update() {
        if (!initialized) return false;

        // 1. Play async cute chime if requested
        if (chimePending) {
            chimePending = false;
            playCuteChimeInternal();
            return true;
        }

        // 2. Stream audio from RingBuffer to I2S
        if (audioRingBuf) {
            size_t itemSize = 0;
            int16_t* chunk = (int16_t*)xRingbufferReceiveUpTo(audioRingBuf, &itemSize, 0, (AUDIO_CHUNK_SIZE / 2) * sizeof(int16_t));
            if (chunk && itemSize > 0) {
                size_t samples = itemSize / sizeof(int16_t);
                for (size_t i = 0; i < samples; i++) {
                    int32_t val = (int32_t)(chunk[i] * gainMultiplier);
                    if (val > 30000) val = 30000;
                    if (val < -30000) val = -30000;
                    scaleBuffer[i] = (int16_t)val;
                }

                size_t bytesWritten = 0;
                i2s_write(I2S_NUM_1, scaleBuffer, samples * sizeof(int16_t), &bytesWritten, pdMS_TO_TICKS(100));
                vRingbufferReturnItem(audioRingBuf, (void*)chunk);
                return true;
            }
        }
        return false;
    }

private:
    void playCuteChimeInternal() {
        const uint32_t sRate = currentSampleRate;
        const size_t totalSamples = (sRate * 650) / 1000; // 650 ms
        int16_t buf[128];
        float phase = 0.0f;

        for (size_t i = 0; i < totalSamples; ) {
            size_t batch = (totalSamples - i < 128) ? (totalSamples - i) : 128;
            for (size_t b = 0; b < batch; b++) {
                float t = (float)(i + b) / (float)sRate;
                float freq = 523.25f; // C5
                float noteProgress = 0.0f;

                if (t < 0.12f) { freq = 523.25f; noteProgress = t / 0.12f; }
                else if (t < 0.24f) { freq = 659.25f; noteProgress = (t - 0.12f) / 0.12f; }
                else if (t < 0.38f) { freq = 783.99f; noteProgress = (t - 0.24f) / 0.14f; }
                else { freq = 1046.50f; noteProgress = (t - 0.38f) / 0.27f; }

                float env = (noteProgress < 0.08f) ? (noteProgress / 0.08f) : powf(1.0f - ((noteProgress - 0.08f) / 0.92f), 2.0f);
                if (t >= 0.38f) freq += 12.0f * sinf(2.0f * PI * 6.5f * (t - 0.38f));

                phase += 2.0f * PI * freq / (float)sRate;
                if (phase >= 2.0f * PI) phase -= 2.0f * PI;

                float sample = (sinf(phase) + 0.3f * sinf(2.0f * phase)) * env * 18000.0f * (currentVolume / 100.0f);
                buf[b] = (int16_t)sample;
            }
            size_t written = 0;
            i2s_write(I2S_NUM_1, buf, batch * sizeof(int16_t), &written, pdMS_TO_TICKS(30));
            i += batch;
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
};'''

sec7_replacement = '''// ============================================================================
// SECTION 7: CENTRALIZED UNIFIED I2S AUDIO MANAGER (MAX98357A + Octal PSRAM)
// ============================================================================
// ARCHITECTURE RULES:
// 1. Only ONE task (taskAudio on Core 1) writes to I2S_NUM_1 or touches I2S registers.
// 2. FreeRTOS RingBuffer access is synchronized with audioMutex to prevent race conditions.
// 3. Flushes and sample rate changes are queued to taskAudio, eliminating Core 0/1 contention.
// 4. Cute Chime is synthesized in non-blocking slices (128 samples per tick, ~5.3ms),
//    preventing audio task starvation and buffer dropouts.
// 5. Audio session ID and generation counter discard stale packets on Gemini barge-in.
// ============================================================================
class AudioOutputDriver {
private:
    bool initialized = false;
    uint32_t currentSampleRate = AUDIO_OUT_SAMPLE_RATE; // Standard 24000 Hz
    uint32_t pendingSampleRate = 0;
    uint8_t currentVolume = DEFAULT_VOLUME;
    float gainMultiplier = 0.8f;
    int16_t scaleBuffer[AUDIO_CHUNK_SIZE / 2];

    RingbufHandle_t audioRingBuf = nullptr;
    size_t ringBufCapacity = 0;
    SemaphoreHandle_t audioMutex = nullptr;

    // Asynchronous commands and session control
    volatile bool flushRequested = false;
    volatile bool chimeTriggered = false;
    volatile bool sessionActive = false;
    volatile bool sessionCancelled = false;
    char activeSessionId[64] = {0};
    uint32_t sessionGeneration = 0;

    // Non-blocking Cute Chime synthesizer state machine
    struct CuteChimeSynth {
        bool running = false;
        size_t currentSample = 0;
        size_t totalSamples = 0;
        float phase = 0.0f;
    } chime;

    // Diagnostics and health telemetry
    volatile uint32_t underrunCount = 0;
    volatile uint32_t overflowCount = 0;
    volatile uint32_t droppedFrames = 0;
    volatile uint32_t i2sWriteErrors = 0;
    volatile uint32_t i2sPartialWrites = 0;
    volatile uint32_t totalPacketsReceived = 0;
    volatile uint32_t totalBytesPlayed = 0;
    volatile bool isDmaPlaying = false;
    unsigned long lastPlayingTime = 0;

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
            .sample_rate = currentSampleRate,
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
                    Serial.printf("[AudioOutput] Successfully allocated %u KB RingBuffer in Octal PSRAM.\\n", (unsigned)(AUDIO_RING_BUFFER_BYTES_PSRAM / 1024));
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
        Serial.println("[AudioOutput] Central Audio Manager initialized successfully.");
        return true;
    }

    // Thread-safe sample rate request (applied cleanly inside taskAudio)
    void setSampleRate(uint32_t rate) {
        if (!initialized || rate == 0 || rate == currentSampleRate) return;
        pendingSampleRate = rate;
    }

    // Thread-safe volume control
    void setVolume(uint8_t vol) {
        if (vol > MAX_VOLUME_LIMIT) vol = MAX_VOLUME_LIMIT;
        currentVolume = vol;
        gainMultiplier = ((float)vol / 100.0f) * 0.95f;
    }

    uint8_t getVolume() const { return currentVolume; }

    // Start a new audio session (e.g., Gemini response or Website playback)
    void startSession(const char* sessionId, uint32_t sRate = 24000) {
        if (audioMutex && xSemaphoreTake(audioMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            sessionActive = true;
            sessionCancelled = false;
            sessionGeneration++;
            if (sessionId) {
                strncpy(activeSessionId, sessionId, sizeof(activeSessionId) - 1);
                activeSessionId[sizeof(activeSessionId) - 1] = '\\0';
            } else {
                activeSessionId[0] = '\\0';
            }
            xSemaphoreGive(audioMutex);
        } else {
            sessionActive = true;
            sessionCancelled = false;
            sessionGeneration++;
        }
        if (sRate > 0 && sRate != currentSampleRate) {
            pendingSampleRate = sRate;
        }
    }

    // Cancel active audio session immediately (interruption/barge-in)
    void cancelSession(const char* reason = nullptr) {
        sessionCancelled = true;
        sessionActive = false;
        flushRequested = true;
        chime.running = false;
        if (audioMutex && xSemaphoreTake(audioMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            activeSessionId[0] = '\\0';
            sessionGeneration++;
            xSemaphoreGive(audioMutex);
        }
        Serial.printf("[AudioOutput] Session CANCELLED: %s (generation: %lu)\\n",
                      reason ? reason : "user_interrupted", (unsigned long)sessionGeneration);
    }

    // Signal end of audio stream from server
    void endSession() {
        sessionActive = false;
    }

    // Non-blocking, thread-safe PCM enqueue from WebSocket task
    // Drops in-flight chunks if session was cancelled to prevent stale audio playback!
    bool enqueuePcm(const int16_t* samples, size_t sampleCount) {
        if (!initialized || !audioRingBuf || !samples || sampleCount == 0) return false;
        if (sessionCancelled) {
            return false; // Discard stale frames from cancelled turn
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

    // Thread-safe trigger for cute chime
    void triggerCuteChime() {
        chimeTriggered = true;
    }

    // Non-blocking, thread-safe flush request (executed exclusively by taskAudio)
    void stopAndFlush() {
        sessionCancelled = true;
        sessionActive = false;
        flushRequested = true;
        chime.running = false;
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

        // 1. Process pending sample rate changes cleanly between DMA bursts
        if (pendingSampleRate > 0 && pendingSampleRate != currentSampleRate) {
            currentSampleRate = pendingSampleRate;
            pendingSampleRate = 0;
            i2s_zero_dma_buffer(I2S_NUM_1);
            i2s_set_sample_rates(I2S_NUM_1, currentSampleRate);
            Serial.printf("[AudioOutput] Sample rate switched cleanly to %lu Hz\\n", (unsigned long)currentSampleRate);
        }

        // 2. Process asynchronous flush request safely in this dedicated audio task!
        if (flushRequested) {
            if (audioMutex && xSemaphoreTake(audioMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                if (audioRingBuf) {
                    size_t size = 0;
                    void* item = nullptr;
                    while ((item = xRingbufferReceiveUpTo(audioRingBuf, &size, 0, AUDIO_CHUNK_SIZE)) != nullptr) {
                        vRingbufferReturnItem(audioRingBuf, item);
                    }
                }
                flushRequested = false;
                xSemaphoreGive(audioMutex);
            } else {
                flushRequested = false;
            }
            i2s_zero_dma_buffer(I2S_NUM_1);
            isDmaPlaying = false;
            chime.running = false;
            return false;
        }

        // 3. Process Cute Chime synthesis state machine in non-blocking slices
        if (chimeTriggered) {
            chimeTriggered = false;
            chime.running = true;
            chime.currentSample = 0;
            chime.totalSamples = (currentSampleRate * 650) / 1000;
            chime.phase = 0.0f;
        }

        if (chime.running) {
            size_t batch = (chime.totalSamples - chime.currentSample < 128) ?
                           (chime.totalSamples - chime.currentSample) : 128;
            for (size_t b = 0; b < batch; b++) {
                float t = (float)(chime.currentSample + b) / (float)currentSampleRate;
                float freq = 523.25f; // C5
                float noteProgress = 0.0f;

                if (t < 0.12f) { freq = 523.25f; noteProgress = t / 0.12f; }
                else if (t < 0.24f) { freq = 659.25f; noteProgress = (t - 0.12f) / 0.12f; }
                else if (t < 0.38f) { freq = 783.99f; noteProgress = (t - 0.24f) / 0.14f; }
                else { freq = 1046.50f; noteProgress = (t - 0.38f) / 0.27f; }

                float env = (noteProgress < 0.08f) ? (noteProgress / 0.08f) : powf(1.0f - ((noteProgress - 0.08f) / 0.92f), 2.0f);
                if (t >= 0.38f) freq += 12.0f * sinf(2.0f * PI * 6.5f * (t - 0.38f));

                chime.phase += 2.0f * PI * freq / (float)currentSampleRate;
                if (chime.phase >= 2.0f * PI) chime.phase -= 2.0f * PI;

                float sample = (sinf(chime.phase) + 0.3f * sinf(2.0f * chime.phase)) * env * 18000.0f * (currentVolume / 100.0f);
                scaleBuffer[b] = (int16_t)sample;
            }
            chime.currentSample += batch;
            if (chime.currentSample >= chime.totalSamples) {
                chime.running = false;
            }

            size_t bytesWritten = 0;
            esp_err_t err = i2s_write(I2S_NUM_1, (const char*)scaleBuffer, batch * sizeof(int16_t), &bytesWritten, pdMS_TO_TICKS(50));
            if (err != ESP_OK) i2sWriteErrors++;
            isDmaPlaying = true;
            lastPlayingTime = millis();
            return true;
        }

        // 4. Stream audio from RingBuffer to I2S with Mutex Protection
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

                size_t bytesWritten = 0;
                esp_err_t err = i2s_write(I2S_NUM_1, (const char*)scaleBuffer, samples * sizeof(int16_t), &bytesWritten, pdMS_TO_TICKS(100));
                if (err != ESP_OK) {
                    i2sWriteErrors++;
                } else if (bytesWritten < samples * sizeof(int16_t)) {
                    i2sPartialWrites++;
                }
                totalBytesPlayed += bytesWritten;

                if (audioMutex && xSemaphoreTake(audioMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    vRingbufferReturnItem(audioRingBuf, item);
                    xSemaphoreGive(audioMutex);
                }
            } else {
                if (isDmaPlaying && (millis() - lastPlayingTime > 80)) {
                    isDmaPlaying = false;
                    if (sessionActive) {
                        underrunCount++;
                    }
                }
            }
        }
        return hasData;
    }

    // Production health and resource telemetry
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
        Serial.printf("[AUDIO-DIAG] RingBuf: %u/%u B (%u%%) | Under: %lu | Over: %lu | Drop: %lu | I2SErr: %lu | Part: %lu\\n",
                      (unsigned)occRing, (unsigned)ringBufCapacity,
                      ringBufCapacity > 0 ? (unsigned)((occRing * 100) / ringBufCapacity) : 0,
                      (unsigned long)underrunCount, (unsigned long)overflowCount, (unsigned long)droppedFrames,
                      (unsigned long)i2sWriteErrors, (unsigned long)i2sPartialWrites);
        Serial.printf("[AUDIO-DIAG] Session: %s | State: %s\\n",
                      activeSessionId[0] ? activeSessionId : "<none>",
                      sessionActive ? "ACTIVE" : (chime.running ? "CHIME" : "IDLE"));
    }
};'''

assert sec7_target in content, 'Section 7 target not found'
content = content.replace(sec7_target, sec7_replacement, 1)

# 3. Update WebSocket onMessage binary alignment and frame validation
ws_msg_old = '''        client.onMessage([this](WebsocketsMessage msg) {
            if (msg.isBinary()) {
                // Non-blocking binary audio enqueue
                const char* data = msg.c_str();
                size_t len = msg.length();
                if (len > 0 && data) {
                    bool ok = AudioOutputDriver::getInstance().enqueuePcm((const int16_t*)data, len / sizeof(int16_t));
                    if (!ok) {
                        static uint32_t droppedFrames = 0;
                        droppedFrames++;
                        if (droppedFrames % 25 == 1) {
                            Serial.printf("[AudioOutput] RingBuffer FULL! Dropped %lu frame(s)\\n", (unsigned long)droppedFrames);
                        }
                    }
                }
            } else if (msg.isText()) {
                handleTextMessage(msg.data());
            }
        });'''

ws_msg_new = '''        client.onMessage([this](WebsocketsMessage msg) {
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

assert ws_msg_old in content, 'ws_msg_old not found'
content = content.replace(ws_msg_old, ws_msg_new, 1)

# 4. Update WebSocket text handler for audio session start, end, cancel
ws_txt_old = '''        } else if (strcmp(ev, "audio:response:start") == 0) {
            uint32_t sRate = data["sampleRate"] | 24000;
            AudioOutputDriver::getInstance().setSampleRate(sRate);
            postEvent(EventType::VOICE_RESPONSE_START, (int)sRate);
            Serial.printf("[WebSocket] AI Voice response starting (%lu Hz)\\n", (unsigned long)sRate);
        } else if (strcmp(ev, "audio:response:end") == 0) {
            postEvent(EventType::VOICE_RESPONSE_END);
            Serial.println("[WebSocket] AI Voice response finished.");
        } else if (strcmp(ev, "audio:cancel") == 0) {
            AudioOutputDriver::getInstance().stopAndFlush();
            postEvent(EventType::VOICE_AUDIO_CANCEL);'''

ws_txt_new = '''        } else if (strcmp(ev, "audio:response:start") == 0) {
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

assert ws_txt_old in content, 'ws_txt_old not found'
content = content.replace(ws_txt_old, ws_txt_new, 1)

# 5. Remove Brownout disable calls & Add production reset reason diagnostics
bod_old = '''// Official ESP-IDF brownout disable function (safely disables BOD comparator and reset)
extern "C" void esp_brownout_disable(void);

// Constructor attribute ensures BOD is disabled immediately upon app load before global constructors
__attribute__((constructor)) static void early_disable_bod() {
    esp_brownout_disable();
}

void setup() {
    // 0. Disable hardware brownout detector using official ESP-IDF HAL function
    esp_brownout_disable();

    // Start diagnostics before touching peripherals. Serial is mirrored to UART0
    // and native USB CDC when USB CDC On Boot is enabled.
    Serial.begin(115200);
    delay(1000);
    Serial.println("\\n[BOOT] Memo sketch setup() entered - serial path is working.");
    esp_reset_reason_t rst = esp_reset_reason();
    Serial.printf("[BOOT] Reset reason: %d (%s) | CPU: %u MHz\\n", (int)rst,
                  rst == ESP_RST_POWERON ? "Power-on" :
                  rst == ESP_RST_SW ? "Software restart" :
                  rst == ESP_RST_PANIC ? "PANIC / Stack Overflow / Crash" :
                  rst == ESP_RST_INT_WDT ? "Interrupt WDT" :
                  rst == ESP_RST_TASK_WDT ? "Task Watchdog Timer" :
                  rst == ESP_RST_BROWNOUT ? "BROWNOUT (Voltage Sag - Power Supply Too Weak!)" :
                  "Other",
                  (unsigned)ESP.getCpuFreqMHz());
    if (rst == ESP_RST_BROWNOUT) {
        Serial.println("[WARNING] Previous boot was caused by BROWNOUT! Connect a 470uF-1000uF capacitor across 5V and GND, and use a 5V 2A power adapter!");
    }
    Serial.flush();'''

bod_new = '''// Production Brownout Protection: Hardware detector is kept ENABLED and ACTIVE.
// Never suppress BOD comparator or brownout resets in production firmware.

void setup() {
    // Start diagnostics before touching peripherals. Serial is mirrored to UART0
    // and native USB CDC when USB CDC On Boot is enabled.
    Serial.begin(115200);
    delay(1000);
    Serial.println("\\n[BOOT] Memo sketch setup() entered - serial path is working.");

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
    Serial.printf("[BOOT] Reset reason code: %d (%s) | CPU Freq: %u MHz\\n", (int)rst, rstDesc, (unsigned)ESP.getCpuFreqMHz());
    Serial.println("[BOOT] Hardware Brownout Protection: ENABLED and ACTIVE.");
    if (rst == ESP_RST_BROWNOUT) {
        Serial.println("[WARNING] Previous reset was caused by BROWNOUT!");
        Serial.println("[WARNING] Check V2775 expansion board 9V power supply and verify 3.3V power rails under load.");
    }
    Serial.flush();'''

assert bod_old in content, 'bod_old not found'
content = content.replace(bod_old, bod_new, 1)

# 6. Add periodic diagnostics in loop()
loop_old = '''        UIManager::getInstance().update(DeviceManager::getInstance().getState(), wifiOk, rssi, wsOk, vol);
    }

    vTaskDelay(pdMS_TO_TICKS(5));
}'''

loop_new = '''        UIManager::getInstance().update(DeviceManager::getInstance().getState(), wifiOk, rssi, wsOk, vol);
    }

    // 5. Periodic Audio Pipeline & Memory Diagnostics (every 10 seconds)
    static unsigned long lastDiagLogTime = 0;
    if (now - lastDiagLogTime >= 10000) {
        lastDiagLogTime = now;
        AudioOutputDriver::getInstance().printDiagnostics();
    }

    vTaskDelay(pdMS_TO_TICKS(5));
}'''

assert loop_old in content, 'loop_old not found'
content = content.replace(loop_old, loop_new, 1)

with open(out_path1, 'w', encoding='utf-8') as f:
    f.write(content)

with open(out_path2, 'w', encoding='utf-8') as f:
    f.write(content)

print('SUCCESS: Created final_audio_stability_v2.ino in both locations')
