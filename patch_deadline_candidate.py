import os

target_file1 = r'C:\Users\USER\Desktop\Memo_Arduino\final\final_deadline_candidate.ino'
target_file2 = r'C:\Users\USER\Desktop\Memo_Arduino\final_deadline_candidate\final_deadline_candidate.ino'

with open(target_file1, 'r', encoding='utf-8') as f:
    content = f.read()

# 1. Update Section 7: AudioOutputDriver
sec7_start_marker = '// ============================================================================\n// SECTION 7: CENTRALIZED UNIFIED I2S AUDIO MANAGER (MAX98357A + Octal PSRAM)\n// ============================================================================'
sec8_start_marker = '// ============================================================================\n// SECTION 8: NON-BLOCKING I2S AUDIO INPUT (INMP441 Microphone Driver)\n// ============================================================================'

assert sec7_start_marker in content, 'sec7_start_marker not found'
assert sec8_start_marker in content, 'sec8_start_marker not found'

sec7_idx = content.find(sec7_start_marker)
sec8_idx = content.find(sec8_start_marker)

sec7_patched = '''// ============================================================================
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
    CMD_CHIME_REQUEST
};

struct AudioCommand {
    AudioCmdType type;
    uint32_t generation;
    uint32_t sampleRate;
    uint8_t volume;
    bool isMp3;
    char sessionId[32];
};

class AudioOutputDriver {
private:
    bool initialized = false;
    uint32_t currentSampleRate = AUDIO_OUT_SAMPLE_RATE; // Strictly fixed at 24000 Hz

    // Asynchronous command queue for serialized lifecycle management
    QueueHandle_t audioCmdQueue = nullptr;

    // FreeRTOS RingBuffer for rolling PCM audio streaming (256KB in PSRAM)
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
    int16_t scaleBuffer[AUDIO_CHUNK_SIZE / 2];
    bool streamEnded = false;
    bool isMp3Stream = false;
    bool prebufferReached = false;
    bool isFlowPaused = false; // Watermark state owned strictly by taskAudio
    uint32_t playbackGeneration = 0;
    char activeSessionId[32] = {0};
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
    std::atomic<uint32_t> totalPacketsReceived{0};
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
            Serial.printf("[AudioOutput] WARNING: I2S write incomplete. Lost %u bytes (total lost: %lu)\\n",
                          (unsigned)bytesRemaining, (unsigned long)unrecoverableDroppedBytes.load());
        }
        totalBytesPlayed.fetch_add(totalWritten, std::memory_order_relaxed);
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
                    Serial.printf("[AudioOutput] Allocated 256 KB (262,144 B) RingBuffer in Octal PSRAM.\\n");
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
            Serial.printf("[AudioOutput] Note: Metadata sample rate %lu Hz. Output hardware is unified at %u Hz.\\n",
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
        cmd.sessionId[0] = '\\0';

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
    uint32_t startSession(const char* sessionId, uint32_t sRate = 24000, bool isMp3 = false) {
        // Fix 4: Strictly reject non-24kHz incoming audio metadata
        if (sRate != 0 && sRate != AUDIO_OUT_SAMPLE_RATE) {
            Serial.printf("[AudioOutput] ERROR: Rejected stream. Unsupported sample rate %lu Hz. Hardware is strictly %u Hz!\\n",
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
        if (sessionId) {
            strncpy(cmd.sessionId, sessionId, sizeof(cmd.sessionId) - 1);
            cmd.sessionId[sizeof(cmd.sessionId) - 1] = '\\0';
        } else {
            cmd.sessionId[0] = '\\0';
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
        cmd.sessionId[0] = '\\0';

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

        Serial.printf("[AudioOutput] Session CANCELLED: %s (new gen: %lu)\\n",
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
        cmd.sessionId[0] = '\\0';

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

    bool checkAndClearReadyAck() {
        return sendReadyAckPending.exchange(false, std::memory_order_acq_rel);
    }

    bool checkAndClearFlowPause() {
        return flowControlPauseNeeded.exchange(false, std::memory_order_acq_rel);
    }

    bool checkAndClearFlowResume() {
        return flowControlResumeNeeded.exchange(false, std::memory_order_acq_rel);
    }

    size_t getFreeBufferBytes() const {
        return audioRingBuf ? xRingbufferGetCurFreeSize(audioRingBuf) : 0;
    }

    // Bounded PCM enqueue: fast 2ms timeout, zero blocking polling loop
    bool enqueuePcm(const int16_t* samples, size_t sampleCount, uint32_t streamGen) {
        if (!initialized || !audioRingBuf || !samples || sampleCount == 0) return false;

        // Disallow 0 or mismatched generation or cancelled or inactive session
        if (streamGen == 0 ||
            streamGen != activeSessionGeneration.load(std::memory_order_acquire) ||
            sessionCancelled.load(std::memory_order_acquire) ||
            !sessionActive.load(std::memory_order_acquire)) {
            droppedFrames.fetch_add(1, std::memory_order_relaxed);
            return false;
        }

        totalPacketsReceived.fetch_add(1, std::memory_order_relaxed);
        size_t bytes = sampleCount * sizeof(int16_t);

        // Bounded wait of 2ms: protects WebSocket callback from ever blocking
        BaseType_t res = xRingbufferSend(audioRingBuf, (const void*)samples, bytes, pdMS_TO_TICKS(2));
        if (res != pdTRUE) {
            overflowCount.fetch_add(1, std::memory_order_relaxed);
            droppedFrames.fetch_add(1, std::memory_order_relaxed);
            return false;
        }

        return true;
    }

    void triggerCuteChime() {
        AudioCommand cmd;
        cmd.type = AudioCmdType::CMD_CHIME_REQUEST;
        cmd.generation = 0;
        cmd.sampleRate = 0;
        cmd.volume = 0;
        cmd.isMp3 = false;
        cmd.sessionId[0] = '\\0';

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
            activeSessionId[0] = '\\0';
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
                    playbackGeneration = cmd.generation;
                    streamEnded = false;
                    prebufferReached = false;
                    isMp3Stream = cmd.isMp3;
                    isFlowPaused = false; // Reset flow pause state for new session
                    strncpy(activeSessionId, cmd.sessionId, sizeof(activeSessionId) - 1);
                    activeSessionId[sizeof(activeSessionId) - 1] = '\\0';
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
                    activeSessionId[0] = '\\0';
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
                case AudioCmdType::CMD_CHIME_REQUEST: {
                    size_t curFree = audioRingBuf ? xRingbufferGetCurFreeSize(audioRingBuf) : 0;
                    size_t curBuffered = (ringBufCapacity > curFree) ? (ringBufCapacity - curFree) : 0;
                    if (!sessionActive.load(std::memory_order_acquire) && curBuffered == 0) {
                        chime.running = true;
                        chime.currentSample = 0;
                        chime.totalSamples = (AUDIO_OUT_SAMPLE_RATE * 650) / 1000;
                        chime.phase = 0.0f;
                    }
                    break;
                }
            }
        }

        // 2. Measure RingBuffer occupancy & Watermark Flow Control (Fix 1: Owned strictly by taskAudio!)
        size_t curFree = audioRingBuf ? xRingbufferGetCurFreeSize(audioRingBuf) : 0;
        size_t bufferedBytes = (ringBufCapacity > curFree) ? (ringBufCapacity - curFree) : 0;

        // Continues monitoring and evaluating occupancy during playback even when producer is paused!
        if (sessionActive.load(std::memory_order_acquire)) {
            if (!isFlowPaused && bufferedBytes >= (ringBufCapacity * 3) / 4) {
                isFlowPaused = true;
                flowControlPauseNeeded.store(true, std::memory_order_release);
            } else if (isFlowPaused && bufferedBytes <= (ringBufCapacity * 2) / 5) {
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
                            scaleBuffer[i] = (int16_t)val;
                        }

                        writeI2sWithRetry((const uint8_t*)scaleBuffer, samples * sizeof(int16_t));
                    }
                }

                // GUARANTEED ZERO-LOCK ITEM RETURN:
                // FreeRTOS RingBuffer API is intrinsically thread-safe.
                // Returned immediately with 0 retries and 0 mutex overhead.
                vRingbufferReturnItem(audioRingBuf, item);
                ringItemsReturned.fetch_add(1, std::memory_order_relaxed);

                return true;
            } else {
                if (isAudioPlaying.load(std::memory_order_acquire) && (millis() - lastPlayingTime > 80)) {
                    isAudioPlaying.store(false, std::memory_order_release);
                    if (sessionActive.load(std::memory_order_acquire) && !streamEnded) {
                        underrunCount.fetch_add(1, std::memory_order_relaxed);
                    }
                }
            }
        } else {
            if (isAudioPlaying.load(std::memory_order_acquire) && (millis() - lastPlayingTime > 80)) {
                isAudioPlaying.store(false, std::memory_order_release);
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
                scaleBuffer[b] = (int16_t)sample;
            }
            chime.currentSample += batch;
            if (chime.currentSample >= chime.totalSamples) {
                chime.running = false;
            }

            writeI2sWithRetry((const uint8_t*)scaleBuffer, batch * sizeof(int16_t), 3, 30);
            isAudioPlaying.store(true, std::memory_order_release);
            lastPlayingTime = millis();
            return true;
        }

        return hasData;
    }

    // Production health, memory and recovery telemetry (Accurate 256KB capacity)
    void printDiagnostics(TaskHandle_t audioHandle = nullptr) {
        size_t freeHeap = ESP.getFreeHeap();
        size_t minHeap = ESP.getMinFreeHeap();
        size_t maxBlock = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
        size_t freePsram = ESP.getFreePsram();
        size_t totalPsram = ESP.getPsramSize();
        size_t freeRing = audioRingBuf ? xRingbufferGetCurFreeSize(audioRingBuf) : 0;
        size_t occRing = (ringBufCapacity > freeRing) ? (ringBufCapacity - freeRing) : 0;
        UBaseType_t stackHighWater = audioHandle ? uxTaskGetStackHighWaterMark(audioHandle) : 0;

        Serial.printf("[AUDIO-DIAG] Heap: %u B (Min: %u B, MaxBlk: %u B) | PSRAM: %u/%u KB\\n",
                      (unsigned)freeHeap, (unsigned)minHeap, (unsigned)maxBlock,
                      (unsigned)(freePsram / 1024), (unsigned)(totalPsram / 1024));
        Serial.printf("[AUDIO-DIAG] RingBuf (PSRAM): %u/%u B (%u%%) | Under: %lu | Over: %lu | Drop: %lu | ItemOut/Ret: %lu/%lu (Leak: %ld)\\n",
                      (unsigned)occRing, (unsigned)ringBufCapacity,
                      ringBufCapacity > 0 ? (unsigned)((occRing * 100) / ringBufCapacity) : 0,
                      (unsigned long)underrunCount.load(), (unsigned long)overflowCount.load(), (unsigned long)droppedFrames.load(),
                      (unsigned long)ringItemsCheckedOut.load(), (unsigned long)ringItemsReturned.load(),
                      (long)(ringItemsCheckedOut.load() - ringItemsReturned.load()));
        Serial.printf("[AUDIO-DIAG] I2S: Partial: %lu | LostBytes: %lu | Played: %lu B | StackFree: %u words | CmdQueueDrops: %lu\\n",
                      (unsigned long)i2sPartialWrites.load(), (unsigned long)unrecoverableDroppedBytes.load(),
                      (unsigned long)totalBytesPlayed.load(), (unsigned)stackHighWater, (unsigned long)cmdQueueFailures.load());
        Serial.printf("[AUDIO-DIAG] Session: %s (ActiveGen: %lu, PlayGen: %lu) | State: %s | Prebuf: %s | Flow: %s\\n",
                      activeSessionId[0] ? activeSessionId : "<none>",
                      (unsigned long)activeSessionGeneration.load(),
                      (unsigned long)playbackGeneration,
                      sessionActive.load() ? "ACTIVE" : (chime.running ? "CHIME" : "IDLE"),
                      prebufferReached ? "READY" : "BUFFERING",
                      isFlowPaused ? "PAUSED" : "FLOWING");
    }
};

'''

content = content[:sec7_idx] + sec7_patched + content[sec8_idx:]

with open(target_file1, 'w', encoding='utf-8') as f:
    f.write(content)

with open(target_file2, 'w', encoding='utf-8') as f:
    f.write(content)

print('Patched final_deadline_candidate.ino in both locations!')
