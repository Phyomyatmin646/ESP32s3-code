import os

audio_service_path = r'C:\Users\USER\Desktop\Ai_Robot_BE_CICD\apps\backend\src\modules\audio\audio.service.ts'
gemini_path = r'C:\Users\USER\Desktop\Ai_Robot_BE_CICD\apps\backend\src\modules\voice\gemini-live.client.ts'

# 1. Update audio.service.ts
with open(audio_service_path, 'r', encoding='utf-8') as f:
    text = f.read()

old_gw_import = "import {\n  getActiveEsp32Session,\n  getAllActiveEsp32Sessions,\n  sendAudioChunkToEsp32,\n  sendControlMessageToEsp32,\n  sendAudioCancelToEsp32,\n} from '../../websocket/esp32.gateway.js';"
new_gw_import = "import {\n  getActiveEsp32Session,\n  getAllActiveEsp32Sessions,\n  sendAudioChunkToEsp32,\n  sendControlMessageToEsp32,\n  sendAudioCancelToEsp32,\n  esp32AudioEvents,\n  waitForEsp32AudioReady,\n} from '../../websocket/esp32.gateway.js';"
assert old_gw_import in text, "old_gw_import not found"
text = text.replace(old_gw_import, new_gw_import, 1)

idx1 = text.find('    // 1. Notify ESP32 that audio playback is starting')
idx2 = text.find("sendControlMessageToEsp32(deviceId, 'audio:response:end',")
assert idx1 != -1 and idx2 != -1, "Slice markers not found"

new_slice = """    // 1. Notify ESP32 that audio playback is starting
    sendControlMessageToEsp32(deviceId, 'audio:response:start', {
      sessionId: playSessionId,
      format: audio.format || AudioFormat.PCM_16BIT,
      sampleRate: 24000,
      channels: audio.channels || 1,
      bitDepth: audio.bitDepth || 16,
      fileName: audio.fileName,
      totalSize: audio.fileSize,
    });

    // 2. Handshake: Wait for ESP32 Audio Engine to flush previous audio and acknowledge readiness
    const isReady = await waitForEsp32AudioReady(deviceId, playSessionId, 400);
    logger.info({ deviceId, playSessionId, isReady }, 'ESP32 audio readiness confirmed');

    // 3. Update robot state to 'speaking'
    targetSession.state = 'speaking';
    dashboardBroadcast.emitRobotState(deviceId, userId, 'speaking', playSessionId);

    // 4. Open GridFS download stream with watermark flow control
    const downloadStream = this.repo.getDownloadStreamFromGridFS(audio.storageRef);

    (async () => {
      let isPaused = false;
      const onFlow = (status: string) => {
        isPaused = (status === 'pause');
      };
      esp32AudioEvents.on(`flow:${deviceId}`, onFlow);

      try {
        let isFirst = true;
        let sentChunks = 0;
        const CHUNK_SIZE = 960; // 960 bytes = 480 samples = 20.0ms at 24kHz 16-bit mono PCM
        const sRate = audio.sampleRate || 24000;

        for await (const chunk of downloadStream) {
          if (targetSession.state !== 'speaking') break; // stopped by user or interruption

          let pcmChunk = chunk as Buffer;
          if (isFirst && pcmChunk.length > 44 && pcmChunk.toString('ascii', 0, 4) === 'RIFF') {
            pcmChunk = pcmChunk.subarray(44);
          }
          isFirst = false;

          // Resample any non-24kHz source on the fly to 24kHz
          if (sRate !== 24000) {
            pcmChunk = resamplePcmTo24k(pcmChunk, sRate);
          }

          for (let offset = 0; offset < pcmChunk.length; offset += CHUNK_SIZE) {
            if (targetSession.state !== 'speaking') break;

            // Watermark flow control: if buffer is 75% full, hold sending until resumed (max 2.5s)
            let waitSteps = 0;
            while (isPaused && waitSteps < 125) {
              await new Promise((r) => setTimeout(r, 20));
              waitSteps++;
            }

            const slice = pcmChunk.subarray(offset, Math.min(offset + CHUNK_SIZE, pcmChunk.length));
            const sent = sendAudioChunkToEsp32(deviceId, slice);
            if (!sent) break;

            sentChunks++;
            // Adaptive flow-control:
            // First 75 chunks (~1.5s of audio) burst at 8ms delay to prefill the Octal PSRAM RingBuffer cushion.
            // Chunks 76+ stream at exactly 20ms matching 24kHz physical consumption to prevent buffer overflow.
            const delayMs = (sentChunks < 75) ? 8 : 20;
            await new Promise((r) => setTimeout(r, delayMs));
          }
        }

        logger.info({ audioId, deviceId }, 'Completed audio playback stream to ESP32 robot');
        """

text = text[:idx1] + new_slice + text[idx2:]

# Also add cleanup of onFlow listener in finally block or error block
old_err_block = """      } catch (err) {
        logger.error({ err, audioId, deviceId }, 'Error streaming audio from GridFS to robot');
        sendControlMessageToEsp32(deviceId, 'audio:cancel', { reason: 'stream_read_error' });
        targetSession.state = 'idle';
        dashboardBroadcast.emitRobotState(deviceId, userId, 'idle', playSessionId);
      }
    })();"""

new_err_block = """      } catch (err) {
        logger.error({ err, audioId, deviceId }, 'Error streaming audio from GridFS to robot');
        sendControlMessageToEsp32(deviceId, 'audio:cancel', { reason: 'stream_read_error' });
        targetSession.state = 'idle';
        dashboardBroadcast.emitRobotState(deviceId, userId, 'idle', playSessionId);
      } finally {
        esp32AudioEvents.off(`flow:${deviceId}`, onFlow);
      }
    })();"""

assert old_err_block in text, "old_err_block not found"
text = text.replace(old_err_block, new_err_block, 1)

with open(audio_service_path, 'w', encoding='utf-8') as f:
    f.write(text)
print('Updated audio.service.ts successfully')

# 2. Update gemini-live.client.ts
with open(gemini_path, 'r', encoding='utf-8') as f:
    text = f.read()

old_g_import = "import {\n  getActiveEsp32Session,\n  sendAudioChunkToEsp32,\n  sendControlMessageToEsp32,\n} from '../../websocket/esp32.gateway.js';"
new_g_import = "import {\n  getActiveEsp32Session,\n  sendAudioChunkToEsp32,\n  sendControlMessageToEsp32,\n  waitForEsp32AudioReady,\n} from '../../websocket/esp32.gateway.js';"
assert old_g_import in text, "old_g_import not found"
text = text.replace(old_g_import, new_g_import, 1)

old_g_start = """              sendControlMessageToEsp32(this.configData.deviceId, 'audio:response:start', {
                sessionId: this.configData.sessionId,
                format: 'pcm_16bit',
                sampleRate: 24000, // Gemini Live standard output is 24kHz PCM
                channels: 1,
              });

              const esp32Session = getActiveEsp32Session(this.configData.deviceId);
              if (esp32Session) {
                esp32Session.state = 'speaking';
              }

              dashboardBroadcast.emitRobotState(
                this.configData.deviceId,
                this.configData.userId,
                'speaking',
                this.configData.sessionId
              );
            }

            // Stream PCM chunk to ESP32 speaker as binary frame
            sendAudioChunkToEsp32(this.configData.deviceId, pcmBuffer);"""

new_g_start = """              sendControlMessageToEsp32(this.configData.deviceId, 'audio:response:start', {
                sessionId: this.configData.sessionId,
                format: 'pcm_16bit',
                sampleRate: 24000, // Gemini Live standard output is 24kHz PCM
                channels: 1,
              });

              const esp32Session = getActiveEsp32Session(this.configData.deviceId);
              if (esp32Session) {
                esp32Session.state = 'speaking';
              }

              dashboardBroadcast.emitRobotState(
                this.configData.deviceId,
                this.configData.userId,
                'speaking',
                this.configData.sessionId
              );

              // Handshake: wait briefly for ESP32 audio task to clear previous buffer
              await waitForEsp32AudioReady(this.configData.deviceId, this.configData.sessionId, 300);
            }

            // Stream PCM chunk to ESP32 speaker as binary frame
            sendAudioChunkToEsp32(this.configData.deviceId, pcmBuffer);"""

assert old_g_start in text, "old_g_start not found"
text = text.replace(old_g_start, new_g_start, 1)

with open(gemini_path, 'w', encoding='utf-8') as f:
    f.write(text)
print('Updated gemini-live.client.ts successfully')
