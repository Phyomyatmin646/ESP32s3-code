import os

gemini_path = r'C:\Users\USER\Desktop\Ai_Robot_BE_CICD\apps\backend\src\modules\voice\gemini-live.client.ts'

with open(gemini_path, 'r', encoding='utf-8') as f:
    text = f.read()

# 1. Update imports
old_import = """import {
  sendAudioChunkToEsp32,
  sendAudioCancelToEsp32,
  sendControlMessageToEsp32,
  getActiveEsp32Session,
} from '../../websocket/esp32.gateway.js';"""

new_import = """import {
  sendAudioChunkToEsp32,
  sendAudioCancelToEsp32,
  sendControlMessageToEsp32,
  getActiveEsp32Session,
  waitForEsp32AudioReady,
} from '../../websocket/esp32.gateway.js';"""

assert old_import in text, "old_import not found"
text = text.replace(old_import, new_import, 1)

# 2. Add handshake wait before first PCM chunk
old_first_chunk = """              dashboardBroadcast.emitRobotState(
                this.configData.deviceId,
                this.configData.userId,
                'speaking',
                this.configData.sessionId
              );
            }

            // Stream PCM chunk to ESP32 speaker as binary frame
            sendAudioChunkToEsp32(this.configData.deviceId, pcmBuffer);"""

new_first_chunk = """              dashboardBroadcast.emitRobotState(
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

assert old_first_chunk in text, "old_first_chunk not found"
text = text.replace(old_first_chunk, new_first_chunk, 1)

with open(gemini_path, 'w', encoding='utf-8') as f:
    f.write(text)

print('Updated gemini-live.client.ts successfully')
