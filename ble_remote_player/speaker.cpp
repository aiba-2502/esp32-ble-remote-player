#include "speaker.h"

#include "app_config.h"
#include "playback.h"

#include "BluetoothA2DPSource.h"

#include <Arduino.h>
#include <esp_avrc_api.h>
#include <string.h>

namespace {

BluetoothA2DPSource a2dpSource;

struct SpeakerState {
  bool     connected   = false;
  uint32_t connectedAt = 0;
  bool     volumeSent  = false;
};

SpeakerState speaker;

void updateConnection() {
  bool connected = a2dpSource.is_connected();
  if (connected == speaker.connected) return;

  speaker.connected = connected;
  if (connected) {
    speaker.connectedAt = millis();
    speaker.volumeSent  = false;  // 再接続のたびに音量を送り直す
    Serial.println("スピーカー接続");
  } else {
    Serial.println("スピーカー切断");
  }
}

void setVolumeWhenReady() {
  if (!speaker.connected || speaker.volumeSent ||
      millis() - speaker.connectedAt <= VOLUME_DELAY_MS) {
    return;
  }

  // set_volume() は値が変わらないと AVRCP を送らないため、本体へ直接送る。
  a2dpSource.set_volume(VOLUME);
  esp_avrc_ct_send_set_absolute_volume_cmd(0, VOLUME);
  speaker.volumeSent = true;
  Serial.printf("音量 %u\n", VOLUME);
}

void reconnectAfterPress() {
  if (!consumePlaybackRequest()) return;

  esp_bd_addr_t* last = a2dpSource.get_last_peer_address();
  static const esp_bd_addr_t none = {};
  if (!speaker.connected && memcmp(*last, none, sizeof(none)) != 0) {
    Serial.println("スピーカー未接続、接続を試みる");
    a2dpSource.connect_to(*last);
  }
}

}  // namespace

void startSpeaker() {
  // Classic と BLE の両方を使えるよう、BLE 用メモリを残す。
  a2dpSource.set_default_bt_mode(ESP_BT_MODE_BTDM);
  // 前回接続したアドレスへ直接再接続できるようにする。
  a2dpSource.set_auto_reconnect(true);
  a2dpSource.start(SPEAKER_NAME, getFrames);
  Serial.println("スピーカーを探しています");
}

void updateSpeaker() {
  updateConnection();
  setVolumeWhenReady();
  reconnectAfterPress();
}

bool isSpeakerConnected() {
  return speaker.connected;
}
