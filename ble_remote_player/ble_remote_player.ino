// BLE シャッターリモコンのボタンで WAV を Bluetooth スピーカーから鳴らす。
// ESP32 を A2DP ソースと BLE HID ホストとして同時に使う。

#include "app_config.h"
#include "playback.h"
#include "remote.h"
#include "speaker.h"

#include <Arduino.h>

namespace {

bool remoteStarted = false;

void startRemoteWhenReady() {
  // Classic の探索と BLE スキャンを同時に走らせない。スピーカーが見つからない
  // 場合もリモコンを使えるよう、一定時間後は接続を待たずに BLE を開始する。
  if (remoteStarted ||
      (!isSpeakerConnected() && millis() <= BLE_START_TIMEOUT_MS)) {
    return;
  }

  if (!isSpeakerConnected()) {
    Serial.println("スピーカー未接続のまま BLE を開始する");
  }
  startRemote();
  remoteStarted = true;
}

}  // namespace

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  printPlaybackInfo();
  startSpeaker();
}

void loop() {
  updateSpeaker();
  startRemoteWhenReady();
  updateRemote();

  digitalWrite(LED_PIN, isPlaybackActive() ? HIGH : LOW);
  delay(10);
}
