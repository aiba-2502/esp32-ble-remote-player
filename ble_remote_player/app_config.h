#pragma once

#include <stdint.h>

constexpr const char* SPEAKER_NAME = "C06";
constexpr const char* REMOTE_NAME  = "vyviyon";  // 前方一致、大文字小文字は無視する

// スピーカーが見つからなくてもリモコンを使えるよう、一定時間で BLE を始める。
constexpr uint32_t BLE_START_TIMEOUT_MS = 30000;

// AVRCP は A2DP より少し遅れて繋がるため、接続直後の音量設定を待つ。
constexpr uint32_t VOLUME_DELAY_MS = 2000;
constexpr uint8_t  VOLUME          = 100;  // 0〜127。ESP32 とスピーカー本体の両方に効く

// C06 の無音ミュートを防ぐ待機信号。振幅は実機に合わせて調整する。
constexpr uint32_t KEEPALIVE_HZ        = 30;
constexpr int16_t  KEEPALIVE_AMPLITUDE = 100;  // 約 -50 dBFS。0 で完全な無音

// 再接続を押下として再生した直後に届く入力レポートの二重発火を防ぐ。
constexpr uint32_t WAKE_PRESS_GUARD_MS = 1000;

constexpr int LED_PIN = 2;
