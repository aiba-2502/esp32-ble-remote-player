#include "playback.h"

#include "app_config.h"
#include "sound.h"

#include "BluetoothA2DPSource.h"

#include <Arduino.h>
#include <math.h>

namespace {

struct PlaybackState {
  volatile bool     playing      = false;
  volatile uint32_t position     = 0;
  volatile bool     pressPending = false;
};

PlaybackState playback;

static_assert(SOUND_SAMPLE_RATE % KEEPALIVE_HZ == 0,
              "1 周期がサンプル数で割り切れないと波形の継ぎ目でノイズが出る");

}  // namespace

void printPlaybackInfo() {
  Serial.printf("音源 %lu フレーム / 約 %.1f 秒\n",
                static_cast<unsigned long>(SOUND_FRAMES),
                SOUND_FRAMES / static_cast<float>(SOUND_SAMPLE_RATE));
}

void triggerPlayback() {
  playback.position     = 0;
  playback.playing      = true;
  playback.pressPending = true;
}

int32_t getFrames(Frame* frame, int32_t frameCount) {
  static const uint32_t period = SOUND_SAMPLE_RATE / KEEPALIVE_HZ;
  static uint32_t       phase  = 0;

  for (int32_t i = 0; i < frameCount; i++) {
    uint32_t pos = playback.position;
    int16_t  value;
    if (playback.playing && pos < SOUND_FRAMES) {
      value             = soundData[pos];
      playback.position = pos + 1;
    } else {
      playback.playing = false;
      value = KEEPALIVE_AMPLITUDE * sinf(2.0f * PI * phase / period);
      phase = (phase + 1) % period;
    }
    frame[i].channel1 = value;
    frame[i].channel2 = value;
  }
  return frameCount;
}

bool isPlaybackActive() {
  return playback.playing;
}

bool consumePlaybackRequest() {
  if (!playback.pressPending) return false;

  playback.pressPending = false;
  return true;
}
