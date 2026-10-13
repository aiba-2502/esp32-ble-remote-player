#pragma once

#include <stdint.h>

struct Frame;

int32_t getFrames(Frame* frame, int32_t frameCount);
void printPlaybackInfo();
void triggerPlayback();
bool isPlaybackActive();
bool consumePlaybackRequest();
