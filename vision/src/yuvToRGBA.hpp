#ifndef YUVTORGB_H
#define YUVTORGB_H

#include <cstdint>

void convertYuvToRGBA(uint8_t* bufferYuv, uint8_t* bufferRGBA, int width, int height);
// void yuv422ToRgba(std::vector<uint8_t> &out, const uint8_t *const in, const uint32_t width, const uint32_t height) const;

int clamp(float value);

#endif
