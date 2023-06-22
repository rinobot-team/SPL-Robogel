#ifndef YUVTORGB_H
#define YUVTORGB_H

#include <cstdint>

void convertYuvToRGBA(uint8_t* bufferYuv, uint8_t* bufferRGBA, int width, int height);

int clamp(int value);

#endif