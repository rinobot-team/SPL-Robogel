#ifndef _RGBTOPNG_H
#define _RGBTOPNG_H

#include "lodepng.hpp"
#include <stdio.h>

#define ALPHA 255
#define DEFAULT_WIDTH 520
#define DEFAULT_HEIGHT 520

void convertoToRGBA(uint32_t *image_rgb32);

void generatePNG(const char* fileName, const uint32_t *image_rgba32, int width, int height);

#endif