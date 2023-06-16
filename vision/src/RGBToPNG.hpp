#ifndef _RGBTOPNG_H
#define _RGBTOPNG_H

#include "../lib/lodepng.hpp"
#include <stdio.h>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <cstring>

#define ALPHA 255
#define DEFAULT_WIDTH 520
#define DEFAULT_HEIGHT 520

void convertoToRGBA(uint32_t *image_rgb32);

void generatePNG(const char* fileName, const uint32_t *image_rgba32, int width, int height);

char* generateImageName();

#endif