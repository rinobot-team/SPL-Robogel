#ifndef RGBATOPNG_H
#define RGBATOPNG_H

#include "lib/lodepng.hpp"
#include <iostream>
#include <ctime>
#include <iomanip>
#include <cstring>

void saveRGBAtoPNG(const uint8_t* bufferRGBA, int width, int height, char* fileName);

char* generateFileName(int count);

#endif