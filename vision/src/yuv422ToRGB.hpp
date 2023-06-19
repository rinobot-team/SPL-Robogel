/*
YUV is a color format represented as:

Y bytes (length is width * height) that represent luma (brightness)
U and V bytes (length is width * height / 2) that represent the color value

Every U and V bytes goes with 2 Y bytes. There is 2 * much brightness info than color.

*/

#ifndef _YUV422TORGB_H
#define _YUV422TORGB_H

#include <stdint.h>

void yuv422_to_rgb32(const uint8_t *image_yuv422, uint8_t *rgbaData, int width, int height);

void yuv422_to_rgb32_int(uint8_t *image_yuv422, uint32_t *image_rgb32, int width, int height);

#endif