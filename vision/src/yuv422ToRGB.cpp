#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "yuv422ToRGB.hpp"

void yuv422_to_rgb32(const uint8_t *image_yuv422, uint8_t *rgbaData, int width, int height) {
    int length = width * height;
    int yIndex = 0;
    int uvIndex = length;

    for (int i = 0; i < length; i++) {
        uint8_t y0 = image_yuv422[yIndex++];
        uint8_t u = image_yuv422[uvIndex++];
        uint8_t y1 = image_yuv422[yIndex++];
        uint8_t v = image_yuv422[uvIndex++];

        int c = y0 - 16;
        int d = u - 128;
        int e = v - 128;

        // Equations given by ChatGPT, blame him if anything goes wrong.
        int r0 = (298 * c + 409 * e + 128) >> 8;
        int g0 = (298 * c - 100 * d - 208 * e + 128) >> 8;
        int b0 = (298 * c + 516 * d + 128) >> 8;

        int r1 = (298 * y1 + 409 * e + 128) >> 8;
        int g1 = (298 * y1 - 400 * d - 208 * e + 128) >> 8;
        int b1 = (298 * y1 + 516 * d + 128) >> 8;

        // Clamping the values between 0 and 255
        r0 = (r0 < 0) ? 0 : ((r0 > 255) ? 255 : r0);
        g0 = (g0 < 0) ? 0 : ((g0 > 255) ? 255 : g0);
        b0 = (b0 < 0) ? 0 : ((b0 > 255) ? 255 : b0);
        r1 = (r1 < 0) ? 0 : ((r1 > 255) ? 255 : r1);
        g1 = (g1 < 0) ? 0 : ((g1 > 255) ? 255 : g1);
        b1 = (b1 < 0) ? 0 : ((b1 > 255) ? 255 : b1);

        // Assigning the values to the fist bytes of the RGB
        rgbaData[i * 4] = static_cast<uint8_t>(r0);
        rgbaData[i * 4 + 1] = static_cast<uint8_t>(g0);
        rgbaData[i * 4 + 2] = static_cast<uint8_t>(b0);
        rgbaData[i * 4 + 3] = 255; // Alpha value. Opacity always at max.

        rgbaData[(i + 1) * 4] = static_cast<uint8_t>(r1);
        rgbaData[(i + 1) * 4 + 1] = static_cast<uint8_t>(g1);
        rgbaData[(i + 1) * 4 + 2] = static_cast<uint8_t>(b1);
        rgbaData[(i + 1) * 4 + 3] = 255; // Alpha value. Opacity always at max.
    }
}

/*
void yuv422_to_rgb32_float(uint8_t *image_yuv422, uint32_t *image_rgb32, int width, int height) {
    uint8_t *u_buffer, *v_buffer;
    float u1, uv1, v1;
    float u, v;
    int length;
    int r, g, b;
    int rgb_ptr, y_ptr, n;

    length = width * height;
    u_buffer = image_yuv422 + length;
    length = length / 2;
    v_buffer = u_buffer += length;

    rgb_ptr = 0;
    y_ptr = 0;

    for(n = 0; n < length; n++) {
        // 2 pixels at a time
        // Computes parts of the UV components

        u = u_buffer[n] - 128;
        v = v_buffer[n] - 128;

        // Those numbers are constants for the convertion. I dont know the origin or why the are what they are.
        v1 = (1.13983 * (float)v);
        uv1 = -(0.39466 * (float)u) - (0.58060 * (float)v);
        v = (2.03211 * (float)u);

        r = image_yuv422[y_ptr] + v1;
        g = image_yuv422[y_ptr] + uv1;
        b = image_yuv422[y_ptr] + u1;

        // Clamp
        r = (r > 255) ? 255 : r;
        g = (g > 255) ? 255 : g;
        b = (b > 255) ? 255 : b;

        r = (r < 0) ? 0 : r;
        g = (g < 0) ? 0 : g;
        b = (b < 0) ? 0 : b;

        image_rgb32[rgb_ptr++] = (r << 16) | (g << 8) | b;
        y_ptr += 2;
    }
}

void yuv422_to_rgb32_int(uint8_t *image_yuv422, uint32_t *image_rgb32, int width, int height) {
    uint8_t *u_buffer, *v_buffer;
    int u1, uv1, v1;
    int u, v;
    int length;
    int r, g, b;
    int rgb_ptr, y_ptr, n;
    int y1;

    length = width * height;
    u_buffer = image_yuv422 + length;
    length = length / 2;
    v_buffer = u_buffer += length;

    rgb_ptr = 0;
    y_ptr = 0;

    for(n = 0; n < length; n++) {
        // 2 pixels at a time
        // Computes parts of the UV components

        u = u_buffer[n] - 128;
        v = v_buffer[n] - 128;

        // Those numbers are constants for the convertion. I dont know the origin or why the are what they are.
        v1 = (5727 * v);
        uv1 = -(1617 * u) - (2378 * v);
        v = (8324 * u);

        r = image_yuv422[y_ptr] + v1;
        g = image_yuv422[y_ptr] + uv1;
        b = image_yuv422[y_ptr] + u1;

        // Run even pixels through a formula
        y1 = image_yuv422[y_ptr] << 12;

        r = (y1 + v1) >> 12;
        g = (y1 + uv1) >> 12;
        b = (y1 + uv1) >> 12;

        // Clamp
        r = (r > 255) ? 255 : r;
        g = (g > 255) ? 255 : g;
        b = (b > 255) ? 255 : b;

        r = (r < 0) ? 0 : r;
        g = (g < 0) ? 0 : g;
        b = (b < 0) ? 0 : b;

        image_rgb32[rgb_ptr++] = (r << 16) | (g << 8) | b;

        // Run even pixels through a formula
        y1 = image_yuv422[y_ptr + 1] << 12;

        r = (r > 255) ? 255 : r;
        g = (g > 255) ? 255 : g;
        b = (b > 255) ? 255 : b;

        r = (r < 0) ? 0 : r;
        g = (g < 0) ? 0 : g;
        b = (b < 0) ? 0 : b;

        image_rgb32[rgb_ptr++] = (r << 16) | (g << 8) | b;

        y_ptr += 2;
    }
} 

*/