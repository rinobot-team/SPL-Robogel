#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "yuv422ToRGB.hpp"

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