#include "yuvToRGBA.hpp"

void convertYuvToRGBA(uint8_t* bufferYuv, uint8_t* bufferRGBA, int width, int height){
    for (int h = 0; h < height; h++) {
        for (int w = 0; w < width; w += 2) {
            int yIndex = h * width + w * 2;
            int uIndex = yIndex + 1;
            int vIndex = yIndex + 3;
            int rgbaIndex = h * width * 4 + w * 4;
            
            int y0 = bufferYuv[yIndex];
            int u = bufferYuv[uIndex];
            int y1 = bufferYuv[yIndex + 2];
            int v = bufferYuv[vIndex];

            // Converting to RGBA
            // I dont understand the math behind it. Hope it works!
            int r0 = y0 + (1.370705 * (v - 128));
            int g0 = y0 - (0.698001 * (v - 128)) - (0.337633 * (u - 128));
            int b0 = y0 + (1.732446 * (u - 128));

            int r1 = y1 + (1.370705 * (v - 128));
            int g1 = y1 - (0.698001 * (v - 128)) - (0.337633 * (u - 128));
            int b1 = y1 + (1.732446 * (u - 128));

            // Assigning the RGBA values to the output buffer
            bufferRGBA[rgbaIndex] = clamp(r0);
            bufferRGBA[rgbaIndex + 1] = clamp(g0);
            bufferRGBA[rgbaIndex + 2] = clamp(b0);
            bufferRGBA[rgbaIndex + 3] = 255; // Max value to alpha channel (100% solid colour)

            bufferRGBA[rgbaIndex + 4] = clamp(r1);
            bufferRGBA[rgbaIndex + 5] = clamp(g1);
            bufferRGBA[rgbaIndex + 6] = clamp(b1);
            bufferRGBA[rgbaIndex + 7] = 255;
        }
    } 
}

int clamp(int value) {
    if (value < 0){
        return 0;
    } else if (value > 255) {
        return 255;
    } else {
        return value;
    }
}