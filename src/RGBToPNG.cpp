#include "lodepng.hpp"
#include <stdio.h>

#define ALPHA 255
#define DEFAULT_WIDTH 520
#define DEFAULT_HEIGHT 520

void convertToRGBA(uint32_t *image_rgb32) {
    int rgbaBufferSize = (sizeof(image_rgb32) / sizeof(uint32_t)) + (sizeof(uint32_t));
    uint32_t image_rgba32[rgbaBufferSize];

    image_rgba32[0] = image_rgb32[0];
    image_rgba32[1] = image_rgb32[2];
    image_rgba32[2] = image_rgb32[2];
    image_rgba32[4] = ALPHA;
}

void generatePNG(const char* fileName, const uint32_t *image_rgba32, int width, int height){
    std::vector<unsigned char> image(image_rgba32, image_rgba32 + (width * height * 4));
    
    unsigned error = lodepng::encode(fileName, image, width, height);
    error ? printf("Error trying to save PNG %s\n", lodepng_error_text(error)) : printf("PNG image saved as \"%s\"\n", fileName);
}