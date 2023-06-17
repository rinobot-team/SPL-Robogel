#include "RGBToPNG.hpp"
#include <iomanip>
#include <ctime>

void convertoToRGBA(uint32_t *image_rgb32, int rgbaBufferSize) {
    uint32_t image_rgba32[rgbaBufferSize];

    image_rgba32[0] = image_rgb32[0];
    image_rgba32[1] = image_rgb32[1];
    image_rgba32[2] = image_rgb32[2];
    image_rgba32[3] = ALPHA;
}

void convertToRGBA(uint32_t* image_rgb32, uint32_t* image_rgba32, int size) {
    for (int i = 0; i < size; i++) {
        uint32_t rgb_pixel = image_rgb32[i];

        // Extrai os componentes RGB do pixel
        uint8_t red = (rgb_pixel >> 16) & 0xFF;
        uint8_t green = (rgb_pixel >> 8) & 0xFF;
        uint8_t blue = rgb_pixel & 0xFF;

        // Combina os componentes RGB com um componente de alfa fixo (255 para opacidade total)
        uint32_t rgba_pixel = (red << 24) | (green << 16) | (blue << 8) | 0xFF;

        // Armazena o pixel RGBA no array de saída
        image_rgba32[i] = rgba_pixel;
    }
}


void generatePNG(const char* fileName, const uint32_t *image_rgba32, int width, int height){
    std::vector<unsigned char> image(image_rgba32, image_rgba32 + (width * height * 4));
    
    unsigned error = lodepng::encode(fileName, image, width, height);
    error ? printf("Error trying to save PNG %s\n", lodepng_error_text(error)) : printf("PNG image saved as \"%s\"\n", fileName);
}

char* generateImageName(){
    // Fetching current time and local time
    std::time_t curTime = std::time(nullptr);
    std::tm* localTime = std::localtime(&curTime);

    // Formating date and time strings
    std::stringstream ss;
    ss << std::put_time(localTime, "%Y%m%d-%H%M%S");

    // Copying from string to char*
    std::string timeStr = ss.str();
    char* timeChar = new char[timeStr.size() + 1];
    std::strcpy(timeChar, timeStr.c_str());

    return timeChar;
}