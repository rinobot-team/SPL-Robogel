#include "RGBToPNG.hpp"

void convertoToRGBA(uint32_t *image_rgb32) {
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