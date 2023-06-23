#include "rgbaToPNG.hpp"

void saveRGBAtoPNG(const uint8_t* bufferRGBA, int width, int height, char* fileName) {
    std::vector<unsigned char> image;
    image.reserve(width * height * 4);

    // Copying the values from the RGBA buffer to the image vec
    for (int i = 0; i < width * height * 4; i++) {
        image.push_back(bufferRGBA[i]);
    }

    unsigned error = lodepng::encode(fileName, image, width, height);
    if(error)
        std::cout << "Error while saving the image as PNG: \n" << lodepng_error_text(error) << std::endl;
}

char* generateFileName(int count) {
    std::time_t currTime = std::time(nullptr);
    std::tm* localTime = std::localtime(&currTime);

    std::stringstream ss;
    ss << std::put_time(localTime, "%Y-%m-%d::%H:%M:%S");
    std::string dateTimeStr = ss.str();
    dateTimeStr.append("::IMG-");
    dateTimeStr.append(std::to_string(count));
    dateTimeStr.append(".png");
    char* dateTimeChar = new char[dateTimeStr.size() + 1];
    std::strcpy(dateTimeChar, dateTimeStr.c_str());

    return dateTimeChar;
}