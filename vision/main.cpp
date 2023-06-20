/*
This is the main file of the image processing system.
Make any non-algorithm changes here.
*/
#include "lib/lodepng.hpp"
#include "src/yuv422ToRGB.hpp"
#include "src/RGBToPNG.hpp"
#include <malloc.h>
#include <limits.h>
#include <stdio.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <cassert>
#include <cerrno>
#include <cstring>
#include <ctime>
#include <iostream>
#include <stdexcept>
#include "camera/NaoCamera.hpp"
// #include "utils/Logger.hpp"
// #include "utils/speech.hpp"
#include "utils/Timer.hpp"
// #include "blackboard/Blackboard.hpp"

using namespace std;

#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480

int main(int argc, char* argv[]){

    NaoCamera *cam= new NaoCamera("/dev/video-bottom", IO_METHOD_MMAP, kVGA);
    const uint8_t *camBuffer = cam->get(V4L2_PIX_FMT_YUYV);

    //uint8_t *imageBuffer = const_cast<uint8_t*>(camBuffer);
    int bufferSize = static_cast<int>(IMAGE_WIDTH) * static_cast<int>(IMAGE_HEIGHT);
    cout << "Buffers Size: " << bufferSize << " bytes" << endl;
    uint8_t *imageBuffer = new uint8_t[bufferSize];
    memcpy(imageBuffer, camBuffer, bufferSize);
    //memcpy(imageBuffer, camBuffer, bufferSize);
    /*
    for (int j = 0; j < bufferSize; j++) {
        cout << "BufferValues[" << j << "]: " << static_cast<int>(camBuffer[j]) << endl;
    }
    */

    // Converting from YUV422 to RGB32
    //uint32_t *RGBBuffer = new uint32_t[bufferSize * 4];
    //uint32_t *RGBABuffer = new uint32_t[bufferSize * 4];
    uint32_t RGBABuffer[bufferSize * 4];
    yuv422_to_rgb32(imageBuffer, RGBABuffer, IMAGE_WIDTH, IMAGE_HEIGHT);
    // Expanding to RGBA
    //convertoToRGBA(RGBBuffer, bufferSize * 4);
    //convertToRGBA(RGBBuffer, RGBABuffer, bufferSize * 4);

    for (int i = 0; i < bufferSize / 2; i++) {
        cout << "RGBBuffer Values["<< i << "]: " << (RGBABuffer[i]) << endl;
    }

    // Generating PNG Image
    generatePNG(generateImageName(), RGBABuffer, IMAGE_WIDTH, IMAGE_HEIGHT);

    delete cam;

    return 0;
}