/*
This is the main file of the image processing system.
Make any non-algorithm changes here.
*/
#include "./lib/lodepng.hpp"
#include "./src/yuv422ToRGB.hpp"
#include "./src/RGBToPNG.hpp"
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
    int bufferSize = IMAGE_WIDTH * IMAGE_HEIGHT * 3;

    cout << "Buffers Size: " << bufferSize << " bytes" << endl;

    uint8_t *imageBuffer;
    memcpy(imageBuffer, camBuffer, bufferSize);

    for (int j = 0; j < bufferSize; j++) {
        cout << "BufferValues[" << j << "]: " << static_cast<int>(camBuffer[j]) << endl;
    }

    
    // Converting from YUV422 to RGB32
    uint32_t RGBBuffer[bufferSize * 4];
    yuv422_to_rgb32_int(imageBuffer, RGBBuffer, IMAGE_WIDTH, IMAGE_HEIGHT);

    // Expanding to RGBA
    convertoToRGBA(RGBBuffer);

    /*
    // Generating PNG Image
    generatePNG(generateImageName(), RGBBuffer, IMAGE_WIDTH, IMAGE_HEIGHT);
    */

    delete cam;

    return 0;
}