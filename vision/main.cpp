/*
This is the main file of the image processing system.
Make any non-algorithm changes here.
*/
#include "lib/lodepng.hpp"
#include "src/yuvToRGBA.hpp"
#include "src/rgbaToPNG.hpp"
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
#include <fstream>
// #include "utils/Logger.hpp"
// #include "utils/speech.hpp"
#include "utils/Timer.hpp"
// #include "blackboard/Blackboard.hpp"

using namespace std;

#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480

int main(int argc, char* argv[]){

    NaoCamera *cam= new NaoCamera("/dev/video-top", IO_METHOD_MMAP, kVGA);

    const int bufferSizeYuv = IMAGE_WIDTH * IMAGE_HEIGHT * 2; // YUV allocates 2 bytes per pixel
    const int bufferSizeRGBA = IMAGE_WIDTH * IMAGE_HEIGHT * 4; // RGBA allocates 4 bytes per pixel

    int count = 0;
    while(count < 30) {
        // Copying the data of the camera buffer to the yuv buffer
        uint8_t* bufferYuv = new uint8_t[bufferSizeYuv];
        const uint8_t *camBuffer = cam->get(V4L2_PIX_FMT_YUYV);
        memcpy(bufferYuv, camBuffer, bufferSizeYuv);
        // for(int i = 0; i < bufferSizeYuv/100; i++){
            // cout << (int) bufferYuv[i] << ", "; 
        // }
        // cout << endl;
        // continue;

        /*
        std::string yuv_image_filename("imagem" + std::to_string(count) + ".yuv");
        std::ofstream yuv_file(yuv_image_filename, std::ios::binary);
        if (yuv_file.is_open()) {
		yuv_file.write(reinterpret_cast<const char*>(bufferYuv), bufferSizeYuv);
		yuv_file.close();
		std::cout << "yuv File written successfully." << std::endl;
        } else {
		std::cout << "Unable to open yuv file for writing." << std::endl;
        }
        */
        
        uint8_t* bufferRGBA = new uint8_t[bufferSizeRGBA];
        convertYuvToRGBA(bufferYuv, bufferRGBA, IMAGE_WIDTH, IMAGE_HEIGHT);
        
        std::string image_filename("imagem" + std::to_string(count) + ".rgba");
        std::ofstream file(image_filename, std::ios::binary);
        if (file.is_open()) {
		file.write(reinterpret_cast<const char*>(bufferRGBA), bufferSizeRGBA);
		file.close();
		std::cout << "File written successfully." << std::endl;
        } else {
		std::cout << "Unable to open file for writing." << std::endl;
        }

        saveRGBAtoPNG(bufferRGBA, IMAGE_WIDTH, IMAGE_HEIGHT, generateFileName(count));

        count++;
        delete [] bufferYuv;
        delete [] bufferRGBA;
    }

    delete cam;

    return 0;
}
