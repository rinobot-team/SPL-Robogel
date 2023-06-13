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
#include "NaoCamera.hpp"
// #include "utils/Logger.hpp"
// #include "utils/speech.hpp"
#include "utils/Timer.hpp"
// #include "blackboard/Blackboard.hpp"

using namespace std;

#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480

int main(int argc, char* argv[]){



        NaoCamera *cam = new NaoCamera("/dev/video-bottom", IO_METHOD_MMAP, kVGA);

        const uint8_t *buffer = cam->get(V4L2_PIX_FMT_YUYV);

        int bufferSize = IMAGE_WIDTH * IMAGE_HEIGHT * 3;
        cout << "Tamanho do buffer: " << bufferSize << " bytes" << endl; //imprimindo tamanho do buffer

        for (int j = 0; j < bufferSize; j++) {
            cout << "Valor do buffer[" << j << "]: " << static_cast<int>(buffer[j]) << endl;
        }


    delete cam;

    return 0;
}