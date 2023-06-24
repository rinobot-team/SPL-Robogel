#include "yuvToRGBA.hpp"
#include <iostream>

using namespace std;
void convertYuvToRGBA(uint8_t* bufferYuv, uint8_t* bufferRGBA, int width, int height){

    // UYVY
    int size = width * height;
    
    cout << "hello!" << endl;
    for (int i = 0, j = 0; i < size * 2; i += 4, j += 8) {
        // int u = bufferYuv[i];
        // int y0 = bufferYuv[i + 1];
        // int v = bufferYuv[i + 2];
        // int y1 = bufferYuv[i + 3];

        int y0 = bufferYuv[i + 0];
        int u = bufferYuv[i + 1];
        int y1 = bufferYuv[i + 2];
        int v = bufferYuv[i + 3];
        if(i % 100 == 0){
            cout << "y0 = " << y0  << "("  << i << ")"<< endl;
            cout << "v = " << v << endl;
        }
    
        int d = u - 128;
        int f = v - 128;
    
        bufferRGBA[j + 0] = clamp(y0 + 1.402 * f);
        bufferRGBA[j + 1] = clamp(y0 - 0.344 * d - 0.714 * f);
        bufferRGBA[j + 2] = clamp(y0 + 1.772 * d);
        bufferRGBA[j + 3] = 255;

        bufferRGBA[j + 4] = clamp(y1 + 1.402 * f);
        bufferRGBA[j + 5] = clamp(y1 - 0.344 * d - 0.714 * f);
        bufferRGBA[j + 6] = clamp(y1 + 1.772 * d);
        bufferRGBA[j + 7] = 255;

    }
    
}

int clamp(float value) {
    if (value < 0){
        return 0;
    } else if (value > 255) {
        return 255;
    } else {
        return value;
    }
}
