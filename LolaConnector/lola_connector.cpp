#include "lola_connector.h"

#include <cmath>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <map>

#include <boost/asio.hpp>
#include <msgpack.hpp>

#include <joints.h>
#include <lola_frame.h>

using namespace boost::asio;
using namespace std;

static bool lola_shutdown = false;

void ctrlc_handler(int) {
    lola_shutdown = true;
}

int main(int, char*[]) {
    // Register some handlers so we can clean-up when we're killed.
    signal(SIGINT, ctrlc_handler);
    signal(SIGTERM, ctrlc_handler);

    io_service io_service;
    local::stream_protocol::socket socket(io_service);
    socket.connect("/tmp/robocup");

    constexpr int max_len = 100000;
    char data[max_len] = {'\0'};
    boost::system::error_code ec;
    LolaFrameHandler frame_handler;
    while (true) {
        const LolaSensorFrame& sensor_frame = frame_handler.unpack(data, socket.receive(boost::asio::buffer(data, max_len)));
        // Print accelerometer values as an example.
        printf("accel:\t%.3f\t%.3f\t%.3f\n", sensor_frame.imu.accel.x, sensor_frame.imu.accel.y, sensor_frame.imu.accel.z);
        auto& joints = frame_handler.actuator_frame.joints;
        // Set the head pitch to something.
        //TODO: Insert walking engine and stuff here. :)
        joints.head[HeadPitch] = {.angle = 0.3f, .stiffness = 1.f};
        char* buffer;
        size_t size;
        tie(buffer, size) = frame_handler.pack();
        socket.send(boost::asio::buffer(buffer, size));
        if (lola_shutdown) {
            // When finishing, set all stiffnesses to -1. Also it's necessary to get another packet otherwise we can't send.
            socket.receive(boost::asio::buffer(data, max_len));
            set_stiffness(-1.f, &frame_handler.actuator_frame.joints.legs);
            set_stiffness(-1.f, &frame_handler.actuator_frame.joints.arms);
            set_stiffness(-1.f, &frame_handler.actuator_frame.joints.head);
            tie(buffer, size) = frame_handler.pack();
            socket.send(boost::asio::buffer(buffer, size));
            break;
        }
    }
    return 0;
}
