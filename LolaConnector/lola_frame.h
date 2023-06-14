#pragma once

#include <map>
#include <string>
#include <vector>

#include <msgpack.hpp>

#include <battery.h>
#include <fsr.h>
#include <imu.h>
#include <joints.h>
#include <leds.h>

struct LolaSensorFrame {
    Joints joints;
    IMU imu;
    FSR fsr;
    Battery battery;
};

struct LolaActuatorFrame {
    Joints joints;
    Leds leds;
};

class LolaFrameHandler {
public:
    LolaFrameHandler();
    const LolaSensorFrame& unpack(const char* const buffer, size_t size);
    std::pair<char*, size_t> pack();

    LolaActuatorFrame actuator_frame;

private:
    void initSensorFrame();
    void initActuatorFrame();

    LolaSensorFrame sensor_frame;
    std::map<std::string, std::vector<float*>> sensor_frame_positions;
    std::map<std::string, std::vector<float*>> actuator_frame_positions;
    msgpack::sbuffer buffer;
};
