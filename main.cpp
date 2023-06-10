#include "LolaConnector/lola_connector.h"
#include "HTWKMotion/ankle_balancer.h"
#include "HTWKMotion/arm_controller.h"
#include "HTWKMotion/sit_motion.h"
#include "HTWKMotion/walking_engine.h"
#include "utils/imu.h"

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
	auto sit_motion = SitMotion();
	auto ankle_balancer = AnkleBalancer();
	auto arm_controller = ArmController();
	auto walking_engine = WalkingEngine();
	auto odo = Odometry();
	bool lola_sit_forever = false;
	// auto imu_filter = IMU();
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
		//TODO: Insert walking engine and stuff here. :)
		if (/*client_connected &&*/ !lola_shutdown && !lola_sit_forever) {
			if (!sit_motion.isStanding()) {
				joints.legs = sit_motion.getUp(sensor_frame.joints.legs, ankle_balancer, &arm_controller);
				joints.arms = arm_controller.proceed();
				walking_engine.reset();
				// TODO: You probably want to have a few else if statements here for things that should override the walking, e.g. getting up.
			} else {
				// joints.legs = walking_engine.proceed(sensor_frame.fsr, imu_filter.angles.pitch, imu_filter.angles.roll, ankle_balancer,
				joints.legs = walking_engine.proceed(sensor_frame.fsr, sensor_frame.imu.gyr.pitch, sensor_frame.imu.gyr.roll, ankle_balancer,
						sensor_frame.imu.gyr.yaw, &odo, &arm_controller);
				joints.arms = arm_controller.proceed();
			}
		} else {
			if (walking_engine.isStanding()) {
				joints.legs = sit_motion.sitDown(sensor_frame.joints.legs, ankle_balancer, &arm_controller);
			} else {
				// walking_engine.setRequest(0, 0, 0, 0, Shoot::NONE);
				walking_engine.setRequest(0, 0, 0, 0);
				joints.legs = walking_engine.proceed(sensor_frame.fsr, sensor_frame.imu.gyr.pitch, sensor_frame.imu.gyr.roll, ankle_balancer,
						sensor_frame.imu.gyr.yaw, &odo, &arm_controller);
			}
			joints.arms = arm_controller.proceed();
		}
		// Set the head pitch to something.
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
