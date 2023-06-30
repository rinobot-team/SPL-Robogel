#include "LolaConnector/lola_connector.h"
#include "HTWKMotion/ankle_balancer.h"
#include "HTWKMotion/arm_controller.h"
#include "HTWKMotion/sit_motion.h"
#include "HTWKMotion/walking_engine.h"
#include "utils/imu.h"
#include "LolaConnector/fila.h"
#include "nao.h"

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

	//Lola's State
	static bool lola_shutdown = false;
	bool lola_sit_forever = false;

	float filterX = 0, filterY = 0, filterZ = 0, filterP = 0, filterR = 0;
	float last_filterX = 0, last_filterY = 0, last_filterZ = 0, last_filterPitch = 0, last_filterRoll = 0; 

void freshFilter(Fila dadoX, Fila dadoY, Fila dadoZ, Fila dadoP, Fila dadoR, IMU imu){
	//Filtro Accel
	last_filterX = filterX;
	last_filterY = filterY;
	last_filterZ = filterZ;
	dadoX.freshFila(imu.accel.x);
	dadoY.freshFila(imu.accel.y);
	dadoZ.freshFila(imu.accel.z);
	filterX = dadoX.filter();
	filterY = dadoY.filter();
	filterZ = dadoZ.filter();
}

void ctrlc_handler(int) {
	nao.setState(Lola_state::sit);
}

int main(int, char*[]) {
	NAO nao;

	Fila dadoX, dadoY, dadoZ, dadoP, dadoR;
	constexpr int max_len = 100000;
	char data[max_len] = {'\0'};
	boost::system::error_code ec;
	LolaFrameHandler frame_handler;

	signal(SIGINT, ctrlc_handler);
	signal(SIGTERM, ctrlc_handler);

	io_service io_service;
	local::stream_protocol::socket socket(io_service);

	socket.connect("/tmp/robocup");

	int pra_frente = 0;
	int pro_lado = 0;

	while (true) {
		// Declarações / Atribuições de sensores
		const LolaSensorFrame& sensor_frame = frame_handler.unpack(data, socket.receive(boost::asio::buffer(data, max_len)));
		//Leds& leds = frame_handler.actuator_frame.leds; // Leds
		auto& battery = sensor_frame.battery; // Bateria
		auto& fsr = sensor_frame.fsr; // FSR: sensores dos pés
		auto& imu = sensor_frame.imu; // IMU: Accel e GYR
	
		nao.fresh(imu);

		// Vetor dos FSR
		float fsrR[4] = {fsr.right.fl, fsr.right.fr, fsr.right.rl, fsr.right.rr};
		float fsrL[4] = {fsr.left.fl, fsr.left.fr, fsr.left.rl, fsr.left.rr};

		char* buffer;
		size_t size;
		tie(buffer, size) = frame_handler.pack();
		socket.send(boost::asio::buffer(buffer, size));
		
		//TODO: Insert walking engine and stuff here. :)
		switch (nao.getState()){
			case Lola_state::begin:
				nao.standingBegin(sensor_frame);
			break;
			case Lola_state::standingBegin:
				if(!nao.standingBegin(sensor_frame)){
					nao.isStand(fsrR, fsrL);
				}
			break;
			case Lola_state::stand:
				nao.stand(sensor_frame, fsrR, fsrL);
			break;
			case Lola_state::fallingF:
				nao.falling(fsrR, fsrL);
			break;
			case Lola_state::fallingB:
				nao.falling(fsrR, fsrL);
			break;
			case Lola_state::fallen:
				nao.fallen();
			break;
			case Lola_state::standingF:
				nao.standing();
			break;
			case Lola_state::standingB:
				nao.standing();
			break;
			case Lola_state::sit:
				nao.sit(sensor_frame);
			break;
			case Lola_state::shutdown:
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