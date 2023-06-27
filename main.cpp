#include "LolaConnector/lola_connector.h"
#include "HTWKMotion/ankle_balancer.h"
#include "HTWKMotion/arm_controller.h"
#include "HTWKMotion/sit_motion.h"
#include "HTWKMotion/walking_engine.h"
#include "utils/imu.h"
#include "LolaConnector/fila.h"
#include "LolaConnector/state.h"

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

void ctrlc_handler(int) {
	lola_shutdown = true;
}

int main(int, char*[]) {
	//Declaração state
	NAO nao;

	// Declarações / Atribuições
	/*auto sit_motion = SitMotion();
	auto ankle_balancer = AnkleBalancer();
	auto arm_controller = ArmController();
	auto walking_engine = WalkingEngine();
	auto odo = Odometry();*/

	// Register some handlers so we can clean-up when we're killed.
	signal(SIGINT, ctrlc_handler);
	signal(SIGTERM, ctrlc_handler);

	io_service io_service;
	local::stream_protocol::socket socket(io_service);

	socket.connect("/tmp/robocup");

	Fila dadoX, dadoY, dadoZ, dadoP, dadoR;
	constexpr int max_len = 100000;
	char data[max_len] = {'\0'};
	boost::system::error_code ec;

	int pra_frente = 0;
	int pro_lado = 0;

	while (true) {
		// Declarações / Atribuições de sensores
		/*const LolaSensorFrame& sensor_frame = frame_handler.unpack(data, socket.receive(boost::asio::buffer(data, max_len)));
		auto& joints = frame_handler.actuator_frame.joints; //Juntas
		auto& leds = frame_handler.actuator_frame.leds; // Leds
		auto& battery = sensor_frame.battery; // Bateria
		auto& fsr = sensor_frame.fsr; // FSR: sensores dos pés
		auto& imu = sensor_frame.imu; // IMU: Accel e GYR

		// Vetor dos FSR
		float fsrR[4] = {fsr.right.fl, fsr.right.fr, fsr.right.rl, fsr.right.rr};
		float fsrL[4] = {fsr.left.fl, fsr.left.fr, fsr.left.rl, fsr.left.rr};*/

		nao.declara();

		// Atualizações do filtro
		/*freshFilter(dadoX, filterX, last_filterX, dadoY, filterY, last_filterY, dadoZ, filterZ, last_filterZ,
		dadoP, filterP, last_filterPitch, dadoR, filterR, last_filterRoll, imu); //Atualizar parâmetros conforme aumentar os filtros*/

		nao.freshFilter();

		//TODO: Insert walking engine and stuff here. :)
		if (/*client_connected &&*/ !lola_shutdown && !lola_sit_forever && (nao.getState() == Lola_state::begin || nao.getState() == Lola_state::standingBegin || nao.getState() == Lola_state::stand)){
			if (!sit_motion.isStanding()) {
				nao.standingBegin();
			} 
			else{
				lola_last_state = lola_state;
				lola_state = Lola_state::stand;
				cout << "Stand!" << endl;

				//joints.legs = walking_engine.proceed(sensor_frame.fsr, imu_filter.angles.pitch (0.1), imu_filter.angles.roll (0), ankle_balancer,

				joints.arms = arm_controller.proceed();
				joints.legs = walking_engine.proceed(sensor_frame.fsr, 0.1, 0, ankle_balancer,sensor_frame.imu.gyr.yaw, &odo, &arm_controller);

				nao.isFalling();
				
				if(nao.seguro()){
					cout << "No chao!" << endl;
					/*if(pra_frente < 400 && pro_lado == 0){
						walking_engine.setRequest(0.07, 0, 0, 0.1);
						pra_frente++;
					}
					else if(pro_lado < 400 && pra_frente == 400){
						walking_engine.setRequest(0.07, 0, 1, 0.1);
						pro_lado++;
					}
					else{
						joints.head[HeadPitch] = {.angle = 0.1f, .stiffness = 1.f};
						lola_sit_forever = true;
					}*/
				}
				else{
					cout << "Fora do chao!" << endl;
					walking_engine.reset();
				}
			}
		}
		//FALLING
		else if(lola_state == Lola_state::fallingF || lola_state == Lola_state::fallingB || lola_state == Lola_state::fallingR || lola_state == Lola_state::fallingL){
			if(lola_state == Lola_state::fallingF){
				cout << "Falling Front" << endl;
				//isfalling()
				fallen(imu, fsrR, fsrL, filterZ);
			}
			else if(lola_state == Lola_state::fallingB){
				cout << "Falling Back" << endl;
				fallen(imu, fsrR, fsrL, filterZ);
			}
			else if(lola_state == Lola_state::fallingR){
				cout << "Falling Right" << endl;
				fallen(imu, fsrR, fsrL, filterZ);
			}
			else{
				cout << "Falling Left" << endl;
				fallen(imu, fsrR, fsrL, filterZ);
			}
		}
		//FALLEN
		else if(lola_state == Lola_state::fallen){
			cout << "FALLEN!!!" << endl;
			//fallen();
			isStanding(imu, filterX, last_filterX, filterY, last_filterY, filterZ, last_filterZ);
		}
		//STANDING FRONT
		else if(lola_state == Lola_state::standingF){
			cout << "Standing Front" << endl;
			//standingFront()
			if(filterZ == -10 && seguro(fsrR, fsrL)){
				lola_last_state = lola_state;
				lola_state = Lola_state::stand;
			}
			else if(isFalling(imu, fsrR, fsrL, filterX, last_filterX, filterY, last_filterY, filterZ, last_filterZ)){}
		}
		//STANDING BACK	
		else if(lola_state == Lola_state::standingB){
			cout << "Standing Front" << endl;
			//standingBack()
			if(filterZ == -10 && seguro(fsrR, fsrL)){
				lola_last_state = lola_state;
				lola_state = Lola_state::stand;
			}
			else if(isFalling(imu, fsrR, fsrL, filterX, last_filterX, filterY, last_filterY, filterZ, last_filterZ)){}
		}
		
		else{
			if (walking_engine.isStanding()) {
				joints.legs = sit_motion.sitDown(sensor_frame.joints.legs, ankle_balancer, &arm_controller);
			}
			else{
				walking_engine.setRequest(0, 0, 0, 0, Shoot::NONE);
				walking_engine.setRequest(0, 0, 0, 0);
				joints.legs = walking_engine.proceed(sensor_frame.fsr, sensor_frame.imu.gyr.pitch, sensor_frame.imu.gyr.roll, ankle_balancer, sensor_frame.imu.gyr.yaw, &odo, &arm_controller);
			}
			//walking_engine.reset();
			joints.arms = arm_controller.proceed();
		}

		// Set the head pitch to something.
		char* buffer;
		size_t size;
		tie(buffer, size) = frame_handler.pack();
		socket.send(boost::asio::buffer(buffer, size));
		if (lola_shutdown){
			// When finishing, set all stiffnesses to -1. Also it's necessary to get another packet otherwise we can't send.
			socket.receive(boost::asio::buffer(data, max_len));
			set_stiffness(-1.f, &frame_handler.actuator_frame.joints.legs);
			set_stiffness(-1.f, &frame_handler.actuator_frame.joints.arms);
			set_stiffness(-1.f, &frame_handler.actuator_frame.joints.head);
			tie(buffer, size) = frame_handler.pack();
			socket.send(boost::asio::buffer(buffer, size));
			break;
		}
		
		Behavior estadoDeJogo;
		frame_handler.actuator_frame.leds.eyes.left.fill(estadoDeJogo.geteyecolor());

	}
	return 0;
}