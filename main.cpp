#include "LolaConnector/lola_connector.h"
#include "HTWKMotion/ankle_balancer.h"
#include "HTWKMotion/arm_controller.h"
#include "HTWKMotion/sit_motion.h"
#include "HTWKMotion/walking_engine.h"
#include "utils/imu.h"
#include "LolaConnector/fila.h"

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
	enum class Lola_state{
		//Estado incial
		begin,

		//Levantado
		stand,

		//Levantado inicial / Front / Back
		standingBegin,
		standingF,
		standingB,

		//Caindo Front / Back / Right / Left
		fallingF,
		fallingB,
		fallingR,
		fallingL,

		//Caido
		fallen,

		//Sentado
		sit
	};
	Lola_state lola_state = Lola_state::begin;
	Lola_state lola_last_state;
	float filterX = 0, filterY = 0, filterZ = 0;
	float last_filterX = 0, last_filterY = 0, last_filterZ = 0; 

bool seguro(float fsrR[], float fsrL[]){
	float footR = 0;
	float footL = 0;
	for(int c = 0; c < 4; c++){
		footR += fsrR[c];
		footL += fsrL[c];
	}

	if(footR < 1 && footL < 1){
		return false;
	}
	else{
		return true;
	}
}

bool fallen(IMU imu, float right[], float left[]){
	//Accel
	//float Az = imu.accel.z;

	//Contato pés
	float foot = 0;
	for(int c = 0; c < 4; c++){
		foot += (right[c] + left[c]);
	}

	//Condição
	if(filterZ > -1 && foot < 1 ){
		lola_last_state = lola_state;
		lola_state = Lola_state::fallen;
		return true;
	}
	else{
		return false;
	}
}

bool isStanding(IMU imu){
	//float Ax = imu.accel.x;
	//float Ay = imu.accel.y;
	//float Az = imu.accel.z;

	//Gyr
	//float pitch = imu.gyr.pitch; // Front / Back
	//float roll = imu.gyr.roll; // Right / Left

	if(lola_last_state == Lola_state::fallingF || (lola_last_state == Lola_state::fallingR || lola_last_state == Lola_state::fallingL)){
		if((filterZ < -1 && last_filterZ > filterZ) && (filterX < 8 && filterX > 1) ){
			lola_last_state = lola_state;
			lola_state = Lola_state::standingF;
			return true;
		}
		else{
			return false;
		}
	}
	else if(lola_last_state == Lola_state::fallingB || (lola_last_state == Lola_state::fallingR || lola_last_state == Lola_state::fallingL)){
		if((filterZ < -1 && last_filterZ > filterZ) && (filterX > -8.5 && filterX < -1.5)){
			lola_last_state = lola_state;
			lola_state = Lola_state::standingB;
			return true;
		}
		else{
			return false;
		}
	}
}

bool isFalling(IMU imu, float fsrR[], float fsrL[]){
	//Acell
	//float Ax = imu.accel.x;
	//float Ay = imu.accel.y;
	//float Az = imu.accel.z;

	//Last state
	if(lola_last_state == Lola_state::stand || lola_last_state == Lola_state::fallingF || lola_last_state == Lola_state::fallingB || lola_last_state == Lola_state::fallingL || 
	lola_last_state == Lola_state::fallingR || lola_last_state == Lola_state::standingF || lola_last_state == Lola_state::standingB || lola_last_state == Lola_state::fallen){

		//Pés no chão
		if(!seguro(fsrR, fsrL)){
			//Angulação Az
			if((filterZ > -9 && filterZ < -2) && last_filterZ < filterZ){
				//Front
				if(filterX > 4 && (filterY > -3 && filterY < 3)){
					cout << "F" << endl;
					lola_last_state = lola_state;
					lola_state = Lola_state::fallingF;
					return true;
				}
				//Back
				else if(filterX < -4 && (filterY > -3 && filterY < 3)){
					cout << "B" << endl;
					lola_last_state = lola_state;
					lola_state = Lola_state::fallingB;
					return true;
				}
				//Right
				else if(filterY > 5 && (filterX > -3 && filterX < 3)){
					cout << "R" << endl;
					lola_last_state = lola_state;
					lola_state = Lola_state::fallingR;
					return true;
				}
				//Left
				else if(filterY < -5 && (filterX > -3 && filterX < 3)){
					cout << "L" << endl;
					lola_last_state = lola_state;
					lola_state = Lola_state::fallingL;
					return true;
				}
				else{
					return false;
				}
			}
			return false;
		}
		else{
			return false;
		}
	}
	else{
		return false;
	}
}

void ctrlc_handler(int) {
	lola_shutdown = true;
}

int main(int, char*[]) {
	auto sit_motion = SitMotion();
	auto ankle_balancer = AnkleBalancer();
	auto arm_controller = ArmController();
	auto walking_engine = WalkingEngine();
	auto odo = Odometry();

	// Register some handlers so we can clean-up when we're killed.
	signal(SIGINT, ctrlc_handler);
	signal(SIGTERM, ctrlc_handler);

	io_service io_service;
	local::stream_protocol::socket socket(io_service);

	socket.connect("/tmp/robocup");

	Fila dadoX, dadoY, dadoZ;
	constexpr int max_len = 100000;
	char data[max_len] = {'\0'};
	boost::system::error_code ec;
	LolaFrameHandler frame_handler;
	int pra_frente = 0;
	int pro_lado = 0;
	while (true) {
		//Declarações
		const LolaSensorFrame& sensor_frame = frame_handler.unpack(data, socket.receive(boost::asio::buffer(data, max_len)));
		auto& joints = frame_handler.actuator_frame.joints; //Juntas
		auto& leds = frame_handler.actuator_frame.leds; // Leds
		auto& battery = sensor_frame.battery; // Bateria
		auto& fsr = sensor_frame.fsr; // FSR: sensores dos pés
		auto& imu = sensor_frame.imu; // IMU: Accel e GYR
		auto& gyr = imu.gyr;// GYR
		//Vetor dos FSR
		float fsrR[4] = {fsr.right.fl, fsr.right.fr, fsr.right.rl, fsr.right.rr};
		float fsrL[4] = {fsr.left.fl, fsr.left.fr, fsr.left.rl, fsr.left.rr};
		//Atualizações do filtro
		last_filterX = filterX;
		last_filterY = filterY;
		last_filterZ = filterZ;
		dadoX.freshFila(imu.accel.x);
		dadoY.freshFila(imu.accel.y);
		dadoZ.freshFila(imu.accel.z);
		filterX = dadoX.filter();
		filterY = dadoY.filter();
		filterZ = dadoZ.filter();

		/*cout << "Filtro X: " << filterX << endl;
		cout << "X" << imu.accel.x << endl;
		cout << "Filtro Y: " << filterY << endl;
		cout << "Y" << imu.accel.y << endl;
		cout << "Filtro Z: " << filterZ << endl;
		cout << "Z" << imu.accel.z << endl;*/

		//Status battery
		//cout << "Bateria: " << battery.charge << '\n';
        if(battery.charge >= 0.75){            
            leds.eyes.right.fill(RGB::GREEN);
        }
        else if(battery.charge >= 0.5 && battery.charge < 0.75){            
            leds.eyes.right.fill(RGB::YELLOW);
        }
        else if(battery.charge >= 0.25 && battery.charge < 0.5){
            leds.eyes.right.fill(RGB::ORANGE);
        }
        else{
            leds.eyes.right.fill(RGB::RED);
        }
		
		//TODO: Insert walking engine and stuff here. :)
		if (/*client_connected &&*/ !lola_shutdown && !lola_sit_forever && (lola_state == Lola_state::begin || lola_state == Lola_state::standingBegin || lola_state == Lola_state::stand)){
			if (!sit_motion.isStanding()) {
				lola_state = Lola_state::standingBegin;
				joints.head[HeadPitch] = {.angle = 0.3f, .stiffness = 1.f};
				joints.legs = sit_motion.getUp(sensor_frame.joints.legs, ankle_balancer, &arm_controller);
				joints.arms = arm_controller.proceed();
				walking_engine.reset();
				// TODO: You probably want to have a few else if statements here for things that should override the walking, e.g. getting up.
			} 
			else{
				lola_last_state = lola_state;
				lola_state = Lola_state::stand;
				cout << "Stand!" << endl;

				//joints.legs = walking_engine.proceed(sensor_frame.fsr, imu_filter.angles.pitch, imu_filter.angles.roll, ankle_balancer,

				joints.arms = arm_controller.proceed();
				joints.legs = walking_engine.proceed(sensor_frame.fsr, 0.1, 0, ankle_balancer,sensor_frame.imu.gyr.yaw, &odo, &arm_controller);

				isFalling(imu, fsrR, fsrL);
				
				if(seguro(fsrR, fsrL)){
					cout << "No chao!" << endl;
					if(pra_frente < 1000 && pro_lado == 0){
						//cout << "Pra frente " << pra_frente << endl;
						walking_engine.setRequest(0.07, 0, 0, 0.1);
						pra_frente++;
					}
					/*else if(pro_lado < 400 && pra_frente == 400){
						//cout << "Pro lado " << pro_lado << endl;
						walking_engine.setRequest(0.07, 0, 1, 0.1);
						pro_lado++;
					}*/
					else{
						joints.head[HeadPitch] = {.angle = 0.1f, .stiffness = 1.f};
						lola_sit_forever = true;
					}
				}
				else{
					//cout << "Fora do chao!" << endl;
					//walking_engine.reset();
					walking_engine.setRequest(0, 0, 0, 0.1);
					
				}
			}
		}
		//FALLING
		else if(lola_state == Lola_state::fallingF || lola_state == Lola_state::fallingB || lola_state == Lola_state::fallingR || lola_state == Lola_state::fallingL){
			if(lola_state == Lola_state::fallingF){
				cout << "Falling Front" << endl;
				fallen(imu, fsrR, fsrL);
			}
			else if(lola_state == Lola_state::fallingB){
				cout << "Falling Back" << endl;
				fallen(imu, fsrR, fsrL);
			}
			else if(lola_state == Lola_state::fallingR){
				cout << "Falling Right" << endl;
				fallen(imu, fsrR, fsrL);
			}
			else{
				cout << "Falling Left" << endl;
				fallen(imu, fsrR, fsrL);
			}
		}
		//FALLEN
		else if(lola_state == Lola_state::fallen && (lola_last_state == Lola_state::fallingB || lola_last_state == Lola_state::fallingF || lola_last_state == Lola_state::fallingR || lola_last_state == Lola_state::fallingL)){
			cout << "FALLEN!!!" << endl;
			isStanding(imu);
		}
		//STANDING FRONT
		else if(lola_state == Lola_state::standingF){
			cout << "Standing Front" << endl;
			//standingFront()
			if(filterZ == -10 && seguro(fsrR, fsrL)){
				lola_last_state = lola_state;
				lola_state = Lola_state::stand;
			}
			else if(isFalling(imu, fsrR, fsrL)){}
		}
		//STANDING BACK	
		else if(lola_state == Lola_state::standingB){
			cout << "Standing Front" << endl;
			//standingBack()
			if(filterZ == -10 && seguro(fsrR, fsrL)){
				lola_last_state = lola_state;
				lola_state = Lola_state::stand;
			}
			else if(isFalling(imu, fsrR, fsrL)){}
		}
		
		else{
			if (walking_engine.isStanding()) {
				joints.legs = sit_motion.sitDown(sensor_frame.joints.legs, ankle_balancer, &arm_controller);
			}
			else{
				// walking_engine.setRequest(0, 0, 0, 0, Shoot::NONE);
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
	}
	return 0;
}