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

	float last_pos;
	
	//Lola's State
	static bool lola_shutdown = false;
	bool lola_sit_forever = false;
	enum class Lola_state{
		begin,
		stand,
		standing1,
		standing2,
		//standingF,
		//standingB,
		fallingF,
		fallingB,
		fallen,
		sit
	};
	Lola_state lola_state = Lola_state::begin;
	Lola_state lola_last_state;

/*bool contato(float front, float back){
	cout << "Front: " << front << endl;
	cout << "Back: " << back << endl;
	if((front < 1 && back > 1.6) || (front > 1 && back < 1.2)){
		return true;
	}
	else{
		return false;
	}
}

bool caindo(float fsrR[], float fsrL[]){
	float right_footF = fsrR[0] + fsrR[1];
	float right_footR = fsrR[2] + fsrR[3];
	float left_footF = fsrL[0] + fsrL[1];
	float left_footR = fsrL[2] + fsrL[3];

	cout << "right_footF: " << right_footF << endl;
	cout << "right_footR: " << right_footR << endl;
	cout << "left_footF: " << left_footF << endl;
	cout << "left_footR: " << left_footR << endl;

	if(contato(right_footF, right_footR) && contato(left_footF, left_footR)){
		return true;
	}
	else{
		return false;
	}
}*/

bool fallen(IMU imu, float right[], float left[]){
	float Az = imu.accel.z;

	//Gyr
	float pitch = imu.gyr.pitch; // Front / Back
	float roll = imu.gyr.roll; // Right / Left

	float foot = 0;
	for(int c = 0; c < 4; c++){
		foot += right[c] + left[c];
	}

	if(Az > -1 && foot < 1 && (pitch > -0.6 || pitch < 0.6)){
		return true;
	}
	else
		return false;
}

bool isStanding(IMU imu){
	float Ax = imu.accel.x;
	float Ay = imu.accel.y;
	float Az = imu.accel.z;

	//Gyr
	float pitch = imu.gyr.pitch; // Front / Back
	float roll = imu.gyr.roll; // Right / Left

	if((Ax > -4.5 || Ax < 4.5) || (Ay > -4.5 || Ay < 4.5) && (Az > -1.5 && Az < 1.5) && (last_pos > Az)){
		return true;
	}
	else{
		return false;
	}
}

bool isFallingF(IMU imu){
	//Acell
	float Ax = imu.accel.x;
	float Ay = imu.accel.y;
	float Az = imu.accel.z;

	//Gyr
	float pitch = imu.gyr.pitch; // Front / Back
	float roll = imu.gyr.roll; // Right / Left

	if((Ax < -4.5 || Ax > 4.5) || (Ay < -4.5 || Ay > 4.5) && (Az > -7 && Az < -1.5) && (last_pos < Az)){
		if(pitch > 1.1){
			lola_last_state = lola_state;
			lola_state = Lola_state::fallingF;
			return true;
		}
		else{
			return false;
		}
	}
	else{
		return false;
	}
}

bool isFallingB(IMU imu){
	//Acell
	float Ax = imu.accel.x;
	float Ay = imu.accel.y;
	float Az = imu.accel.z;

	//Gyr
	float pitch = imu.gyr.pitch; // Front / Back
	float roll = imu.gyr.roll; // Right / Left

	if((Ax < -4.5 || Ax > 4.5) || (Ay < -4.5 || Ay > 4.5) && (Az > -7 && Az < -1.5) && (last_pos < Az)){
		if(pitch < -1.1){
			lola_last_state = lola_state;
			lola_state = Lola_state::fallingB;
			return true;
		}
		else{
			return false;
		}
	}
	else{
		return false;
	}
}

bool seguro(float fsrR[], float fsrL[]){
	float footR = 0;
	float footL = 0;
	for(int c = 0; c < 4; c++){
		footR += fsrR[c];
		footL += fsrL[c];
	}

	if(footR < 2 && footL < 2){
		return false;
	}
	else{
		return true;
	}
}

void ctrlc_handler(int) {
	lola_shutdown = true;
}

int main(int, char*[]) {
	cout << "oi!!" << endl;

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
	cout << "oi1.5!!" << endl;

	socket.connect("/tmp/robocup");
	cout << "oi2!!" << endl;

	constexpr int max_len = 100000;
	char data[max_len] = {'\0'};
	boost::system::error_code ec;
	LolaFrameHandler frame_handler;
	int pra_frente = 0;
	int pro_lado = 0;
	while (true) {
		cout << "oi3!" << endl;
		//Declarações
		const LolaSensorFrame& sensor_frame = frame_handler.unpack(data, socket.receive(boost::asio::buffer(data, max_len)));
		auto& joints = frame_handler.actuator_frame.joints;
		auto& leds = frame_handler.actuator_frame.leds;
		auto& battery = sensor_frame.battery;
		auto& fsr = sensor_frame.fsr;
		auto& imu = sensor_frame.imu;
		auto& gyr = sensor_frame.imu.gyr;
		float fsrR[4] = {fsr.right.fl, fsr.right.fr, fsr.right.rl, fsr.right.rr};
		float fsrL[4] = {fsr.left.fl, fsr.left.fr, fsr.left.rl, fsr.left.rr};

		//GYR
		cout << "Pitch: " << gyr.pitch << endl;
		cout << "Roll: " << gyr.roll << endl;
		cout << "Yaw: " << gyr.yaw << endl;

		//status battery
		cout << "Bateria: " << battery.charge << '\n';
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
		if (/*client_connected &&*/ !lola_shutdown && !lola_sit_forever && (lola_state == Lola_state::begin || lola_state == Lola_state::standing1 || lola_state == Lola_state::stand)){
			if (!sit_motion.isStanding()) {
				lola_state = Lola_state::standing1;
				joints.head[HeadPitch] = {.angle = 0.3f, .stiffness = 1.f};
				joints.legs = sit_motion.getUp(sensor_frame.joints.legs, ankle_balancer, &arm_controller);
				joints.arms = arm_controller.proceed();
				walking_engine.reset();
				// TODO: You probably want to have a few else if statements here for things that should override the walking, e.g. getting up.
			} 
			else{
				lola_state = Lola_state::stand;
				cout << "Stand!" << endl;
				//joints.legs = walking_engine.proceed(sensor_frame.fsr, imu_filter.angles.pitch, imu_filter.angles.roll, ankle_balancer,
				joints.arms = arm_controller.proceed();
				joints.legs = walking_engine.proceed(sensor_frame.fsr, 0.1, 0, ankle_balancer,sensor_frame.imu.gyr.yaw, &odo, &arm_controller);

				if(isFallingB(sensor_frame.imu)){
					lola_last_state = lola_state;
					lola_state = Lola_state::fallingB;
				}
				else if(isFallingF(sensor_frame.imu)){
					lola_last_state = lola_state;
					lola_state = Lola_state::fallingF;
				}
				
				if(seguro(fsrR, fsrL) && lola_state == Lola_state::stand){
					last_pos = imu.accel.z;
					//cout << "No chao!" << endl;
					/*if(pra_frente < 1000 && pro_lado == 0){
						//cout << "Pra frente " << pra_frente << endl;
						walking_engine.setRequest(0.07, 0, 0, 0.1);
						pra_frente++;
					}
					/*else if(pro_lado < 400 && pra_frente == 400){
						//cout << "Pro lado " << pro_lado << endl;
						walking_engine.setRequest(0.07, 0, 1, 0.1);
						pro_lado++;
					}
					else{
						joints.head[HeadPitch] = {.angle = 0.1f, .stiffness = 1.f};
						lola_sit_forever = true;
					}*/
				}
				else{
					//cout << "Fora do chao!" << endl;
					//walking_engine.reset();
					walking_engine.setRequest(0, 0, 0, 0.1);
					
				}
			}
		}
		else if((lola_state == Lola_state::fallingB || lola_state == Lola_state::fallingF) && lola_last_state != Lola_state::fallen){
			cout << "Falling!" << endl;
			if(fallen(sensor_frame.imu, fsrR, fsrL)){
				lola_last_state = lola_state;
				lola_state = Lola_state::fallen;
			}
			else{
				//set_stiffness(0.f, &frame_handler.actuator_frame.joints.legs);
				//set_stiffness(0.f, &frame_handler.actuator_frame.joints.arms);
				//set_stiffness(0.f, &frame_handler.actuator_frame.joints.head);
			}
		}
		else if(lola_state == Lola_state::fallen){
			cout << "FALLEN!!!" << endl;
			if(isStanding(sensor_frame.imu)){
				lola_last_state = lola_state;
				lola_state = Lola_state::standing2;
			}
		}
		else if(lola_state == Lola_state::standing2 && lola_last_state == Lola_state::fallen){
			cout << "Standing..." << endl;
			if((!isFallingB(sensor_frame.imu) || !isFallingF(sensor_frame.imu)) && !fallen(sensor_frame.imu, fsrR, fsrL) && seguro(fsrR, fsrL)){
				lola_last_state = lola_state;
				lola_state = Lola_state::stand;
			}
			else if(fallen(sensor_frame.imu, fsrR, fsrL)){
				lola_state = Lola_state::fallen;
			}
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
		joints.head[HeadPitch] = {.angle = 0.3f, .stiffness = 1.f};
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
