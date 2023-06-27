#include "LolaConnector/lola_connector.h"
#include "HTWKMotion/ankle_balancer.h"
#include "HTWKMotion/arm_controller.h"
#include "HTWKMotion/sit_motion.h"
#include "HTWKMotion/walking_engine.h"
#include "utils/imu.h"
#include "LolaConnector/fila.h"
#include "state.h"

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

NAO::NAO(){
    setState(Lola_state::begin);
	
}

void NAO::setState(Lola_state newEstado){
    state = newEstado;
}

void NAO::setLastState(){
    last_state = state;
}

Lola_state NAO::getState(){
    return state;
}

Lola_state NAO::getLastState(){
    return last_state;
}

bool NAO::seguro(){
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

void NAO::freshFilter(){
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

	//Filtro Gyr
	last_filterPitch = filterP;
	last_filterRoll = filterR;
	dadoP.freshFila(imu.gyr.pitch);
	dadoR.freshFila(imu.gyr.roll);
	filterP = dadoP.filterRP();
	filterR = dadoR.filterRP();
}

void NAO::declara(){
	const LolaSensorFrame& sensor_frame = frame_handler.unpack(data, socket.receive(boost::asio::buffer(data, max_len)));
	auto& joints = frame_handler.actuator_frame.joints; //Juntas
	auto& leds = frame_handler.actuator_frame.leds; // Leds
	auto& battery = sensor_frame.battery; // Bateria
	auto& fsr = sensor_frame.fsr; // FSR: sensores dos pés
	auto& imu = sensor_frame.imu; // IMU: Accel e GYR

	// Vetor dos FSR
	float fsrR[4] = {fsr.right.fl, fsr.right.fr, fsr.right.rl, fsr.right.rr};
	float fsrL[4] = {fsr.left.fl, fsr.left.fr, fsr.left.rl, fsr.left.rr};
}

bool NAO::fallen(){
    //Contato pés
	float foot = 0;
	for(int c = 0; c < 4; c++){
		foot += (right[c] + left[c]);
	}

	//Condição
	if(filterZ > -1 && foot < 1 ){
        setLastState();
		setState(Lola_state::fallen);
		return true;
	}
	else{
		return false;
	}
}

bool NAO::isStanding(){
    if(getLastState() == Lola_state::fallingF || (getLastState() == Lola_state::fallingR || getLastState() == Lola_state::fallingL)){
		if((filterZ < -1 && last_filterZ > filterZ) && (filterX < 8 && filterX > 1) ){
			setLastState();
			setState(Lola_state::standingF);
			return true;
		}
		else{
			return false;
		}
	}
	else if(getLastState() == Lola_state::fallingB || (getLastState() == Lola_state::fallingR || getLastState() == Lola_state::fallingL)){
		if((filterZ < -1 && last_filterZ > filterZ) && (filterX > -8.5 && filterX < -1.5)){
			setLastState();
			setState(Lola_state::standingB);
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

bool NAO::isFalling(){
    //Last state
	if(getLastState() == Lola_state::stand || getLastState() == Lola_state::fallingF || getLastState() == Lola_state::fallingB || getLastState() == Lola_state::fallingL || 
	getLastState() == Lola_state::fallingR || getLastState() == Lola_state::standingF || getLastState() == Lola_state::standingB || getLastState() == Lola_state::fallen){

		//Pés no chão
		if(!seguro(fsrR, fsrL)){
			//Angulação Az
			if((filterZ > -9 && filterZ < -2) && last_filterZ < filterZ){
				//Front
				if(filterX > 4 && (filterY > -3 && filterY < 3)){
					cout << "F" << endl;
					setLastState();
					setState(Lola_state::fallingF);
					return true;
				}
				//Back
				else if(filterX < -4 && (filterY > -3 && filterY < 3)){
					cout << "B" << endl;
					setLastState();
					setState(Lola_state::fallingB);
					return true;
				}
				//Right
				else if(filterY > 5 && (filterX > -3 && filterX < 3)){
					cout << "R" << endl;
					setLastState();
					setState(Lola_state::fallingR);
					return true;
				}
				//Left
				else if(filterY < -5 && (filterX > -3 && filterX < 3)){
					cout << "L" << endl;
					setLastState();
					setState(Lola_state::fallingL);
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

void NAO::standingBegin(){
	setState(Lola_state::standingBegin);
	joints.head[HeadPitch] = {.angle = 0.3f, .stiffness = 1.f};
	joints.legs = sit_motion.getUp(sensor_frame.joints.legs, ankle_balancer, &arm_controller);
	joints.arms = arm_controller.proceed();
	walking_engine.reset();
	// TODO: You probably want to have a few else if statements here for things that should override the walking, e.g. getting up.
}

void NAO::stand(){
	
}
