#include "LolaConnector/lola_connector.h"
#include "HTWKMotion/ankle_balancer.h"
#include "HTWKMotion/arm_controller.h"
#include "HTWKMotion/sit_motion.h"
#include "HTWKMotion/walking_engine.h"
#include "utils/imu.h"
#include "LolaConnector/fila.h"
#include "HTWKMotion/state.h"
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

bool NAO::seguro(float fsrR[], float fsrL[]){
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

bool NAO::fallen(IMU imu, float right[], float left[], float filterZ){
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

bool NAO::isStanding(IMU imu, float filterX, float last_filterX, float filterY, float last_filterY, float filterZ, float last_filterZ){
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

bool NAO::isFalling(IMU imu, float fsrR[], float fsrL[], float filterX, float last_filterX, float filterY, float last_filterY, float filterZ, float last_filterZ){
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
