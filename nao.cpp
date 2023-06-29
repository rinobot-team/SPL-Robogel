#include "HTWKMotion/arm_controller.h"
#include "HTWKMotion/ankle_balancer.h"
#include "HTWKMotion/sit_motion.h"
#include "HTWKMotion/walking_engine.h"
#include "LolaConnector/lola_connector.h"
#include "utils/imu.h"
#include "nao.h"

#include <vector>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <map>

#include <boost/asio.hpp>
#include <msgpack.hpp>

#include <lola_frame.h>

NAO::NAO(){
    setState(Lola_state::begin);
    setLastState();
}

Lola_state NAO::getState(){
    return state;
}

Lola_state NAO::getLastState(){
    return last_state;
}

void NAO::setState(Lola_state newEstado){
    state = newEstado;
}

void NAO::setLastState(){
    last_state = state;
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

void NAO::fresh(){
    // Sensor
    LolaSensorFrame& sensor_frameAUX = frame_handler.unpack(data, socket.receive(boost::asio::buffer(data, max_len)));
    sensor_frame = sensor_frameAUX;
    // Fila
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

bool isFalling(){
	//Last state (teste)
	//if(lola_last_state == Lola_state::stand || lola_last_state == Lola_state::fallingF || lola_last_state == Lola_state::fallingB || lola_last_state == Lola_state::fallingL || 
	//lola_last_state == Lola_state::fallingR || lola_last_state == Lola_state::standingF || lola_last_state == Lola_state::standingB || lola_last_state == Lola_state::fallen){

		//Pés no chão
		if(!seguro()){
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
//}

bool isFallen(){
    //Accel
	//float Az = imu.accel.z;

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
	//float Ax = imu.accel.x;
	//float Ay = imu.accel.y;
	//float Az = imu.accel.z;

	//Gyr
	//float pitch = imu.gyr.pitch; // Front / Back
	//float roll = imu.gyr.roll; // Right / Left

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

bool NAO::isStand(){
	if(filterZ == -10 && seguro(fsrR, fsrL)){
		setLastState();
		setState(Lola_state::stand);
	}
}


bool NAO::standingBegin(){
    if (!sit_motion.isStanding()) {
		lola_state = Lola_state::standingBegin;
		joints.head[HeadPitch] = {.angle = 0.3f, .stiffness = 1.f};
		joints.legs = sit_motion.getUp(sensor_frame.joints.legs, ankle_balancer, &arm_controller);
		joints.arms = arm_controller.proceed();
		walking_engine.reset();
		// TODO: You probably want to have a few else if statements here for things that should override the walking, e.g. getting up.
        setState(Lola_state::standingBegin);
        return true;
	}
    else{
        setState(Lola_state::stand);
        return false;
    } 
}
void NAO::stand(){
    setLastState();
    setState(Lola_state::stand);
	cout << "Stand!" << endl;

	//joints.legs = walking_engine.proceed(sensor_frame.fsr, imu_filter.angles.pitch (0.1), imu_filter.angles.roll (0), ankle_balancer,

	joints.arms = arm_controller.proceed();
	joints.legs = walking_engine.proceed(sensor_frame.fsr, 0.1, 0, ankle_balancer,sensor_frame.imu.gyr.yaw, &odo, &arm_controller);

	isFalling(imu, fsrR, fsrL);
				
	if(seguro(fsrR, fsrL)){
		cout << "No chao!" << endl;
        // ESTRATÉGIA
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

void NAO::falling(){
    switch(getState()){
        case Lola_state::fallingF:
            cout << "Falling Front" << endl;
		    //isfalling()
		    fallen(imu, fsrR, fsrL);
            break;
        case Lola_state::fallingB:
            cout << "Falling Back" << endl;
			//isfalling()
			fallen(imu, fsrR, fsrL);
            break;
        case Lola_state::fallingR:
            cout << "Falling Right" << endl;
			//isfalling()
			fallen(imu, fsrR, fsrL);
            break;
        case Lola_state::fallingL:
            cout << "Falling Left" << endl;
			//isfalling()
			fallen(imu, fsrR, fsrL);
            break;
    }
}

void NAO::fallen(){
    cout << "FALLEN!!!" << endl;
	//fallen();
	isStanding();
}

void NAO::standing(){
    switch(getState()){
        case Lola_state::standingF:
            cout << "Standing Front" << endl;
			//standingFront()
			isStanding();
			isFalling();
            break;
        case Lola_state::standingB:
            cout << "Standing Back" << endl;
			//standingBack()
			isStanding();
			isFalling();
            break;
    }
}

void NAO::finish(){
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

void NAO::shutdown(){
    // When finishing, set all stiffnesses to -1. Also it's necessary to get another packet otherwise we can't send.
	socket.receive(boost::asio::buffer(data, max_len));
	set_stiffness(-1.f, &frame_handler.actuator_frame.joints.legs);
	set_stiffness(-1.f, &frame_handler.actuator_frame.joints.arms);
	set_stiffness(-1.f, &frame_handler.actuator_frame.joints.head);
	tie(buffer, size) = frame_handler.pack();
	socket.send(boost::asio::buffer(buffer, size));
	break;
}

 