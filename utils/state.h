#ifndef STATE_H
#define STATE_H

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

class NAO {
private:

Lola_state state;
Lola_state last_state;

public:
	NAO();
	Lola_state getLastState();
    Lola_state getState();
	void setLastState();
	bool seguro(float fsrR[], float fsrL[]);
    void setState(Lola_state newEstado);
	bool isFalling(IMU imu, float fsrR[], float fsrL[], float filterX, float last_filterX, float filterY, float last_filterY, float filterZ, float last_filterZ);
	bool isStanding(IMU imu, float filterX, float last_filterX, float filterY, float last_filterY, float filterZ, float last_filterZ);
	bool fallen(IMU imu, float right[], float left[], float filterZ);   
};

#endif STATE_H
