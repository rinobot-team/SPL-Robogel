#ifndef NAO_H
#define NAO_H

#include "HTWKMotion/ankle_balancer.h"
#include "HTWKMotion/arm_controller.h"
#include "HTWKMotion/sit_motion.h"
#include "HTWKMotion/walking_engine.h"
#include "utils/imu.h"
#include "LolaConnector/lola_connector.h"
#include "fila.h"

#include <cmath>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <map>

#include <boost/asio.hpp>
#include <msgpack.hpp>

#include <lola_frame.h>

enum Lola_state{
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
	sit,
    sit_forever,

    shutdown,
	finish
};

class NAO{
private:
    Lola_state state;
    Lola_state last_state;
    void setState(Lola_state newEstado);
    void setLastState();

    // Declarações / Atribuições
	SitMotion sit_motion = SitMotion();
	AnkleBalancer ankle_balancer = AnkleBalancer();
	ArmController arm_controller = ArmController();
	WalkingEngine walking_engine = WalkingEngine();
	Odometry odo = Odometry();
	Fila dadoX, dadoY, dadoZ, dadoP, dadoR;
    LolaFrameHandler& frame_handler;
    Joints& joints = frame_handler.actuator_frame.joints; //Juntas
	int pra_frente = 0;
	int pro_lado = 0;

    // Sensores -- fresh
	float filterX = 0, filterY = 0, filterZ = 0, filterP = 0, filterR = 0;
	float last_filterX = 0, last_filterY = 0, last_filterZ = 0, last_filterPitch = 0, last_filterRoll = 0;
public:
    NAO();

    Lola_state getState();	// Retorna o estado atual
    Lola_state getLastState();	// Retorna o ultimo estado
    bool standingBegin(LolaSensorFrame sensor_frame);	// Levantar(inicial)
	bool isStand(float fsrR[], float fsrL[]);	// Verifica se o NAO está de pé
    void stand(LolaSensorFrame sensor_frame, float fsrR[], float fsrL[]);	// Ações
	bool isFalling(float fsrR[], float fsrL[]);	// Verifica se o NAO está caíndo
	void falling(float fsrR[], float fsrL[]);	// Ações
	bool isFallen(float fsrR[], float fsrL[]);	// Verifica se o NAO está caído
	void fallen();	// Ações
	bool isStanding();	// Verifica se o NAO está levantando
	void standing(float fsrR[], float fsrL[]);	// Ações
	void sit(LolaSensorFrame sensor_frame);	//Ação de finalização	

	bool seguro(float fsrR[], float fsrL[]);
	void fresh(IMU imu);
};

NAO::NAO(){
    setState(Lola_state::begin);
    setLastState();
}

#endif