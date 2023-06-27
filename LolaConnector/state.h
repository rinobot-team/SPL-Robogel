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
// Estáticos
Lola_state state;
Lola_state last_state;
Fila dadoX, dadoY, dadoZ, dadoP, dadoR;
float filterX = 0, filterY = 0, filterZ = 0, filterP = 0, filterR = 0;
float last_filterX = 0, last_filterY = 0, last_filterZ = 0, last_filterPitch = 0, last_filterRoll = 0;
LolaFrameHandler frame_handler;

// Mutáveis
const LolaSensorFrame& sensor_frame = frame_handler.unpack(data, socket.receive(boost::asio::buffer(data, max_len)));
auto& joints = frame_handler.actuator_frame.joints; //Juntas
auto& leds = frame_handler.actuator_frame.leds; // Leds
auto& battery = sensor_frame.battery; // Bateria
auto& fsr = sensor_frame.fsr; // FSR: sensores dos pés
auto& imu = sensor_frame.imu; // IMU: Accel e GYR

// Vetor dos FSR
float fsrR[4] = {fsr.right.fl, fsr.right.fr, fsr.right.rl, fsr.right.rr};
float fsrL[4] = {fsr.left.fl, fsr.left.fr, fsr.left.rl, fsr.left.rr};

public:
	NAO();	// Construtor

	auto sit_motion = SitM otion();
	auto ankle_balancer = AnkleBalancer();
	auto arm_controller = ArmController();
	auto walking_engine = WalkingEngine();
	auto odo = Odometry();

	Lola_state getLastState();		// Retorna o último estado
    Lola_state getState();		// Retorna o estado atual
	void setLastState();		// Seta o estado atual como último estado
    void setState(Lola_state newEstado);		// Seta "newEstado" como estado atual
	void declara();		//Declara / Atribui os sensores
	void freshFilter();		//Atualiza filtros	
	void standingBegin();		// Levanta (Inicial)
	void stand();		// NAO em pé
	void falling();		// NAO caindo
	void fallen();		// NAO caído
	void standing();		//NAO levantando
	bool seguro();		// Verifica se o robô está de pé
	bool isFalling();		// Verifica se o robô está caíndo
	bool isStanding();		// Verifica se o robô está levantando
	bool fallen();		// Verifica se o robô está caído
};

#endif STATE_H
