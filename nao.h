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

    shutdown
};

class NAO{
private:
    Lola_state state;
    Lola_state last_state;
    void setState(Lola_state newEstado);
    void setLastState();

    // Declarações / Atribuições
	auto sit_motion = SitMotion();
	auto ankle_balancer = AnkleBalancer();
	auto arm_controller = ArmController();
	auto walking_engine = WalkingEngine();
	auto odo = Odometry();
	signal(SIGINT, ctrlc_handler);
	signal(SIGTERM, ctrlc_handler);
	io_service io_service;
	local::stream_protocol::socket socket(io_service);
	socket.connect("/tmp/robocup");
	constexpr int max_len = 100000;
	char data[max_len] = {'\0'};
	boost::system::error_code ec;
    Fila dadoX, dadoY, dadoZ, dadoP, dadoR;
    LolaFrameHandler frame_handler;
    // Sensores -- fresh
    const LolaSensorFrame& sensor_frame = frame_handler.unpack(data, socket.receive(boost::asio::buffer(data, max_len)));
	auto& joints = frame_handler.actuator_frame.joints; //Juntas
	auto& leds = frame_handler.actuator_frame.leds; // Leds
	auto& battery = sensor_frame.battery; // Bateria
	auto& fsr = sensor_frame.fsr; // FSR: sensores dos pés
	auto& imu = sensor_frame.imu; // IMU: Accel e GYR
    float fsrR[4] = {fsr.right.fl, fsr.right.fr, fsr.right.rl, fsr.right.rr};
	float fsrL[4] = {fsr.left.fl, fsr.left.fr, fsr.left.rl, fsr.left.rr};
	float filterX = 0, filterY = 0, filterZ = 0, filterP = 0, filterR = 0;
	float last_filterX = 0, last_filterY = 0, last_filterZ = 0, last_filterPitch = 0, last_filterRoll = 0;
public:
    NAO();

    Lola_state getState();	// Retorna o estado atual
    Lola_state getLastState();	// Retorna o ultimo estado
    bool standingBegin();	// Levantar(inicial)
	bool isStand();	// Verifica se o NAO está de pé
    void stand();	// Ações
	bool isFalling();	// Verifica se o NAO está caíndo
	void falling();	// Ações
	bool isFallen();	// Verifica se o NAO está caído
	void fallen();	// Ações
	bool isStanding();	// Verifica se o NAO está levantando
	void standing();	// Ações
	void finish();	//Ação de finalização
	void shutdown(); // Para o código;	



	bool seguro();
	void fresh();
};

#endif