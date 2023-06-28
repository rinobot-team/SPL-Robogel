#include "LolaConnector/lola_connector.h"
#include "HTWKMotion/ankle_balancer.h"
#include "HTWKMotion/arm_controller.h"
#include "HTWKMotion/sit_motion.h"
#include "HTWKMotion/walking_engine.h"
#include "utils/imu.h"
#include "nao.h"

#include <cmath>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <map>

#include <boost/asio.hpp>
#include <msgpack.hpp>

#include <lola_frame.h>

void NAO::NAO(){
    setState(Lola_state::begin);
    setLastState();
}

void NAO::getState(){
    return state;
}

void NAO::getLastState(){
    return last_state;
}

void NAO::setState(Lola_state newEstado){
    state = newEstado;
}

void NAO::setLastState(){
    last_state = state;
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
 