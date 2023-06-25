#include "behavior.h"
#include "sit_motion.h"
#include "../LolaConnector/leds.h"



Behavior::Behavior(){
    estadoDeJogo = GameState::UNSTIFF;
    eyeColor = {1.f,1.f,0.f};
}

void Behavior::unstiff(){
    //função ficar de cócoras
    //joints.legs = sit_motion.sitDown(sensor_frame.joints.legs, ankle_balancer, &arm_controller);
    //desenergizar juntas
    //set_stiffness(-1.f, &frame_handler.actuator_frame.joints.legs);
	//set_stiffness(-1.f, &frame_handler.actuator_frame.joints.arms);
	//set_stiffness(-1.f, &frame_handler.actuator_frame.joints.head);
    //olho azul piscando
    eyeColor = {0.f, 0.f, 1.f};
}

void Behavior::initial(){
    //Levantar
    // joints.legs = sit_motion.getUp(sensor_frame.joints.legs, ankle_balancer, &arm_controller);
	// joints.arms = arm_controller.proceed();
	// walking_engine.reset();
}

void Behavior::ready(){
    //Anda até sua posição no campo
    
}

void Behavior::set(){
    //"Esperando ordem do juiz"
    //podem mover os braços, a cabeça ou levantar, mas n pode sair do lugar(excessão para se mover ao levantar)
    //pd chamar subestados caso não cumpra alguma parada
}

void Behavior::playing(){
    //jogando
    //apertar botão no peito muda para penalizado
}

void Behavior::penalized(){
    //único movimento permitido: se levantar depois de cair
}

void Behavior::finished(){
    //cabô primeiro tempo
}

void Behavior::calibration(){
    //calibração automática
}
