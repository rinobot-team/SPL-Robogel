#include "behavior.h"

//Behavior estadoDeJogo;
//frame_handler.actuator_frame.leds.eyes.left.fill(estadoDeJogo.geteyecolor());

Behavior::Behavior(){
    estadoDeJogo = GameState::UNSTIFF;
    eyeColor = Colors::BLUE;
}

GameState Behavior::getEstadoDeJogo(){
    return estadoDeJogo;
}

void Behavior::setEyeColor(RGB cor){
    eyeColor = cor;
}

void Behavior::unstiff(){
    //função ficar de cócoras
    // joints.legs = sit_motion.sitDown(sensor_frame.joints.legs, ankle_balancer, &arm_controller);
    //desenergizar juntas
    // set_stiffness(-1.f, &frame_handler.actuator_frame.joints.legs);
	// set_stiffness(-1.f, &frame_handler.actuator_frame.joints.arms);
	// set_stiffness(-1.f, &frame_handler.actuator_frame.joints.head);
    //olho azul piscando
    eyeColor = Colors::BLUE; 
}

/*switch(estadoDeGame.getEstadoDeJogo()){
			case GameState::UNSTIFF:
				joints.legs = sit_motion.sitDown(sensor_frame.joints.legs, ankle_balancer, &arm_controller);
				set_stiffness(-1.f, &frame_handler.actuator_frame.joints.legs);
				set_stiffness(-1.f, &frame_handler.actuator_frame.joints.arms);
				set_stiffness(-1.f, &frame_handler.actuator_frame.joints.head);
				break;
			case GameState::INITIAL:
				joints.legs = sit_motion.getUp(sensor_frame.joints.legs, ankle_balancer, &arm_controller);
				joints.arms = arm_controller.proceed();
				walking_engine.reset();
				break;
			default:
				
				break;
		}*/

void Behavior::initial(){
    //Levantar
    // joints.legs = sit_motion.getUp(sensor_frame.joints.legs, ankle_balancer, &arm_controller);
	// joints.arms = arm_controller.proceed();
	// walking_engine.reset();
    eyeColor = Colors::OFF;
}

void Behavior::ready(){
    //Anda até sua posição no campo
    eyeColor = Colors::BLUE;
    
}

void Behavior::set(){
    //"Esperando ordem do juiz"
    //podem mover os braços, a cabeça ou levantar, mas n pode sair do lugar(excessão para se mover ao levantar)
    //pd chamar subestados caso não cumpra alguma parada
    eyeColor = Colors::YELLOW;
}

void Behavior::playing(){
    //jogando
    //apertar botão no peito muda para penalizado
    eyeColor = Colors::GREEN;
}

void Behavior::penalized(){
    //único movimento permitido: se levantar depois de cair
    eyeColor = Colors::RED;
}

void Behavior::finished(){
    //cabô primeiro tempo
    eyeColor = Colors::OFF;
}

void Behavior::calibration(){
    //calibração automática
    eyeColor = Colors::PURPLE;
}
