#pragma once

#include "ankle_balancer.h"
#include "arm_controller.h"
#include "../LolaConnector/leds.h"
#include "walking_engine.h"
#include "sit_motion.h"

struct Colors {
    static constexpr RGB RED{1.f, 0.f, 0.f};
    static constexpr RGB GREEN{0.f, 1.f, 0.f};
    static constexpr RGB BLUE{0.f, 0.f, 1.f};
    static constexpr RGB YELLOW{1.f, 1.f, 0.f};
    static constexpr RGB OFF{0.f, 0.f, 0.f};
    static constexpr RGB PURPLE{0.5f, 0.f, 0.5f};
    static constexpr RGB ORANGE{1.f, 0.5f, 0.f};
};

enum class GameState {
	UNSTIFF, 
	INITIAL, 
	READ, 
	SET, 
	PLAYING, 
	PENALIZED, 
	FINISHED, 
	CALIBRATION
};

class Behavior {
    public:
        Behavior();
        GameState getEstadoDeJogo();
        void setEyeColor(RGB cor);
        void unstiff();
        void initial();
        void ready();
        void set();
        void playing();
        void penalized();
        void finished();
        void calibration();
        RGB geteyecolor() const {return eyeColor;};
     

    private:	
        RGB eyeColor;
        GameState estadoDeJogo;
};