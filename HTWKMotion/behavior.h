#pragma once

#include "../LolaConnector/leds.h"

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
