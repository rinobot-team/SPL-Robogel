#include "GameController.hpp"
#include <arpa/inet.h>
#include <ctime>
#include <dirent.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/poll.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string>
#include <time.h>
#include "blackboard/Blackboard.hpp" // TODO (Abreu)

/*
    Some methods and variables are supposed to be implemented in Blackboard.hpp, like readFrom() and writeTo()
*/

#define POLL_TIMEOUT 200

using namespace std;

const uint8_t PACKETS_PER_SECS = 2;

const uint8_t LEAVE_WIFI_SECS = 20;

const double JITTER_BUFFER = 0.5;

const uint8_t min_packets = JITTER_BUFFER + LEAVE_WIFI_SECS + PACKETS_PER_SECS;

GameController::GameController(BlackBoard *blackboard) : AccessPoint(blackboard), teamInfo(NULL), connected(false) {
    lastState = STATE_INITIAL;
    myLastPenalty = PENALTY_NONE;
    if(readFrom(gameController, connect))
        initialiseConnection();
}

GameController::~GameController() {
    close(sock);
}

GameController::tick() {
    uint8_t previousGameState = readFrom(gameController, gameState);
    data = readFrom(gameController, data);
    teamNumber = readFrom(gameController, teamInfo).teamNumber;
    playerNumber = readFrom(gameController, player_number);
    setOurTeam();

    if(!connected && readFrom(gameController, connect)) 
        initialiseConnection();

    if(connected) wirelessUpdate();

}