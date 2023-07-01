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
#include <iostream>
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

void GameController::tick() {
    uint8_t previousGameState = readFrom(gameController, gameState);
    data = readFrom(gameController, data);
    teamNumber = readFrom(gameController, teamInfo).teamNumber;
    playerNumber = readFrom(gameController, player_number);
    setOurTeam();

    if(!connected && readFrom(gameController, connect)) 
        initialiseConnection();

    if(connected) wirelessUpdate();

    uint8_t gameState = data.state;
    
    // Dont know what to do; runswift depends the whole tick method on the whistle detection
}

void GameController::initialiseConnection() {
    stringstream s;
    s << GAMECONTROLLER_DATA_PORT;
    
    struct addrinfo info, *res;
    memset(&info, 0, sizeof info);
    info.ai_family = AF_UNSPEC;
    info.ai_socktype = SOCK_DGRAM;
    info.ai_flags = AI_PASSIVE;

    if(getaddrinfo(NULL, s.str().c_str(), &info, &res) == -1) {
        std::cerr << "GameController: INVALID ADDR INFO" << std::endl;
        return;
    }

    struct addrinfo *p;
    for (p = res; p != NULL; p = p->ai_next) {
        if((sock = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
            std::cerr << "GameController: COULD NOT USE SOCK, TRYING NEXT" << std::endl;
            continue;
        }

        int enable = 1;
        if(setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) != 0) {
            std::cerr << "GameController: COULD NOT SET SOCKET OPTIONS"; std::endl;
            continue;
        }

        if(bind(sock, p->ai_addr, p->ai_addrlen) == -1) {
            close(sock);
            std::cerr << "GameController: CANNOR BIND, TRYING NEXT" << std::endl;
            continue;
        }

        break;
    }

    if(p == NULL) {
        std::cerr << "GameController: FAILED TO BIND SOCKET" << std::endl;
        return;
    }

    freeaddrinfo(res);

    std::cout << "GameController: CONNECTED TO PORT - " << s.str() << std::endl;
    connected = true;
    writeTo(gameController, connected, connected);
}

void GameController::wirelessUpdate() {
    int bytesReceived;
    struct sockaddr_storage clientAddr;
    socklen_t addr_len = sizeof(clientAddr);

    int dataSize = sizeof(RoboCupGameControlData);
    unsigned char buffer[dataSize + 1];

    struct pollfd ufds[1];
    ufds[0].fd = sock;
    ufds[0].events = POLLIN;

    for(int i = 0; i < 5; i++) {
        int rv = poll(ufds, 1, POLL_TIMEOUT);

        if(rv > 0) {
            bytesReceived = recvfrom(sock, buffer, dataSize, 0, (struct sockadrr *)&clientAddr, &addr_len);
            writeTo(gameController, lastGameControllerIPAdress, inet_ntoa((struct sockaddr_in *)*clientAddr))->sin+addr;
            if(bytesReceived > 0) {
                parseData((RoboCupGameControlData*)buffer);
                handleFinishedPacket();
            }
        }
    }
}

