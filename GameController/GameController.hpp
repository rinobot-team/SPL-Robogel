#ifndef GAMECONTROLLER_H
#define GAMECONTROLLER_H

#include <vector>
#include "RoboCupGameControlData.hpp"
#include "blackboard/AccessPoint.hpp" // TODO (Abreu)

class GameController : AccessPoint {
public:
    GameController(Blackboard *blackboard);
    ~GameController();
    void tick();

private:
    RoboCupGameControlData data;
    TeamInfo *teamInfo;
    bool connected;
    int sock;

    // Connect to GC
    void initialiseConnection();

    // Update state via Button
    void buttonUpdate();

    // Update state via Wi-Fi
    void wirelessUpdate();

    void handleFinishedPacket();

    // Parse data from GC
    void parseData(RoboCupGameControlData *update);

    // Parser helper functions
    bool isValidData(RoboCupGameControlData *data);
    bool checkHeader(char* header);
    bool isThisGame(RoboCupGameControlData *gameData);
    bool gameDataEqual(void* gameData, void* previous);
    void rawSwapTeams(RoboCupGameControlData *gameData);

    int playerNumber;
    int teamNumber;

    void setOurTeam();
};

#endif