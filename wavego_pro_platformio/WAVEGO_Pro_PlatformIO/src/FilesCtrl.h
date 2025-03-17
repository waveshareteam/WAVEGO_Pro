#ifndef FILES_CTRL_H
#define FILES_CTRL_H

#include <Arduino.h>

class FilesCtrl {
private:
    bool flashStatus = false;

public:
    FilesCtrl() {
        // Initialize FilesCtrl
    }
    void init();
    void flash();
    void scan();
    void createMission(String missionName, String content);
    void missionContent(String missionName);
    void appendStep(String missionName, String jsonCmd);
    void insertStep(String missionName, int stepNum, String jsonCmd);
    void replaceStep(String missionName, int stepNum, String jsonCmd);
    void deleteStep(String missionName, int stepNum);
    String readStep(String missionName, int stepNum);
    void deleteMission(String missionName);
    bool checkMission(String missionName);
};

#endif // FILES_CTRL_H