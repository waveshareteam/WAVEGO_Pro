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
};

#endif // FILES_CTRL_H