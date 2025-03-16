#ifndef BODY_CTRL_H
#define BODY_CTRL_H

#include <SCServo.h>

class BodyCtrl{
private:
    SCSCL sc;
    int jointsZeroPos[12]; // array to store the zero position of each joint
    int jointsCurrentPos[12]; // array to store the current position of each joint
    int jointsGoalPos[12]; // array to store the goal position of each joint

public:
    BodyCtrl() {
        // Initialize jointsZeroPos array
        for(int i = 0; i < 12; i++) {
            jointsZeroPos[i] = 512; // or any other default value
            jointsCurrentPos[i] = 512; // or any other default value
            jointsGoalPos[i] = 512; // or any other default value
        }
    }
    void init();
    void jointMiddle();
    void releaseTorque();
    void singleServoCtrl(int id, int pos, int noidea, int speed);
    int* getJointsZeroPosArray();
    void setJointsZeroPosArray(int values[]);
    int* getServoFeedback();
    void setCurrentPosZero();
};

#endif // BODYCTRL_H