#ifndef BODY_CTRL_H
#define BODY_CTRL_H

#include <SCServo.h>

class BodyCtrl{
private:
    SCSCL sc;
    int ServoMiddlePWM[12]; // array to store the zero position of each joint
    int jointsCurrentPos[12]; // array to store the current position of each joint (feedback)
    int legPosBuffer[12] = { 16, 95, 25,
                            -16, 95, 25,
                             16, 95, 25,
                            -16, 95, 25}; // arry to store the position of every leg
    int GoalPWM[12]; // array to store the goal position of each joint

    // int directionArray[12]; // array to store the direction of each joint
    int ServoDirection[16] = {-1,  1,  1,
                               1, -1, -1,
                              -1,  1,  1,
                               1, -1, -1};
    // int jointID[12] = {52, 53, 51, 42, 43, 41, 22, 23, 21, 32, 33, 31};
    int jointID[12] = {53, 52, 51, 41, 42, 43, 23, 22, 21, 31, 32, 33};
    double linkageBuffer[32] = {0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 0, 0};

public:
    BodyCtrl() {
        // Initialize jointsZeroPos array
        for(int i = 0; i < 12; i++) {
            ServoMiddlePWM[i] = 512; // or any other default value
            jointsCurrentPos[i] = 512; // or any other default value
            GoalPWM[i] = 512; // or any other default value
        }
        // directionArray[0] = -1;
        // directionArray[1] = 1; 
        // directionArray[2] = -1;
        // directionArray[3] = 1;
        // directionArray[4] = -1;
        // directionArray[5] = 1;
        // directionArray[6] = 1;
        // directionArray[7] = -1;
        // directionArray[8] = 1;
        // directionArray[9] = -1;
        // directionArray[10] = 1;
        // directionArray[11] = -1;
    }
    void init();
    void jointMiddle();
    void releaseTorque();
    void singleServoCtrl(int id, int pos, int noidea, int speed);
    int* getJointsZeroPosArray();
    void setJointsZeroPosArray(int values[]);
    int* getServoFeedback();
    void setCurrentPosZero();

    void jointAngle(int joint, double angleW);
    double mapDouble(double x, double in_min, double in_max, double out_min, double out_max);
    void jointRad(int joint, double rad);
    void moveTrigger();
    void stand();

    void simpleLinkageIK(double LA, double LB, double aIn, double bIn, uint8_t outputAlpha, uint8_t outputBeta, uint8_t outputDelta);
    void wigglePlaneIK(double LA, double aIn, double bIn, uint8_t outputAlpha, uint8_t outputLen);
    void singleLegPlaneIK(double LS, double LA, double LC, double LD, double LE, double xIn, double yIn, uint8_t outputBeta, uint8_t outputX, uint8_t outputY);
    void singleLegCtrl(uint8_t LegNum, double xPos, double yPos, double zPos);
    void standUp(double cmdInput);
    void singleGaitCtrl(uint8_t LegNum, uint8_t statusInput, float cycleInput, float directionInput, double extendedX, double extendedZ);
    void simpleGait(float GlobalInput, float directionAngle, int turnCmd);
    void triangularGait(float GlobalInput, float directionAngle, int turnCmd);
    void gaitTypeCtrl(float GlobalStepInput, float directionCmd, int turnCmd);
    void standMassCenter(float aInput, float bInput);
    void pitchYawRollHeightCtrl(float pitchInput, float yawInput, float rollInput, float heightInput);
    void balancing(float ACC_Y, float ACC_X);
    void massCenerAdjustTestLoop();


};

#endif // BODYCTRL_H