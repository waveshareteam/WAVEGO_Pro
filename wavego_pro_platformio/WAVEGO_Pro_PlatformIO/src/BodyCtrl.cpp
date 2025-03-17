#include "BodyCtrl.h"
#include "Config.h"

// define joints and servo id.
// Legs:
// 		F - Front
// 		H - Hind
// 		L - Left
// 		R - Right
// --- --- --- --- --- ---
// joints:
// 		H - Hip
//      A - the servo near the Hip.
//      B - the one further from the Hip.
// --- --- --- --- --- ---
// WAVEGO:	
//	LF_B[1]     ^     RF_B[7]
//	LF_A[0]  forward  RF_A[6]
//	  |                 |
//	LF_H[2]           RF_H[8]
//
//
//  LH_H[5]           RH_H[11]
//    |                 |
//  LH_A[3]           RH_A[9]
//  LH_B[4]           RH_B[10]
// --- --- --- --- --- ---
// bus servos:
#define LEG_LF_A	52
#define LEG_LF_B	53
#define LEG_LF_H	51

#define LEG_LH_A	42
#define LEG_LH_B	43
#define LEG_LH_H	41

#define LEG_RF_A	22
#define LEG_RF_B	23
#define LEG_RF_H	21

#define LEG_RH_A	32
#define LEG_RH_B	33
#define LEG_RH_H	31

// structural args:
// the distance between wiggle servo and the plane of the leg linkages.
//   [A/B]---linkage_w---H(Hip)
//                       |
//                       |
//                       |
//                       |
//                       |
//                     Ground
double linkage_w = 19.15;

// --- --- --- --- --- ---
//                  [Hip]
//                    | < l_f
//     <<<[B]-s--[A]<<<
//         /     |
//       l_a    l_a
//       /       |
//      O        O
//      |       /
//     l_b    l_c
//      |     /
//      |    /
//      |   /
//      |  /
//      | /
//      O
//     /  
//   l_d
//   /
//  <---90°
//      .l_e
//           .
// ------------X-----------
double linkage_s = 12.2;    // the distance between two servos.
double linkage_a = 40.0;    // the linkage that connected with the servo.
double linkage_b = 40.0;    // the linkage that limit the direction.
double linkage_c = 39.8153; // the upper part of the leg.
double linkage_d = 31.7750; // the lower part of the leg.
double linkage_e = 30.8076; // the foot.
double linkage_f = 0;       // the distance between Hip and A/B in vertical plane.


// --- --- --- --- --- ---
void BodyCtrl::init() {
    // Initialize BodyCtrl
    Serial1.begin(1000000, SERIAL_8N1, BUS_SERVO_RX, BUS_SERVO_TX);
    sc.pSerial = &Serial1;
    while(!Serial1) {}
}

void BodyCtrl::jointMiddle() {
    // Joint middle
    sc.WritePos(254, 512, 0, 0);
}

void BodyCtrl::releaseTorque() {
    // Release torque
    sc.EnableTorque(254, 0);
}

void BodyCtrl::singleServoCtrl(int id, int pos, int duration, int speed) {
    sc.WritePos(id, pos, duration, speed);
}

int* BodyCtrl::getJointsZeroPosArray() {
    return jointsZeroPos;
}

void BodyCtrl::setJointsZeroPosArray(int values[]) {
    for (int i = 0; i < 12; i++) {
        jointsZeroPos[i] = values[i];
    }
}

int* BodyCtrl::getServoFeedback() {
    jointsCurrentPos[0] = sc.ReadPos(LEG_LF_A);
    jointsCurrentPos[1] = sc.ReadPos(LEG_LF_B);
    jointsCurrentPos[2] = sc.ReadPos(LEG_LF_H);
    jointsCurrentPos[3] = sc.ReadPos(LEG_LH_A);
    jointsCurrentPos[4] = sc.ReadPos(LEG_LH_B);
    jointsCurrentPos[5] = sc.ReadPos(LEG_LH_H);
    jointsCurrentPos[6] = sc.ReadPos(LEG_RF_A);
    jointsCurrentPos[7] = sc.ReadPos(LEG_RF_B);
    jointsCurrentPos[8] = sc.ReadPos(LEG_RF_H);
    jointsCurrentPos[9] = sc.ReadPos(LEG_RH_A);
    jointsCurrentPos[10] = sc.ReadPos(LEG_RH_B);
    jointsCurrentPos[11] = sc.ReadPos(LEG_RH_H);
    return jointsCurrentPos;
}

void BodyCtrl::setCurrentPosZero() {
    BodyCtrl::getServoFeedback();
    for (int i = 0; i < 12; i++) {
        jointsZeroPos[i] = jointsCurrentPos[i];
    }
}

void BodyCtrl::jointAngle(int joint, double angleW) {
    // joint: 1-12
    // angleW: 0-220
    // angle: 0-1023
    int angle = round(map(angleW, 0, 300, 0, 1024));
    jointsGoalPos[joint] = angle * directionArray[joint] + jointsZeroPos[joint];
    sc.RegWritePos(jointID[joint], jointsGoalPos[joint], 0, 0);
}

double BodyCtrl::mapDouble(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void BodyCtrl::jointRad(int joint, double rad) {
    // joint: 1-12
    // rad: 0-5.23598
    // angle: 0-1023
    int angle = round(BodyCtrl::mapDouble(rad, 0, 5.23598, 0, 1024));
    jointsGoalPos[joint] = angle * directionArray[joint] + jointsZeroPos[joint];
    sc.RegWritePos(jointID[joint], jointsGoalPos[joint], 0, 0);
}

void BodyCtrl::moveTrigger(){
    sc.RegWriteAction();
}

void BodyCtrl::stand() {
    BodyCtrl::jointAngle(0, 45);
    BodyCtrl::jointAngle(1, 45);
    BodyCtrl::jointAngle(2, 0);

    BodyCtrl::jointAngle(3, 45);
    BodyCtrl::jointAngle(4, 45);
    BodyCtrl::jointAngle(5, 0);

    BodyCtrl::jointAngle(6, 45);
    BodyCtrl::jointAngle(7, 45);
    BodyCtrl::jointAngle(8, 0);

    BodyCtrl::jointAngle(9, 45);
    BodyCtrl::jointAngle(10, 45);
    BodyCtrl::jointAngle(11, 0);

    BodyCtrl::moveTrigger();
}





// WAVEGO:	
//	LF_B[1]     ^     RF_B[7]
//	LF_A[0]  forward  RF_A[6]
//	  |                 |
//	LF_H[2]           RF_H[8]
//
//
//  LH_H[5]           RH_H[11]
//    |                 |
//  LH_A[3]           RH_A[9]
//  LH_B[4]           RH_B[10]
// --- --- --- --- --- ---
// bus servos:
// #define LEG_LF_A	52
// #define LEG_LF_B	53
// #define LEG_LF_H	51

// #define LEG_LH_A	42
// #define LEG_LH_B	43
// #define LEG_LH_H	41

// #define LEG_RF_A	22
// #define LEG_RF_B	23
// #define LEG_RF_H	21

// #define LEG_RH_A	32
// #define LEG_RH_B	33
// #define LEG_RH_H	31