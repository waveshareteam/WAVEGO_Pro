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
//	LF_B     ^     RF_B
//	LF_A  forward  RF_A
//	  |              |
//	LF_H           RF_H
//
//
//  LH_H           RH_H
//    |              |
//  LH_A           RH_A
//  LH_B           RH_B
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
BodyCtrl::BodyCtrl() : sc() {
    // Constructor
}

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