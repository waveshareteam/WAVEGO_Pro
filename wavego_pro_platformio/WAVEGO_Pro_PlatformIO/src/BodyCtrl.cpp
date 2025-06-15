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

#define LEG_A_FORE 0
#define LEG_A_BACK 1
#define LEG_A_WAVE 2

#define LEG_B_WAVE 3
#define LEG_B_FORE 4
#define LEG_B_BACK 5

#define LEG_C_FORE 6
#define LEG_C_BACK 7
#define LEG_C_WAVE 8

#define LEG_D_WAVE 9
#define LEG_D_FORE 10
#define LEG_D_BACK 11

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
double wiggleError = 0;

// --- --- --- --- --- ---
//                  [Hip]
//                    | < l_f = wiggleError
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
double linkage_f = wiggleError;       // the distance between Hip and A/B in vertical plane.

double WALK_HEIGHT_MAX  = 110;
double WALK_HEIGHT_MIN  = 75;
double WALK_HEIGHT      = 95;
double WALK_LIFT        = 9; // WALK_HEIGHT + WALK_LIFT <= WALK_HEIGHT_MAX.
double WALK_RANGE       = 40;
double WALK_ACC         = 5;
double WALK_EXTENDED_X  = 16;
double WALK_EXTENDED_Z  = 25;
double WALK_SIDE_MAX    = 30;
double WALK_MASS_ADJUST = 21;
double STAND_HEIGHT     = 95;

uint8_t WALK_STATUS     = 0;
float WALK_CYCLE_GLOBAL = 0;    // 0-1.
float WALK_LIFT_PROP    = 0.25; // WALK_LIFT_TIME<1.

float BALANCE_PITCHU_BUFFER;
float BALANCE_ROLL_BUFFER;
float BALANCE_PITCHU_BASE = 0;
float BALANCE_ROLL_BASE   = 0;
float BALANCE_P = 0.72;

float GLOBAL_STEP  = 0;
int   STEP_DELAY   = 5;
float STEP_ITERATE = 0.02;

int SERVO_MOVE_EVERY = 0;



// --- pre render ---
float LAxLA = linkage_a*linkage_a;
float LBxLB = linkage_b*linkage_b;
float LWxLW = linkage_w*linkage_w;
float LExLE = linkage_e*linkage_e;
float LAxLA_LBxLB = LAxLA - LBxLB;
float LBxLB_LAxLA = LBxLB - LAxLA;
float L_CD = (linkage_c+linkage_d)*(linkage_c+linkage_d);
float LAx2  = 2 * linkage_a;
float LBx2  = 2 * linkage_b;
float E_PI  = 180 / M_PI;
float LSs2  = linkage_s/2;
float aLCDE = atan((linkage_c + linkage_d)/linkage_e);
float sLEDC = sqrt(linkage_e*linkage_e + (linkage_d+linkage_c)*(linkage_d+linkage_c));
float O_WLP = 1 - WALK_LIFT_PROP;
float WALK_ACCx2 = WALK_ACC*2;
float WALK_H_L = WALK_HEIGHT - WALK_LIFT;

// ctrl interface.
// refer to OLED screen display for more detail information.
int moveFB = 0;
int moveLR = 0;
int debugMode = 0;
int funcMode  = 0;
float gestureUD = 0;
float gestureLR = 0;
float gestureOffSetMax = 15;
float gestureSpeed = 2;
int STAND_STILL = 0;

// gait type ctrl
// 0: simpleGait(DiagonalGait).
// 1: triangularGait.
int GAIT_TYPE = 0;

// --- --- --- --- --- ---
void BodyCtrl::init() {
    // Initialize BodyCtrl
    Serial1.begin(1000000, SERIAL_8N1, BUS_SERVO_RX, BUS_SERVO_TX);
    sc.pSerial = &Serial1;
    while(!Serial1) {}
}

void BodyCtrl::jointMiddle() {
    // Joint middle
    sc.WritePos(254, 511, 0, 0);
}

void BodyCtrl::releaseTorque() {
    // Release torque
    sc.EnableTorque(254, 0);
}

void BodyCtrl::singleServoCtrl(int id, int pos, int duration, int speed) {
    sc.WritePos(id, pos, duration, speed);
}

int* BodyCtrl::getJointsZeroPosArray() {
    return ServoMiddlePWM;
}

void BodyCtrl::setJointsZeroPosArray(int values[]) {
    for (int i = 0; i < 12; i++) {
        ServoMiddlePWM[i] = values[i];
    }
}

int* BodyCtrl::getServoFeedback() {
    for (int i = 0; i < 12; i++) {
        jointsCurrentPos[i] = sc.ReadPos(jointID[i]);
    }
    return jointsCurrentPos;
}

void BodyCtrl::setCurrentPosZero() {
    BodyCtrl::getServoFeedback();
    for (int i = 0; i < 12; i++) {
        ServoMiddlePWM[i] = jointsCurrentPos[i];
    }
}

void BodyCtrl::jointAngle(int joint, double angleW) {
    // joint: 0-11
    // angleW: 0-300
    // angle: 0-1023
    int angle = round(map(angleW, 0, 300, 0, 1024));
    GoalPWM[joint] = angle * ServoDirection[joint] + ServoMiddlePWM[joint];
    sc.RegWritePos(jointID[joint], GoalPWM[joint], 0, 0);
}

void BodyCtrl::allJointAngle(double angleWs[]) {
    // joint: 0-11
    // angleW: 0-30
    // angle: 0-1023
    for (int i = 0; i < 12; i++) {
        int angle = round(map(angleWs[i], 0, 300, 0, 1024));
        GoalPWM[i] = angle * ServoDirection[i] + ServoMiddlePWM[i];
    }
    sc.SyncWritePos(jointID, 12, GoalPWM, 0, speedArray);
}

double BodyCtrl::mapDouble(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void BodyCtrl::jointRad(int joint, double rad) {
    // joint: 1-12
    // rad: 0-5.23598
    // angle: 0-1023
    int angle = round(BodyCtrl::mapDouble(rad, 0, 5.23598, 0, 1024));
    GoalPWM[joint] = angle * ServoDirection[joint] + ServoMiddlePWM[joint];
    sc.RegWritePos(jointID[joint], GoalPWM[joint], 0, 0);
}

void BodyCtrl::moveTrigger(){
    sc.RegWriteAction();
}

void BodyCtrl::stand() {
    BodyCtrl::jointAngle(0, 45);
    BodyCtrl::jointAngle(1, 45);
    BodyCtrl::jointAngle(2, 0);

    BodyCtrl::jointAngle(3, 0);
    BodyCtrl::jointAngle(4, 45);
    BodyCtrl::jointAngle(5, 45);

    BodyCtrl::jointAngle(6, 45);
    BodyCtrl::jointAngle(7, 45);
    BodyCtrl::jointAngle(8, 0);

    BodyCtrl::jointAngle(9, 0);
    BodyCtrl::jointAngle(10, 45);
    BodyCtrl::jointAngle(11, 45);

    BodyCtrl::moveTrigger();
}

// Simple Linkage IK:
// input the position of the end and return angle.
//   O----O
//  /
// O
// ---------------------------------------------------
// |       /beta           /delta                    |
//        O----LB---------X------                    |
// |     /       omega.   |       \LB                |
//      LA        .                < ----------------|
// |alpha     .          bIn         \LB -EP  <delta |
//    /psi.                           \LB -EP        |
// | /.   lambda          |                          |
// O- - - - - aIn - - - - X -                        |
// ---------------------------------------------------
// alpha, beta > 0 ; delta <= 0 ; aIn, bIn > 0
void BodyCtrl::simpleLinkageIK(double LA, double LB, double aIn, double bIn, uint8_t outputAlpha, uint8_t outputBeta, uint8_t outputDelta){
    double psi;
    double alpha;
    double omega;
    double beta;
    double L2C;
    double LC;
    double lambda;
    double delta;
    if(bIn == 0){
      psi   = acos((LAxLA_LBxLB + aIn * aIn)/(LAx2 * aIn)) * E_PI;
      alpha = 90 - psi;
      omega = acos((aIn * aIn + LBxLB_LAxLA)/(LBx2 * aIn)) * E_PI;
      beta  = psi + omega;
    }
    else{
      L2C = aIn * aIn + bIn * bIn;
      LC  = sqrt(L2C);
      lambda = atan(bIn/aIn) * 180 / M_PI;
      psi    = acos((LAxLA_LBxLB + L2C)/(2 * LA * LC)) * E_PI;
      alpha  = 90 - lambda - psi;
      omega  = acos((LBxLB_LAxLA + L2C)/(2 * LC * LB)) * E_PI;
      beta   = psi + omega;
    }
    delta = 90 - alpha - beta;
    linkageBuffer[outputAlpha] = alpha;
    linkageBuffer[outputBeta]  = beta;
    linkageBuffer[outputDelta] = delta;
}

// Wiggle Plane IK:
// input the position of the end and return angle.
// O-----X
//       |
//       |
//       O
// ------------------------------
//     X                        |
//    /    .                    
//  LA         .                |
//  /alpha         .LB         
// O- - - - - - - - - -.- - - -X|
//                         .  bIn
// ------------aIn-------------X|
// ------------------------------
// alpha, aIn, bIn > 0
void BodyCtrl::wigglePlaneIK(double LA, double aIn, double bIn, uint8_t outputAlpha, uint8_t outputLen){
    double LB;
    double L2C;
    double LC;
    double alpha;
    double beta;
    double lambda;
    double psi;
    if(bIn > 0){
      L2C = aIn * aIn + bIn * bIn;
      LC = sqrt(L2C);
      lambda = atan(aIn/bIn) * E_PI;
      psi = acos(LA/LC) * E_PI;
      LB = sqrt(L2C - LWxLW);
      alpha = psi + lambda - 90;
    }
    else if(bIn == 0){
      alpha = asin(LA/aIn) * E_PI;
      L2C = aIn * aIn + bIn * bIn;
      LB = sqrt(L2C);
    }
    else if(bIn < 0){
      bIn = -bIn;
      L2C = aIn * aIn + bIn * bIn;
      LC = sqrt(L2C);
      lambda = atan(aIn/bIn) * E_PI;
      psi = acos(LA/LC) * E_PI;
      LB = sqrt(L2C - LWxLW);
      alpha = 90 - lambda + psi;
    }
    linkageBuffer[outputAlpha] = alpha;
    linkageBuffer[outputLen]  = LB - wiggleError;
}

// Ctrl single leg plane IK:
// input the args of the leg, return angle and position.
//  SE--------LS(O)-----SE
//             |        |  \.
//                      |   LA
//             |        |beta\.
//       \.                   \.
//        \.   |               O
//         \.              .
//          LB |       LC
//           \.    .
//             O
//         LD
//     .       |
// F
//  \.         |
//   LE       yIn
//    \.       |
//     \.--xIn-X  
// beta > 0 ; xIn, yIn > 0
void BodyCtrl::singleLegPlaneIK(double LS, double LA, double LC, double LD, double LE, double xIn, double yIn, uint8_t outputBeta, uint8_t outputX, uint8_t outputY){
    double bufferS = sqrt((xIn + LSs2)*(xIn + LSs2) + yIn*yIn);
    double lambda = acos(((xIn + LSs2)*(xIn + LSs2) + yIn*yIn + LAxLA - L_CD - LExLE)/(2*bufferS*LA));
    double delta = atan((xIn + LSs2)/yIn);
    double beta = lambda - delta;
    double betaAngle = beta * E_PI;
  
    double theta = aLCDE;
    double omega = asin((yIn - cos(beta)*LA)/sLEDC);
    double nu = M_PI - theta - omega;
    double dFX = cos(nu)*LE;
    double dFY = sin(nu)*LE;
  
    double mu = M_PI/2 - nu;
    double dEX = cos(mu)*LD;
    double dEY = sin(mu)*LD;
  
    double positionX = xIn + dFX - dEX;
    double positionY = yIn - dFY - dEY;
    
    linkageBuffer[outputBeta] = betaAngle;
    linkageBuffer[outputX]  = positionX;
    linkageBuffer[outputY]  = positionY;
}

// Ctrl a single leg of WAVEGO, it moves in a plane.
// input (x,y) position and return angle alpha and angle beta.
//     O  X  O
//    /         .
//   /    |        O
//  O     y     .
//   \.   |  .
//    \.  .
//     O  |
//  .
//   \.   |
//    \-x-X
// ------------------
// x, y > 0
void BodyCtrl::singleLegCtrl(uint8_t LegNum, double xPos, double yPos, double zPos){
    uint8_t alphaOut;
    uint8_t xPosBuffer;
    uint8_t yPosBuffer;
    uint8_t betaOut;
    uint8_t betaB;
    uint8_t betaC;
    uint8_t NumF;
    uint8_t NumB;
    uint8_t wiggleAlpha;
    uint8_t wiggleLen;
    uint8_t NumW;
    if(LegNum == 1){
      NumF = LEG_A_FORE;
      NumB = LEG_A_BACK;
      NumW = LEG_A_WAVE;
      alphaOut   = 0;
      xPosBuffer = 1;
      yPosBuffer = 2;
      betaOut = 3;
      betaB   = 4;
      betaC   = 5;
      wiggleAlpha = 6;
      wiggleLen   = 7;
    }
    else if(LegNum == 2){
      NumF = LEG_B_FORE;
      NumB = LEG_B_BACK;
      NumW = LEG_B_WAVE;
      alphaOut   = 8;
      xPosBuffer = 9;
      yPosBuffer = 10;
      betaOut = 11;
      betaB   = 12;
      betaC   = 13;
      wiggleAlpha = 14;
      wiggleLen   = 15;
    }
    else if(LegNum == 3){
      NumF = LEG_C_FORE;
      NumB = LEG_C_BACK;
      NumW = LEG_C_WAVE;
      alphaOut   = 16;
      xPosBuffer = 17;
      yPosBuffer = 18;
      betaOut = 19;
      betaB   = 20;
      betaC   = 21;
      wiggleAlpha = 22;
      wiggleLen   = 23;
    }
    else if(LegNum == 4){
      NumF = LEG_D_FORE;
      NumB = LEG_D_BACK;
      NumW = LEG_D_WAVE;
      alphaOut   = 24;
      xPosBuffer = 25;
      yPosBuffer = 26;
      betaOut = 27;
      betaB   = 28;
      betaC   = 29;
      wiggleAlpha = 30;
      wiggleLen   = 31;
    }
  
    wigglePlaneIK(linkage_w, zPos, yPos, wiggleAlpha, wiggleLen);
    singleLegPlaneIK(linkage_s, linkage_a, linkage_c, linkage_d, linkage_e, xPos, linkageBuffer[wiggleLen], alphaOut, xPosBuffer, yPosBuffer);
    simpleLinkageIK(linkage_a, linkage_b, linkageBuffer[yPosBuffer], (linkageBuffer[xPosBuffer]-linkage_s/2), betaOut, betaB, betaC);
  
    // jointAngle(NumW, linkageBuffer[wiggleAlpha]);
    // jointAngle(NumF, (90 - linkageBuffer[betaOut]));
    // jointAngle(NumB, linkageBuffer[alphaOut]);

    GoalAngle[NumW] = linkageBuffer[wiggleAlpha];
    GoalAngle[NumF] = (90 - linkageBuffer[betaOut]);
    GoalAngle[NumB] = linkageBuffer[alphaOut];
}

void BodyCtrl::standUp(double cmdInput){
    singleLegCtrl(1, WALK_EXTENDED_X, cmdInput, WALK_EXTENDED_Z);
    singleLegCtrl(2, -WALK_EXTENDED_X, cmdInput, WALK_EXTENDED_Z);
    singleLegCtrl(3, WALK_EXTENDED_X, cmdInput, WALK_EXTENDED_Z);
    singleLegCtrl(4, -WALK_EXTENDED_X, cmdInput, WALK_EXTENDED_Z);
    allJointAngle(GoalAngle);
}

// Ctrl the gait of a single leg with the variable cycleInput change between 0-1.
// when the directionInput > 0, the front point in the gait cycle move away from the middle line of the robot.
// use the extendedX and extendedZ to adjust the position of the middle point in a wiggle cycle.
// statusInput used to reduce the WALK_RANGE.
void BodyCtrl::singleGaitCtrl(uint8_t LegNum, uint8_t statusInput, float cycleInput, float directionInput, double extendedX, double extendedZ){
    double rDist;
    double xGait;
    double yGait;
    double zGait;
    double rDiection = directionInput * M_PI / 180;
  
    if(cycleInput < (1 - WALK_LIFT_PROP)){
      if(cycleInput <= (WALK_ACC/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP)){
        yGait = (WALK_HEIGHT - WALK_LIFT) + cycleInput/(1-WALK_LIFT_PROP-((WALK_ACC+WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP))*WALK_LIFT;
      }
      else if(cycleInput > (WALK_ACC/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP) && cycleInput <= ((WALK_ACC + WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP)){
        yGait = WALK_HEIGHT;
      }
      else if(cycleInput > ((WALK_ACC + WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP) && cycleInput < ((WALK_ACC*2 + WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP)){
        yGait = WALK_HEIGHT - ((cycleInput-((WALK_ACC + WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP))/((WALK_ACC/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP)))*WALK_LIFT;
      }
  
      rDist = (WALK_RANGE*statusInput/2 + WALK_ACC) - (cycleInput/(1 - WALK_LIFT_PROP))*(WALK_RANGE*statusInput + WALK_ACC*2);
    }
    else if(cycleInput >= (1 - WALK_LIFT_PROP)){
      yGait = WALK_HEIGHT - WALK_LIFT;
      rDist = - (WALK_RANGE*statusInput/2 + WALK_ACC) + ((cycleInput-(1-WALK_LIFT_PROP))/WALK_LIFT_PROP)*(WALK_RANGE*statusInput + WALK_ACC*2);
    }
  
    xGait = cos(rDiection) * rDist;
    zGait = sin(rDiection) * rDist;
    singleLegCtrl(LegNum, (xGait + extendedX), yGait, (zGait + extendedZ));
}

// a simple gait to ctrl the robot.
// GlobalInput changes between 0-1.
// use directionAngle to ctrl the direction.
// turnCmd-> -1:left 1:right
void BodyCtrl::simpleGait(float GlobalInput, float directionAngle, int turnCmd){
    float Group_A;
    float Group_B;
  
    Group_A = GlobalInput;
    Group_B = GlobalInput+0.5;
    if(Group_B>1){Group_B--;}
  
    if(!turnCmd){
      singleGaitCtrl(1, 1, Group_A, directionAngle,  WALK_EXTENDED_X, WALK_EXTENDED_Z);
      singleGaitCtrl(4, 1, Group_A, -directionAngle, -WALK_EXTENDED_X, WALK_EXTENDED_Z);
  
      singleGaitCtrl(2, 1, Group_B, directionAngle, -WALK_EXTENDED_X, WALK_EXTENDED_Z);
      singleGaitCtrl(3, 1, Group_B, -directionAngle, WALK_EXTENDED_X, WALK_EXTENDED_Z);
    }
    else if(turnCmd == -1){
      singleGaitCtrl(1, 1.5, Group_A, 90,  WALK_EXTENDED_X, WALK_EXTENDED_Z);
      singleGaitCtrl(4, 1.5, Group_A, 90, -WALK_EXTENDED_X, WALK_EXTENDED_Z);
  
      singleGaitCtrl(2, 1.5, Group_B, -90, -WALK_EXTENDED_X, WALK_EXTENDED_Z);
      singleGaitCtrl(3, 1.5, Group_B, -90, WALK_EXTENDED_X, WALK_EXTENDED_Z);
    }
    else if(turnCmd == 1){
      singleGaitCtrl(1, 1.5, Group_A, -90,  WALK_EXTENDED_X, WALK_EXTENDED_Z);
      singleGaitCtrl(4, 1.5, Group_A, -90, -WALK_EXTENDED_X, WALK_EXTENDED_Z);
  
      singleGaitCtrl(2, 1.5, Group_B, 90, -WALK_EXTENDED_X, WALK_EXTENDED_Z);
      singleGaitCtrl(3, 1.5, Group_B, 90, WALK_EXTENDED_X, WALK_EXTENDED_Z);
    }
}

// Triangular gait generater.
void BodyCtrl::triangularGait(float GlobalInput, float directionAngle, int turnCmd){
    float StepA;
    float StepB;
    float StepC;
    float StepD;
  
    float aInput = 0;
    float bInput = 0;
    float adProp;
  
    StepB = GlobalInput;
    StepC = GlobalInput + 0.25;
    StepD = GlobalInput + 0.5;
    StepA = GlobalInput + 0.75;
  
    if(StepA>1){StepA--;}
    if(StepB>1){StepB--;}
    if(StepC>1){StepC--;}
    if(StepD>1){StepD--;}
  
    if(GlobalInput <= 0.25){
      adProp = GlobalInput;
      aInput =  WALK_MASS_ADJUST - (adProp/0.125)*WALK_MASS_ADJUST;
      bInput = -WALK_MASS_ADJUST;
    }
    else if(GlobalInput > 0.25 && GlobalInput <= 0.5){
      adProp = GlobalInput-0.25;
      aInput = -WALK_MASS_ADJUST + (adProp/0.125)*WALK_MASS_ADJUST;
      bInput = -WALK_MASS_ADJUST + (adProp/0.125)*WALK_MASS_ADJUST;
    }
    else if(GlobalInput > 0.5 && GlobalInput <= 0.75){
      adProp = GlobalInput-0.5;
      aInput =  WALK_MASS_ADJUST - (adProp/0.125)*WALK_MASS_ADJUST;
      bInput =  WALK_MASS_ADJUST;
    }
    else if(GlobalInput > 0.75 && GlobalInput <= 1){
      adProp = GlobalInput-0.75;
      aInput = -WALK_MASS_ADJUST + (adProp/0.125)*WALK_MASS_ADJUST;
      bInput =  WALK_MASS_ADJUST - (adProp/0.125)*WALK_MASS_ADJUST;
    }
  
    if(!turnCmd){
      singleGaitCtrl(1, 1, StepA,  directionAngle,  WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
      singleGaitCtrl(4, 1, StepD, -directionAngle, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);
  
      singleGaitCtrl(2, 1, StepB,  directionAngle, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
      singleGaitCtrl(3, 1, StepC, -directionAngle,  WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);
    }
    else if(turnCmd == -1){
      singleGaitCtrl(1, 1.5, StepA,  90,  WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
      singleGaitCtrl(4, 1.5, StepD,  90, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);
  
      singleGaitCtrl(2, 1.5, StepB, -90, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
      singleGaitCtrl(3, 1.5, StepC, -90,  WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);
    }
    else if(turnCmd == 1){
      singleGaitCtrl(1, 1.5, StepA, -90,  WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
      singleGaitCtrl(4, 1.5, StepD, -90, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);
  
      singleGaitCtrl(2, 1.5, StepB,  90, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
      singleGaitCtrl(3, 1.5, StepC,  90,  WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);
    }
}

// Gait Select
void BodyCtrl::gaitTypeCtrl(float GlobalStepInput, float directionCmd, int turnCmd){
    if(GAIT_TYPE == 0){
      simpleGait(GlobalStepInput, directionCmd, turnCmd);
    }
    else if(GAIT_TYPE == 1){
      triangularGait(GlobalStepInput, directionCmd, turnCmd);
    }
}

// Stand and adjust mass center.
//    |
//    a
//    |
// -b-M  a,b > 0
void BodyCtrl::standMassCenter(float aInput, float bInput){
    singleLegCtrl(1, ( WALK_EXTENDED_X - aInput), STAND_HEIGHT, ( WALK_EXTENDED_Z - bInput));
    singleLegCtrl(2, (-WALK_EXTENDED_X - aInput), STAND_HEIGHT, ( WALK_EXTENDED_Z - bInput));
  
    singleLegCtrl(3, ( WALK_EXTENDED_X - aInput), STAND_HEIGHT, ( WALK_EXTENDED_Z + bInput));
    singleLegCtrl(4, (-WALK_EXTENDED_X - aInput), STAND_HEIGHT, ( WALK_EXTENDED_Z + bInput));
}

void BodyCtrl::pitchYawRollHeightCtrl(float pitchInput, float yawInput, float rollInput, float heightInput){
    legPosBuffer[1]  = STAND_HEIGHT + pitchInput + rollInput + heightInput;
    legPosBuffer[4]  = STAND_HEIGHT - pitchInput + rollInput + heightInput;
    legPosBuffer[7]  = STAND_HEIGHT + pitchInput - rollInput + heightInput;
    legPosBuffer[10] = STAND_HEIGHT - pitchInput - rollInput + heightInput;
  
    if(legPosBuffer[1] > WALK_HEIGHT_MAX){legPosBuffer[1] = WALK_HEIGHT_MAX;}
    if(legPosBuffer[4] > WALK_HEIGHT_MAX){legPosBuffer[4] = WALK_HEIGHT_MAX;}
    if(legPosBuffer[7] > WALK_HEIGHT_MAX){legPosBuffer[7] = WALK_HEIGHT_MAX;}
    if(legPosBuffer[10] > WALK_HEIGHT_MAX){legPosBuffer[10] = WALK_HEIGHT_MAX;}
    
    if(legPosBuffer[1] < WALK_HEIGHT_MIN){legPosBuffer[1] = WALK_HEIGHT_MIN;}
    if(legPosBuffer[4] < WALK_HEIGHT_MIN){legPosBuffer[4] = WALK_HEIGHT_MIN;}
    if(legPosBuffer[7] < WALK_HEIGHT_MIN){legPosBuffer[7] = WALK_HEIGHT_MIN;}
    if(legPosBuffer[10] < WALK_HEIGHT_MIN){legPosBuffer[10] = WALK_HEIGHT_MIN;}
  
    legPosBuffer[2]  = WALK_EXTENDED_Z + yawInput - rollInput;
    legPosBuffer[5]  = WALK_EXTENDED_Z - yawInput - rollInput;
    legPosBuffer[8]  = WALK_EXTENDED_Z - yawInput + rollInput;
    legPosBuffer[11] = WALK_EXTENDED_Z + yawInput + rollInput;
  
    if(legPosBuffer[2] > WALK_EXTENDED_Z + WALK_SIDE_MAX){legPosBuffer[2] = WALK_EXTENDED_Z + WALK_SIDE_MAX;}
    if(legPosBuffer[5] > WALK_EXTENDED_Z + WALK_SIDE_MAX){legPosBuffer[5] = WALK_EXTENDED_Z + WALK_SIDE_MAX;}
    if(legPosBuffer[8] > WALK_EXTENDED_Z + WALK_SIDE_MAX){legPosBuffer[8] = WALK_EXTENDED_Z + WALK_SIDE_MAX;}
    if(legPosBuffer[11] > WALK_EXTENDED_Z + WALK_SIDE_MAX){legPosBuffer[11] = WALK_EXTENDED_Z + WALK_SIDE_MAX;}
  
    if(legPosBuffer[2] < WALK_EXTENDED_Z - WALK_SIDE_MAX){legPosBuffer[2] = WALK_EXTENDED_Z - WALK_SIDE_MAX;}
    if(legPosBuffer[5] < WALK_EXTENDED_Z - WALK_SIDE_MAX){legPosBuffer[5] = WALK_EXTENDED_Z - WALK_SIDE_MAX;}
    if(legPosBuffer[8] < WALK_EXTENDED_Z - WALK_SIDE_MAX){legPosBuffer[8] = WALK_EXTENDED_Z - WALK_SIDE_MAX;}
    if(legPosBuffer[11] < WALK_EXTENDED_Z - WALK_SIDE_MAX){legPosBuffer[11] = WALK_EXTENDED_Z - WALK_SIDE_MAX;}
  
    singleLegCtrl(1,  WALK_EXTENDED_X, legPosBuffer[1] , legPosBuffer[2]);
    singleLegCtrl(2, -WALK_EXTENDED_X, legPosBuffer[4] , legPosBuffer[5]);
    singleLegCtrl(3,  WALK_EXTENDED_X, legPosBuffer[7] , legPosBuffer[8]);
    singleLegCtrl(4, -WALK_EXTENDED_X, legPosBuffer[10], legPosBuffer[11]);
}

void BodyCtrl::pitchYawRoll(float pitchInput, float yawInput, float rollInput){
  legPosBuffer[1]  = STAND_HEIGHT + pitchInput + rollInput;
  legPosBuffer[4]  = STAND_HEIGHT - pitchInput + rollInput;
  legPosBuffer[7]  = STAND_HEIGHT + pitchInput - rollInput;
  legPosBuffer[10] = STAND_HEIGHT - pitchInput - rollInput;

  if(legPosBuffer[1] > WALK_HEIGHT_MAX){legPosBuffer[1] = WALK_HEIGHT_MAX;}
  if(legPosBuffer[4] > WALK_HEIGHT_MAX){legPosBuffer[4] = WALK_HEIGHT_MAX;}
  if(legPosBuffer[7] > WALK_HEIGHT_MAX){legPosBuffer[7] = WALK_HEIGHT_MAX;}
  if(legPosBuffer[10] > WALK_HEIGHT_MAX){legPosBuffer[10] = WALK_HEIGHT_MAX;}
  
  if(legPosBuffer[1] < WALK_HEIGHT_MIN){legPosBuffer[1] = WALK_HEIGHT_MIN;}
  if(legPosBuffer[4] < WALK_HEIGHT_MIN){legPosBuffer[4] = WALK_HEIGHT_MIN;}
  if(legPosBuffer[7] < WALK_HEIGHT_MIN){legPosBuffer[7] = WALK_HEIGHT_MIN;}
  if(legPosBuffer[10] < WALK_HEIGHT_MIN){legPosBuffer[10] = WALK_HEIGHT_MIN;}

  legPosBuffer[2]  = WALK_EXTENDED_Z + yawInput - rollInput;
  legPosBuffer[5]  = WALK_EXTENDED_Z - yawInput - rollInput;
  legPosBuffer[8]  = WALK_EXTENDED_Z - yawInput + rollInput;
  legPosBuffer[11] = WALK_EXTENDED_Z + yawInput + rollInput;

  if(legPosBuffer[2] > WALK_EXTENDED_Z + WALK_SIDE_MAX){legPosBuffer[2] = WALK_EXTENDED_Z + WALK_SIDE_MAX;}
  if(legPosBuffer[5] > WALK_EXTENDED_Z + WALK_SIDE_MAX){legPosBuffer[5] = WALK_EXTENDED_Z + WALK_SIDE_MAX;}
  if(legPosBuffer[8] > WALK_EXTENDED_Z + WALK_SIDE_MAX){legPosBuffer[8] = WALK_EXTENDED_Z + WALK_SIDE_MAX;}
  if(legPosBuffer[11] > WALK_EXTENDED_Z + WALK_SIDE_MAX){legPosBuffer[11] = WALK_EXTENDED_Z + WALK_SIDE_MAX;}

  if(legPosBuffer[2] < WALK_EXTENDED_Z - WALK_SIDE_MAX){legPosBuffer[2] = WALK_EXTENDED_Z - WALK_SIDE_MAX;}
  if(legPosBuffer[5] < WALK_EXTENDED_Z - WALK_SIDE_MAX){legPosBuffer[5] = WALK_EXTENDED_Z - WALK_SIDE_MAX;}
  if(legPosBuffer[8] < WALK_EXTENDED_Z - WALK_SIDE_MAX){legPosBuffer[8] = WALK_EXTENDED_Z - WALK_SIDE_MAX;}
  if(legPosBuffer[11] < WALK_EXTENDED_Z - WALK_SIDE_MAX){legPosBuffer[11] = WALK_EXTENDED_Z - WALK_SIDE_MAX;}

  singleLegCtrl(1,  WALK_EXTENDED_X, legPosBuffer[1] , legPosBuffer[2]);
  singleLegCtrl(2, -WALK_EXTENDED_X, legPosBuffer[4] , legPosBuffer[5]);
  singleLegCtrl(3,  WALK_EXTENDED_X, legPosBuffer[7] , legPosBuffer[8]);
  singleLegCtrl(4, -WALK_EXTENDED_X, legPosBuffer[10], legPosBuffer[11]);
}

// BALANCE
void BodyCtrl::balancing(float ACC_Y, float ACC_X){
    BALANCE_PITCHU_BUFFER += ACC_X * BALANCE_P;
    BALANCE_ROLL_BUFFER -= ACC_Y * BALANCE_P;
  
    if(BALANCE_PITCHU_BUFFER > 21){BALANCE_PITCHU_BUFFER = 21;}
    if(BALANCE_PITCHU_BUFFER < -21){BALANCE_PITCHU_BUFFER = -21;}
  
    if(BALANCE_ROLL_BUFFER > 21){BALANCE_ROLL_BUFFER = 21;}
    if(BALANCE_ROLL_BUFFER < -21){BALANCE_ROLL_BUFFER = -21;}
    
    // pitchYawRollHeightCtrl(BALANCE_PITCHU_BUFFER, 0, BALANCE_ROLL_BUFFER, STAND_HEIGHT);
    pitchYawRoll(BALANCE_PITCHU_BUFFER, 0, BALANCE_ROLL_BUFFER);

    allJointAngle(GoalAngle);
    delay(1);
}

// mass center adjust test.
void BodyCtrl::massCenerAdjustTestLoop(){
    for(float i = 0; i<=20; i = i+0.5){
      standMassCenter(-20+i, i);
      allJointAngle(GoalAngle);
      Serial.println("1");
      delay(1);
    }
  
    for(float i = 0; i<=20; i = i+0.5){
      standMassCenter(i, 20-i);
      allJointAngle(GoalAngle);
      Serial.println("2");
      delay(1);
    }
  
    for(float i = 0; i<=20; i = i+0.5){
      standMassCenter(20-i, -i);
      allJointAngle(GoalAngle);
      Serial.println("3");
      delay(1);
    }
  
    for(float i = 0; i<=20; i = i+0.5){
      standMassCenter(-i, -20+i);
      allJointAngle(GoalAngle);
      Serial.println("4");
      delay(1);
    }
}

void BodyCtrl::inputCmd(int fb, int lr){
    moveFB = fb;
    moveLR = lr;
}

void BodyCtrl::setInterpolationParams(int delayInput, float iterateInput){
  STEP_DELAY   = delayInput;
  STEP_ITERATE = iterateInput;
}

void BodyCtrl::setGaitParams(double maxHeight, double minHeight, double height, double lift, 
          double range, double acc, double extendedX, double extendedZ, double sideMax, double massAdjust){
  WALK_HEIGHT_MAX  = maxHeight;
  WALK_HEIGHT_MIN  = minHeight;
  WALK_HEIGHT      = height;
  WALK_LIFT        = lift; // WALK_HEIGHT + WALK_LIFT <= WALK_HEIGHT_MAX.
  WALK_RANGE       = range;
  WALK_ACC         = acc;
  WALK_EXTENDED_X  = extendedX;
  WALK_EXTENDED_Z  = extendedZ;
  WALK_SIDE_MAX    = sideMax;
  WALK_MASS_ADJUST = massAdjust;
}

void BodyCtrl::robotCtrl(){
  // move ctrl.
  if(!debugMode && !funcMode){
    if(moveFB == 0 && moveLR == 0 && STAND_STILL == 0){
      standMassCenter(0, 0);
      allJointAngle(GoalAngle);
      STAND_STILL = 1;
      GLOBAL_STEP = 0;
      delay(STEP_DELAY);
    }
    else if(moveFB == 0 && moveLR == 0 && STAND_STILL == 1){
      allJointAngle(GoalAngle);
      delay(STEP_DELAY);
    }
    else{
      STAND_STILL = 0;
      gestureUD = 0;
      gestureLR = 0;
      if(GLOBAL_STEP > 1){GLOBAL_STEP = 0;}
      if(moveFB == 1 && moveLR == 0){gaitTypeCtrl(GLOBAL_STEP, 0, 0);}
      else if(moveFB == -1 && moveLR == 0){gaitTypeCtrl(GLOBAL_STEP, 180, 0);}
      else if(moveFB == 1 && moveLR == -1){gaitTypeCtrl(GLOBAL_STEP, 30, 0);}
      else if(moveFB == 1 && moveLR == 1){gaitTypeCtrl(GLOBAL_STEP, -30, 0);}
      else if(moveFB == -1 && moveLR == 1){gaitTypeCtrl(GLOBAL_STEP, -120, 0);}
      else if(moveFB == -1 && moveLR == -1){gaitTypeCtrl(GLOBAL_STEP, 120, 0);}
      else if(moveFB == 0 && moveLR == -1){gaitTypeCtrl(GLOBAL_STEP, 0, -1);}
      else if(moveFB == 0 && moveLR == 1){gaitTypeCtrl(GLOBAL_STEP, 0, 1);}
      allJointAngle(GoalAngle);
      GLOBAL_STEP += STEP_ITERATE;
      delay(STEP_DELAY);
    }
  }
}


// key frames ctrl.
// 0 <= rateInput <= 1
// use it fuction to ctrl a number change from numStart to numEnd.
float BodyCtrl::linearCtrl(float numStart, float numEnd, float rateInput){
  float numOut;
  numOut = (numEnd - numStart)*rateInput + numStart;
  return numOut;
}


// linearCtrl() function is a simple example, which shows how besselCtrl works.
float BodyCtrl::besselCtrl(float numStart, float numEnd, float rateInput){
  float numOut;
  numOut = (numEnd - numStart)*((cos(rateInput*M_PI-M_PI)+1)/2) + numStart;
  return numOut;
}


// Functions.
void BodyCtrl::functionStayLow(){
  for(float i = 0; i<=1; i+=0.02){
    standUp(besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i));
    allJointAngle(GoalAngle);
    delay(6);
  }
  delay(300);
  for(float i = 0; i<=1; i+=0.02){
    standUp(besselCtrl(WALK_HEIGHT_MIN, WALK_HEIGHT_MAX, i));
    allJointAngle(GoalAngle);
    delay(6);
  }
  for(float i = 0; i<=1; i+=0.02){
    standUp(besselCtrl(WALK_HEIGHT_MAX, WALK_HEIGHT, i));
    allJointAngle(GoalAngle);
    delay(6);
  }
}


void BodyCtrl::functionHandshake(){
  for(float i = 0; i<=1; i+=0.01){
    singleLegCtrl(1,  besselCtrl(WALK_EXTENDED_X, 0, i), besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MAX, i), besselCtrl(WALK_EXTENDED_Z, -15, i));
    singleLegCtrl(3,  besselCtrl(WALK_EXTENDED_X, 0, i), besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MAX, i), WALK_EXTENDED_Z);

    singleLegCtrl(2,  -WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN-10, i), besselCtrl(WALK_EXTENDED_Z, 2*WALK_EXTENDED_Z, i));
    singleLegCtrl(4,  -WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN-10, i), besselCtrl(WALK_EXTENDED_Z, 2*WALK_EXTENDED_Z, i));

    allJointAngle(GoalAngle);
    delay(8);
  }


  for(float i = 0; i<=1; i+=0.01){
    singleLegCtrl(3,  besselCtrl(0, WALK_RANGE/2+WALK_EXTENDED_X, i), besselCtrl(WALK_HEIGHT_MAX, WALK_HEIGHT_MIN, i), besselCtrl(WALK_EXTENDED_Z, 0, i));

    allJointAngle(GoalAngle);
    delay(8);
  }

  for(int shakeTimes = 0; shakeTimes < 3; shakeTimes++){
    for(float i = 0; i<=1; i+=0.02){
      singleLegCtrl(3,  WALK_RANGE/2+WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN, WALK_HEIGHT_MIN+30, i), 0);

      allJointAngle(GoalAngle);
      delay(9);
    }
    for(float i = 0; i<=1; i+=0.02){
      singleLegCtrl(3,  WALK_RANGE/2+WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN+30, WALK_HEIGHT_MIN, i), 0);

      allJointAngle(GoalAngle);
      delay(9);
    }
  }

  for(float i = 0; i<=1; i+=0.01){
    singleLegCtrl(1,  besselCtrl(0, WALK_EXTENDED_X, i), besselCtrl(WALK_HEIGHT_MAX, WALK_HEIGHT, i), besselCtrl(-15, WALK_EXTENDED_Z, i));
    singleLegCtrl(3,  besselCtrl(WALK_RANGE/2+WALK_EXTENDED_X, WALK_EXTENDED_X, i), besselCtrl(WALK_HEIGHT_MIN, WALK_HEIGHT, i), besselCtrl(0, WALK_EXTENDED_Z, i));

    singleLegCtrl(2,  -WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN-10, WALK_HEIGHT, i), besselCtrl(2*WALK_EXTENDED_Z, WALK_EXTENDED_Z, i));
    singleLegCtrl(4,  -WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN-10, WALK_HEIGHT, i), besselCtrl(2*WALK_EXTENDED_Z, WALK_EXTENDED_Z, i));

    allJointAngle(GoalAngle);
    delay(8);
  }
}


void BodyCtrl::functionJump(){
  for(float i = 0; i<=1; i+=0.02){
    singleLegCtrl(1, WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i), WALK_EXTENDED_Z);
    singleLegCtrl(2,-WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i), WALK_EXTENDED_Z);
    singleLegCtrl(3, WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i), WALK_EXTENDED_Z);
    singleLegCtrl(4,-WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i), WALK_EXTENDED_Z);
    allJointAngle(GoalAngle);
    delay(12);
  }
  delay(300);
  singleLegCtrl(1, WALK_EXTENDED_X, WALK_HEIGHT_MAX, WALK_EXTENDED_Z);
  singleLegCtrl(2,-WALK_EXTENDED_X, WALK_HEIGHT_MAX, WALK_EXTENDED_Z);
  singleLegCtrl(3, WALK_EXTENDED_X, WALK_HEIGHT_MAX, WALK_EXTENDED_Z);
  singleLegCtrl(4,-WALK_EXTENDED_X, WALK_HEIGHT_MAX, WALK_EXTENDED_Z);
  allJointAngle(GoalAngle);
  delay(120);

  for(float i = 0; i<=1; i+=0.02){
    singleLegCtrl(1, WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN, WALK_HEIGHT, i), WALK_EXTENDED_Z);
    singleLegCtrl(2,-WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN, WALK_HEIGHT, i), WALK_EXTENDED_Z);
    singleLegCtrl(3, WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN, WALK_HEIGHT, i), WALK_EXTENDED_Z);
    singleLegCtrl(4,-WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN, WALK_HEIGHT, i), WALK_EXTENDED_Z);
    allJointAngle(GoalAngle);
    delay(12);
  }
}

void BodyCtrl::ugvCtrl(float leftSpd, float rightSpd){
  if (leftSpd == rightSpd) {
    if (leftSpd == 0) {
      inputCmd(0, 0);
    } else if (leftSpd > 0) {
      inputCmd(1, 0);
    } else if (leftSpd < 0) {
      inputCmd(-1, 0);
    }
    return;
  } else if (leftSpd == -rightSpd) {
    if (leftSpd > 0) {
      inputCmd(0, 1);
    } else if (leftSpd < 0) {
      inputCmd(0, -1);
    }
    return;
  } else {
    if (leftSpd > 0 && rightSpd > 0 && abs(leftSpd) > abs(rightSpd)) {
      inputCmd(1, 1);
    } else if (leftSpd < 0 && rightSpd < 0 && abs(leftSpd) < abs(rightSpd)) {
      inputCmd(-1, -1);
    } else if (leftSpd > 0 && rightSpd > 0 && abs(leftSpd) < abs(rightSpd)) {
      inputCmd(1, -1);
    } else if (leftSpd < 0 && rightSpd < 0 && abs(leftSpd) > abs(rightSpd)) {
      inputCmd(-1, 1);
    }
  }
}

void BodyCtrl::ptCtrl(float axisX, float axisY){
  double pit = mapDouble(axisY, -30, 30, -17.5, 17.5);
  double yaw = mapDouble(axisX, -180, 180, -30, 30);
  pitchYawRoll(pit, yaw, 0);
}