#include <math.h>
#define ANG2DEG 0.017453292

// Instantiate a servo control object.
SMS_STS st;

// place holder.
void serialCtrl();

// Used to store the feedback information from the servo.
struct ServoFeedback {
  bool status;
  int pos;
  int speed;
  int load;
  float voltage;
  float current;
  float temper;
  byte mode;
};

ServoFeedback servoFeedback[7];
// [0] BASE_SERVO_ID
// [1] SHOULDER_DRIVING_SERVO_ID
// [2] SHOULDER_DRIVEN_SERVO_ID
// [3] ELBOW_SERVO_ID
// [4] WRIST_SERVO_ID
// [5] ROLL_SERVO_ID
// [6] GRIPPER_SERVO_ID



// input the angle in radians, and it returns the number of servo steps.
double calculatePosByRad(double radInput) {
  return round((radInput / (2 * M_PI)) * ARM_SERVO_POS_RANGE);
}

// input the number of servo steps and the joint name
// return the joint angle in radians.
double calculateRadByFeedback(int inputSteps, int jointName) {
  double getRad;
  switch(jointName){
  case BASE_JOINT:
    getRad = -(inputSteps * 2 * M_PI / ARM_SERVO_POS_RANGE) + M_PI;
    break;
  case SHOULDER_JOINT:
    getRad = (inputSteps * 2 * M_PI / ARM_SERVO_POS_RANGE) - M_PI;
    break;
  case ELBOW_JOINT:
    getRad = (inputSteps * 2 * M_PI / ARM_SERVO_POS_RANGE) - (M_PI / 2);
    break;
  case WRIST_JOINT:
    getRad = (inputSteps * 2 * M_PI / ARM_SERVO_POS_RANGE) - M_PI;
    break;
  case ROLL_JOINT:
    getRad = -(inputSteps * 2 * M_PI / ARM_SERVO_POS_RANGE) + M_PI;
    break;
  case EOAT_JOINT:
    getRad = inputSteps * 2 * M_PI / ARM_SERVO_POS_RANGE;
    break;
  }
  return getRad;
}


// input the ID of the servo,
// and get the information saved in servoFeedback[5].
// returnType: false - return everything.
//              true - return only when failed.
bool getFeedback(byte servoID, bool returnType) {
  if(st.FeedBack(servoID)!=-1) {
    servoFeedback[servoID - 11].status = true;
  	servoFeedback[servoID - 11].pos = st.ReadPos(-1);
    servoFeedback[servoID - 11].speed = st.ReadSpeed(-1);
    servoFeedback[servoID - 11].load = st.ReadLoad(-1);
    servoFeedback[servoID - 11].voltage = st.ReadVoltage(-1);
    servoFeedback[servoID - 11].current = st.ReadCurrent(-1);
    servoFeedback[servoID - 11].temper = st.ReadTemper(-1);
    servoFeedback[servoID - 11].mode = st.ReadMode(servoID);
    if(!returnType){
      if(InfoPrint == 1){Serial.print("Servo ID:");Serial.print(servoID);
                    Serial.print(" status: checked. pos:");
                    Serial.println(servoFeedback[servoID - 11].pos);}
    }
    else{
      return true;
    }
    return true;
  } else{
    servoFeedback[servoID - 11].status = false;
    if(InfoPrint == 1){Serial.print("Servo ID:");Serial.print(servoID);
                  Serial.println(" status: failed.");}
  	return false;
  }
}


// input the old servo ID and the new ID you want it to change to.
bool changeID(byte oldID, byte newID) {
  if(!getFeedback(oldID, true)) {
    if(InfoPrint == 1) {Serial.print("change: ");Serial.print(oldID);Serial.println("failed");}
    return false;
  }
  else {
    st.unLockEprom(oldID);
    st.writeByte(oldID, SMS_STS_ID, newID);
    st.LockEprom(newID);

    if(InfoPrint == 1) {Serial.print("change: ");Serial.print(oldID);Serial.println("succeed");}
    return true;
  }
}


// ctrl the torque lock of a servo.
// input the servo ID and command: 1-on : produce torque.
//                                 0-off: release torque.
void servoTorqueCtrl(byte servoID, u8 enableCMD){
  st.EnableTorque(servoID, enableCMD);
}


// set the current position as the middle position of the servo.
// input the ID of the servo that you wannna set middle position. 
void setMiddlePos(byte InputID){
  st.CalibrationOfs(InputID);
}


// to release all servos' torque for 10s.
void emergencyStopProcessing() {
  st.EnableTorque(254, 0);
  delay(10000);
  st.EnableTorque(254, 1);
}


// position check.
// it will wait for the servo to move to the goal position.
void waitMove2Goal(byte InputID, s16 goalPosition, s16 offSet){
  while(servoFeedback[InputID - 11].pos < goalPosition - offSet || 
        servoFeedback[InputID - 11].pos > goalPosition + offSet){
    if (!servoFeedback[InputID - 11].status) {
      servoTorqueCtrl(254, 0);
      break;
    }
    getFeedback(InputID, true);
    delay(10);
  }
}


// initialize bus servo libraris and uart2ttl.
void RoArmM3_servoInit(){
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  while(!Serial1) {}
  if(InfoPrint == 1){Serial.println("ServoCtrl init succeed.");}
}


// check the status of every servo,
// if all status are ok, set the RoArmM3_initCheckSucceed as 1.
// 0: used to init check, print everything.
// 1: used to check while working, print when failed.
void RoArmM3_initCheck(bool returnType) {
  RoArmM3_initCheckSucceed = false;
  RoArmM3_initCheckSucceed = getFeedback(BASE_SERVO_ID, true) &&
                             getFeedback(SHOULDER_DRIVING_SERVO_ID, true) &&
                             getFeedback(SHOULDER_DRIVEN_SERVO_ID, true) &&
                             getFeedback(ELBOW_SERVO_ID, true);
  if(!returnType){
    if(InfoPrint == 1 || RoArmM3_initCheckSucceed){Serial.println("All bus servos status checked.");}
    else if(InfoPrint == 1 || !RoArmM3_initCheckSucceed){Serial.println("Bus servos status check: failed.");}
  }
  else if(returnType && RoArmM3_initCheckSucceed){}
  else if(returnType && !RoArmM3_initCheckSucceed){
    if(InfoPrint == 1){Serial.println("Check failed.");}
  }
}


// set all servos PID as the RoArm-M2 settings.
bool setServosPID(byte InputID, byte InputP) {
  if(!getFeedback(InputID, true)){return false;}
  st.unLockEprom(InputID);
  st.writeByte(InputID, ST_PID_P_ADDR, InputP); 
  st.LockEprom(InputID);
  return true;
}


// move every joint to its init position.
// it moves only when RoArmM3_initCheckSucceed is 1.
void RoArmM3_moveInit() {
  if(!RoArmM3_initCheckSucceed){
    if(InfoPrint == 1){Serial.println("Init failed, skip moveInit.");}
    return;
  }
  else if(InfoPrint == 1){Serial.println("Stop moving to initPos.");}

  // move BASE_SERVO to middle position.
  if(InfoPrint == 1){Serial.println("Moving BASE_JOINT to initPos.");}
  st.WritePosEx(BASE_SERVO_ID, ARM_SERVO_MIDDLE_POS, ARM_SERVO_INIT_SPEED, ARM_SERVO_INIT_ACC);

  // release SHOULDER_DRIVEN_SERVO torque.
  if(InfoPrint == 1){Serial.println("Unlock the torque of SHOULDER_DRIVEN_SERVO.");}
  servoTorqueCtrl(SHOULDER_DRIVEN_SERVO_ID, 0);

  // move SHOULDER_DRIVING_SERVO to middle position.
  if(InfoPrint == 1){Serial.println("Moving SHOULDER_JOINT to initPos.");}
  st.WritePosEx(SHOULDER_DRIVING_SERVO_ID, ARM_SERVO_MIDDLE_POS, ARM_SERVO_INIT_SPEED, ARM_SERVO_INIT_ACC);

  // check SHOULDER_DRIVEING_SERVO position.
  if(InfoPrint == 1){Serial.println("...");}
  waitMove2Goal(SHOULDER_DRIVING_SERVO_ID, ARM_SERVO_MIDDLE_POS, 120);

  // wait for the jitter to go away.
  delay(1200);

  // set the position as the middle of the SHOULDER_DRIVEN_SERVO.
  if(InfoPrint == 1){Serial.println("Set this pos as the middle pos for SHOULDER_DRIVEN_SERVO.");}
  setMiddlePos(SHOULDER_DRIVEN_SERVO_ID);

  // SHOULDER_DRIVEN_SERVO starts producing torque.
  if(InfoPrint == 1){Serial.println("SHOULDER_DRIVEN_SERVO starts producing torque.");}
  servoTorqueCtrl(SHOULDER_DRIVEN_SERVO_ID, 1);
  delay(10);

  // move ELBOW_SERVO to middle position.
  if(InfoPrint == 1){Serial.println("Moving ELBOW_SERVO to middle position.");}
  st.WritePosEx(ELBOW_SERVO_ID, ARM_SERVO_MIDDLE_POS, ARM_SERVO_INIT_SPEED, ARM_SERVO_INIT_ACC);
  waitMove2Goal(ELBOW_SERVO_ID, ARM_SERVO_MIDDLE_POS, 80);
  st.WritePosEx(WRIST_SERVO_ID, ARM_SERVO_MIDDLE_POS, ARM_SERVO_INIT_SPEED, ARM_SERVO_INIT_ACC);
  st.WritePosEx(ROLL_SERVO_ID, ARM_SERVO_MIDDLE_POS, ARM_SERVO_INIT_SPEED, ARM_SERVO_INIT_ACC);

  if(InfoPrint == 1){Serial.println("Moving GRIPPER_SERVO to middle position.");}
  st.WritePosEx(GRIPPER_SERVO_ID, ARM_SERVO_MIDDLE_POS, ARM_SERVO_INIT_SPEED, ARM_SERVO_INIT_ACC);

  delay(1000);
}


// // // single joint ctrl for simple uses, base on radInput // // //

// use this function to compute the servo position to ctrl base joint.
// returnType 0: only returns the base joint servo position and save it to goalPos[0],
//               servo will NOT move.
//            1: returns the base joint servo position and save it to goalPos[0],
//               servo moves.
// input the angle in radius(double), the speedInput(u16) is servo steps/second,
// the accInput(u8) is the acceleration of the servo movement.
// radInput increase, move to left.
int RoArmM3_baseJointCtrlRad(byte returnType, double radInput, u16 speedInput, u8 accInput) {
  radInput = -constrain(radInput, -M_PI, M_PI);
  s16 computePos = calculatePosByRad(radInput) + ARM_SERVO_MIDDLE_POS;
  goalPos[0] = computePos;

  if(returnType){
    st.WritePosEx(BASE_SERVO_ID, goalPos[0], speedInput, accInput);
  }
  return goalPos[0];
}


// use this function to compute the servo position to ctrl shoudlder joint.
// returnType 0: only returns the shoulder joint servo position and save it to goalPos[1] and goalPos[2],
//               servo will NOT move.
//            1: returns the shoulder joint servo position and save it to goalPos[1] and goalPos[2],
//               servo moves.
// input the angle in radius(double), the speedInput(u16) is servo steps/second,
// the accInput(u8) is the acceleration of the servo movement.
// radInput increase, it leans forward.
int RoArmM3_shoulderJointCtrlRad(byte returnType, double radInput, u16 speedInput, u8 accInput) {
  radInput = constrain(radInput, -M_PI/2, M_PI/2);
  s16 computePos = calculatePosByRad(radInput);
  goalPos[1] = ARM_SERVO_MIDDLE_POS + computePos;
  goalPos[2] = ARM_SERVO_MIDDLE_POS - computePos;
  
  if(returnType == 1){
    st.WritePosEx(SHOULDER_DRIVING_SERVO_ID, goalPos[1], speedInput, accInput);
    st.WritePosEx(SHOULDER_DRIVEN_SERVO_ID, goalPos[2], speedInput, accInput);
  }
  else if(returnType == SHOULDER_DRIVING_SERVO_ID){
    return goalPos[1];
  }
  else if(returnType == SHOULDER_DRIVEN_SERVO_ID){
    return goalPos[2];
  }
}


// use this function to compute the servo position to ctrl elbow joint.
// returnType 0: only returns the elbow joint servo position and save it to goalPos[3],
//               servo will NOT move.
//            1: returns the elbow joint servo position and save it to goalPos[3],
//               servo moves.
// input the angle in radius(double), the speedInput(u16) is servo steps/second,
// the accInput(u8) is the acceleration of the servo movement.
// angleInput increase, it moves down.
int RoArmM3_elbowJointCtrlRad(byte returnType, double radInput, u16 speedInput, u8 accInput) {
  s16 computePos = calculatePosByRad(radInput) + 1024;
  goalPos[3] = constrain(computePos, 512, 3071);

  if(returnType){
    st.WritePosEx(ELBOW_SERVO_ID, goalPos[3], speedInput, accInput);
  }
  return goalPos[3];
}


int RoArmM3_wristJointCtrlRad(byte returnType, double radInput, u16 speedInput, u8 accInput) {
  radInput = constrain(radInput, -M_PI/2, M_PI/2);
  s16 computePos = calculatePosByRad(radInput);
  goalPos[4] = ARM_SERVO_MIDDLE_POS + computePos;
  
  if(returnType){
    st.WritePosEx(WRIST_SERVO_ID, goalPos[4], speedInput, accInput);
  }
  return goalPos[4];
}


int RoArmM3_rollJointCtrlRad(byte returnType, double radInput, u16 speedInput, u8 accInput) {
  radInput = constrain(radInput, -M_PI, M_PI);
  s16 computePos = calculatePosByRad(radInput);
  goalPos[5] = ARM_SERVO_MIDDLE_POS - computePos;
  
  if(returnType){
    st.WritePosEx(ROLL_SERVO_ID, goalPos[5], speedInput, accInput);
  }
  return goalPos[5];
}


// use this function to compute the servo position to ctrl grab/hand joint.
// returnType 0: only returns the hand joint servo position and save it to goalPos[4],
//               servo will NOT move.
//            1: returns the hand joint servo position and save it to goalPos[4],
//               servo moves.
// ctrl type 0: status ctrl. - cmd 0: release 
//                                 1: grab
//           1: position ctrl. - cmd: input angle in radius.
int RoArmM3_handJointCtrlRad(byte returnType, double radInput, u16 speedInput, u8 accInput) {
  s16 computePos = calculatePosByRad(radInput);
  goalPos[6] = constrain(computePos, 700, 3396);

  if (returnType) {
    st.WritePosEx(GRIPPER_SERVO_ID, goalPos[6], speedInput, accInput);
  }
  return goalPos[6];
}


// use this function to ctrl the max torque of base joint.
void RoArmM3_baseTorqueCtrl(int inputTorque) {
  st.unLockEprom(BASE_SERVO_ID);
  st.writeWord(BASE_SERVO_ID, SMS_STS_TORQUE_LIMIT_L, constrain(inputTorque, ST_TORQUE_MIN, ST_TORQUE_MAX));
  st.LockEprom(BASE_SERVO_ID);
}


// use this function to ctrl the max torque of shoulder joint.
void RoArmM3_shoulderTorqueCtrl(int inputTorque) {
  st.unLockEprom(SHOULDER_DRIVING_SERVO_ID);
  st.writeWord(SHOULDER_DRIVING_SERVO_ID, SMS_STS_TORQUE_LIMIT_L, constrain(inputTorque, ST_TORQUE_MIN, ST_TORQUE_MAX));
  st.LockEprom(SHOULDER_DRIVING_SERVO_ID);

  st.unLockEprom(SHOULDER_DRIVEN_SERVO_ID);
  st.writeWord(SHOULDER_DRIVEN_SERVO_ID, SMS_STS_TORQUE_LIMIT_L, constrain(inputTorque, ST_TORQUE_MIN, ST_TORQUE_MAX));
  st.LockEprom(SHOULDER_DRIVEN_SERVO_ID);
}


// use this function to ctrl the max torque of elbow joint.
void RoArmM3_elbowTorqueCtrl(int inputTorque) {
  st.unLockEprom(ELBOW_SERVO_ID);
  st.writeWord(ELBOW_SERVO_ID, SMS_STS_TORQUE_LIMIT_L, constrain(inputTorque, ST_TORQUE_MIN, ST_TORQUE_MAX));
  st.LockEprom(ELBOW_SERVO_ID);
}


void RoArmM3_wristTorqueCtrl(int inputTorque) {
  st.unLockEprom(WRIST_SERVO_ID);
  st.writeWord(WRIST_SERVO_ID, SMS_STS_TORQUE_LIMIT_L, constrain(inputTorque, ST_TORQUE_MIN, ST_TORQUE_MAX));
  st.LockEprom(WRIST_SERVO_ID);
}

void RoArmM3_rollTorqueCtrl(int inputTorque) {
  st.unLockEprom(ROLL_SERVO_ID);
  st.writeWord(ROLL_SERVO_ID, SMS_STS_TORQUE_LIMIT_L, constrain(inputTorque, ST_TORQUE_MIN, ST_TORQUE_MAX));
  st.LockEprom(ROLL_SERVO_ID);
}


// use this function to ctrl the max torque of hand joint.
void RoArmM3_handTorqueCtrl(int inputTorque) {
  st.unLockEprom(GRIPPER_SERVO_ID);
  st.writeWord(GRIPPER_SERVO_ID, SMS_STS_TORQUE_LIMIT_L, constrain(inputTorque, ST_TORQUE_MIN, ST_TORQUE_MAX));
  st.LockEprom(GRIPPER_SERVO_ID);
}


// dynamic external force adaptation.
// mode: 0 - stop: reset every limit torque to 1000.
//       1 - start: set the joint limit torque. 
// b, s, e, h = bassJoint, shoulderJoint, elbowJoint, handJoint
// example:
// starts. input the limit torque of every joint.
// {"T":112,"mode":1,"b":50,"s":50,"e":50,"h":50}
// stop
// {"T":112,"mode":0,"b":1000,"s":1000,"e":1000,"h":1000}
void RoArmM3_dynamicAdaptation(byte inputM, int inputB, int inputS, int inputE, int inputT, int inputR, int inputH) {
  if (inputM == 0) {
    RoArmM3_baseTorqueCtrl(ST_TORQUE_MAX);
    RoArmM3_shoulderTorqueCtrl(ST_TORQUE_MAX);
    RoArmM3_elbowTorqueCtrl(ST_TORQUE_MAX);
    RoArmM3_wristTorqueCtrl(ST_TORQUE_MAX);
    RoArmM3_rollTorqueCtrl(ST_TORQUE_MAX);
    RoArmM3_handTorqueCtrl(ST_TORQUE_MAX);
  } else if (inputM == 1) {
    RoArmM3_baseTorqueCtrl(inputB);
    RoArmM3_shoulderTorqueCtrl(inputS);
    RoArmM3_elbowTorqueCtrl(inputE);
    RoArmM3_wristTorqueCtrl(inputE);
    RoArmM3_rollTorqueCtrl(inputE);
    RoArmM3_handTorqueCtrl(inputH);
  }
}


// this function uses relative radInput to set a new X+ axis.
// dirInput: 
//              0
//              X+
//        -90 - ^ - 90
//              |
//          -180 180 
void setNewAxisX(double angleInput) {
  double radInput = (angleInput / 180) * M_PI;
  RoArmM3_shoulderJointCtrlRad(1, 0, 500, 20);
  waitMove2Goal(SHOULDER_DRIVING_SERVO_ID, goalPos[1], 200);

  RoArmM3_elbowJointCtrlRad(1, 0, 500, 20);
  RoArmM3_wristJointCtrlRad(1, 0, 500, 20);
  RoArmM3_rollJointCtrlRad(1, 0, 500, 20);
  waitMove2Goal(ELBOW_SERVO_ID, goalPos[3], 200);

  RoArmM3_baseJointCtrlRad(1, 0, 500, 20);
  waitMove2Goal(BASE_SERVO_ID, goalPos[0], 20);
  delay(1000);

  RoArmM3_baseJointCtrlRad(1, -radInput, 500, 20);
  waitMove2Goal(BASE_SERVO_ID, goalPos[0], 20);
  delay(1000);

  setMiddlePos(BASE_SERVO_ID);

  delay(5);
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
void simpleLinkageIkRad(double LA, double LB, double aIn, double bIn) {
  double psi, alpha, omega, beta, L2C, LC, lambda, delta;

  if (fabs(bIn) < 1e-6) {
    psi = acos((LA * LA + aIn * aIn - LB * LB) / (2 * LA * aIn)) + t2rad;
    alpha = M_PI / 2.0 - psi;
    omega = acos((aIn * aIn + LB * LB - LA * LA) / (2 * aIn * LB));
    beta = psi + omega - t3rad;
  } else {
    L2C = aIn * aIn + bIn * bIn;
    LC = sqrt(L2C);
    lambda = atan2(bIn, aIn);
    psi = acos((LA * LA + L2C - LB * LB) / (2 * LA * LC)) + t2rad;
    alpha = M_PI / 2.0 - lambda - psi;
    omega = acos((LB * LB + L2C - LA * LA) / (2 * LC * LB));
    beta = psi + omega - t3rad;
  }

  delta = M_PI / 2.0 - alpha - beta;

  SHOULDER_JOINT_RAD = alpha;
  ELBOW_JOINT_RAD    = beta;
  EOAT_JOINT_RAD_BUFFER  = delta;

  nanIK = isnan(alpha) || isnan(beta) || isnan(delta);
}


// AI prompt:
// *** this function is written with AI. ***

// AI prompt:
// I need a C language function. In a 2D Cartesian coordinate system, 
// input a coordinate point (x, y). The function should return two values:

// The distance from this coordinate point to the origin of the coordinate system.
// The angle, in radians, between the line connecting this point and the origin 
// of the coordinate system and the positive direction of the x-axis. 
// The angle should be in the range (-π, π).
void cartesian_to_polar(double x, double y, double* r, double* theta) {
    *r = sqrt(x * x + y * y);
    *theta = atan2(y, x);
}


// AI prompt:
// *** this function is written with AI. ***
// 我现在需要一个功能与上面函数相反的函数：
// 输入机械臂三个关节的轴的角度（弧度制），返回当前机械臂末端点的坐标点。

// 你在回答的过程中可以告诉我你还有什么其它需要的信息。
// '''
// use this two functions to compute the position of coordinate point
// by inputing the jointRad.
// 这个函数用于将极坐标转换为直角坐标
void polarToCartesian(double r, double theta, double &x, double &y) {
  x = r * cos(theta);
  y = r * sin(theta);
}


// this function is used to compute the position of the end point.
// input the angle of every joint in radius.
// compute the positon and save it to lastXYZ by default.
void RoArmM3_computePosbyJointRad(double base_joint_rad, double shoulder_joint_rad, double elbow_joint_rad, double wrist_joint_rad, double roll_joint_rad, double hand_joint_rad) {
  double aOut, bOut,   cOut, dOut,   eOut, fOut,   gOut, hOut;
  double r_ee, z_ee;

  polarToCartesian(l2, ((M_PI / 2) - (shoulder_joint_rad + t2rad)), aOut, bOut);
  // Serial.print("a:");Serial.print(aOut);Serial.print(" - b:");Serial.print(bOut);
  polarToCartesian(l3, ((M_PI / 2) - (elbow_joint_rad + shoulder_joint_rad + t3rad)), cOut, dOut);
  // Serial.print(" - c:");Serial.print(cOut);Serial.print(" - d:");Serial.print(dOut);

  // polarToCartesian(lE, -((-wrist_joint_rad + tErad)  - (M_PI/2 - shoulder_joint_rad - elbow_joint_rad)), eOut, fOut);
  polarToCartesian(lE, ((M_PI / 2) - (elbow_joint_rad + shoulder_joint_rad + wrist_joint_rad + tErad)), eOut, fOut);
  
  // Serial.print(" - e:");Serial.print(eOut);Serial.print(" - f:");Serial.println(fOut);

  r_ee = aOut + cOut + eOut;
  z_ee = bOut + dOut + fOut;

  polarToCartesian(r_ee, base_joint_rad, gOut, hOut);

  lastX = gOut;
  lastY = hOut;
  lastZ = z_ee;
  // lastT = wrist_joint_rad - (M_PI - shoulder_joint_rad - elbow_joint_rad) + (M_PI / 2);
  // lastT = ((M_PI / 2) - (elbow_joint_rad + shoulder_joint_rad - wrist_joint_rad + tErad));
  lastT = (elbow_joint_rad + shoulder_joint_rad + wrist_joint_rad) - M_PI/2;
}


// EEmode funcs change here.
// get position by servo feedback.
void RoArmM3_getPosByServoFeedback() {
  getFeedback(BASE_SERVO_ID, true);
  getFeedback(SHOULDER_DRIVING_SERVO_ID, true);
  getFeedback(ELBOW_SERVO_ID, true);

  getFeedback(WRIST_SERVO_ID, true);
  getFeedback(ROLL_SERVO_ID, true);

  getFeedback(GRIPPER_SERVO_ID, true);

  radB = calculateRadByFeedback(servoFeedback[BASE_SERVO_ID - 11].pos, BASE_JOINT);
  radS = calculateRadByFeedback(servoFeedback[SHOULDER_DRIVING_SERVO_ID - 11].pos, SHOULDER_JOINT);
  radE = calculateRadByFeedback(servoFeedback[ELBOW_SERVO_ID - 11].pos, ELBOW_JOINT);
  radT = calculateRadByFeedback(servoFeedback[WRIST_SERVO_ID - 11].pos, WRIST_JOINT);
  radR = calculateRadByFeedback(servoFeedback[ROLL_SERVO_ID - 11].pos, ROLL_JOINT);
  radG = calculateRadByFeedback(servoFeedback[GRIPPER_SERVO_ID - 11].pos, EOAT_JOINT);

  RoArmM3_computePosbyJointRad(radB, radS, radE, radT, radR, radG);
}


// feedback info in json.
void RoArmM3_infoFeedback() {
  jsonInfoHttp.clear();
  jsonInfoHttp["T"] = 1051;
  jsonInfoHttp["x"] = lastX;
  jsonInfoHttp["y"] = lastY;
  jsonInfoHttp["z"] = lastZ;
  jsonInfoHttp["tit"] = lastT;

  jsonInfoHttp["b"] = radB;
  jsonInfoHttp["s"] = radS;
  jsonInfoHttp["e"] = radE;
  jsonInfoHttp["t"] = radT;
  jsonInfoHttp["r"] = radR;
  jsonInfoHttp["g"] = radG;

  jsonInfoHttp["tB"] = servoFeedback[BASE_SERVO_ID - 11].load;
  jsonInfoHttp["tS"] = servoFeedback[SHOULDER_DRIVING_SERVO_ID - 11].load - servoFeedback[SHOULDER_DRIVEN_SERVO_ID - 11].load;
  jsonInfoHttp["tE"] = servoFeedback[ELBOW_SERVO_ID - 11].load;
  jsonInfoHttp["tT"] = servoFeedback[WRIST_SERVO_ID - 11].load;
  jsonInfoHttp["tR"] = servoFeedback[ROLL_SERVO_ID - 11].load;
  jsonInfoHttp["tG"] = servoFeedback[GRIPPER_SERVO_ID - 11].load;

  String getInfoJsonString;
  serializeJson(jsonInfoHttp, getInfoJsonString);
  Serial.println(getInfoJsonString);
}


// AI prompt:
// 在平面直角坐标系中，有一点A，输入点A的X,Y坐标和角度参数theta(弧度制)，点A绕直角坐标系原点
// 逆时针转动theta作为点B，返回点B的XY坐标值，我需要C语言的函数。
// AI prompt:
// In a 2D Cartesian coordinate system, there is a point A.
// Input the X and Y coordinates of point A and an angle parameter theta (in radians).
// Point A rotates counterclockwise around the origin of the Cartesian coordinate 
// system by an angle of theta to become point B. Return the XY coordinates of point B.
// I need a C language function.
void rotatePoint(double theta, double *xB, double *yB) {
  double alpha = tErad + theta;

  *xB = -lE * cos(alpha);
  *yB = -lE * sin(alpha);
}


// AI prompt:
// 在平面直角坐标系种，有一点A，输入点A的X,Y坐标值，输入一个距离参数S，
// 点A向原点方向移动S作为点B，返回点B的坐标值。我需要C语言的函数。
void movePoint(double xA, double yA, double s, double *xB, double *yB) {
  double distance = sqrt(pow(xA, 2) + pow(yA, 2));  
  if(distance - s <= 1e-6) {
    *xB = 0; 
    *yB = 0;
  }
  else {
    double ratio = (distance - s) / distance;
    *xB = xA * ratio;
    *yB = yA * ratio;
  }
}


// ---===< Muti-assembly IK config here >===---
// change this func and goalPosMove()
// Coordinate Ctrl: input the coordinate point of the goal position to compute
// the goalPos of every joints.
void RoArmM3_baseCoordinateCtrl(double inputX, double inputY, double inputZ, double inputT, double inputR, double inputG){
  rotatePoint((inputT - M_PI), &delta_x, &delta_y);
  // Serial.print("delta_X:");Serial.print(delta_x);Serial.print(" - delta_y:");Serial.println(delta_y);
  movePoint(inputX, inputY, delta_x, &beta_x, &beta_y);
  cartesian_to_polar(beta_x, beta_y, &base_r, &BASE_JOINT_RAD);
  simpleLinkageIkRad(l2, l3, base_r, inputZ + delta_y);
  WRIST_JOINT_RAD = EOAT_JOINT_RAD_BUFFER + inputT;
  ROLL_JOINT_RAD = inputR;
  EOAT_JOINT_RAD = inputG;
}


// update last position for later use.
void RoArmM3_lastPosUpdate(){
  lastX = goalX;
  lastY = goalY;
  lastZ = goalZ;
  lastT = goalT;
  lastR = goalR;
  // lastG = goalG;
}


// use jointCtrlRad functions to compute goalPos for every servo,
// then use this function to move the servos.
// cuz the functions like baseCoordinateCtrl is not gonna make servos move.
void RoArmM3_goalPosMove(){
  RoArmM3_baseJointCtrlRad(0, BASE_JOINT_RAD, 0, 0);
  RoArmM3_shoulderJointCtrlRad(0, SHOULDER_JOINT_RAD, 0, 0);
  RoArmM3_elbowJointCtrlRad(0, ELBOW_JOINT_RAD, 0, 0);
  
  RoArmM3_wristJointCtrlRad(0, WRIST_JOINT_RAD, 0, 0);
  RoArmM3_rollJointCtrlRad(0, ROLL_JOINT_RAD, 0, 0);

  RoArmM3_handJointCtrlRad(0, EOAT_JOINT_RAD, 0, 0);
  st.SyncWritePosEx(servoID, 7, goalPos, moveSpd, moveAcc);
}


// ctrl a single joint abs angle(rad).
// joint: 1-BASE_JOINT + ->left
//        2-SHOULDER_JOINT + ->down
//        3-ELBOW_JOINT + ->down
//        4-EOAT_JOINT + ->grab/down
// inputRad: input the goal angle in radius of the joint.
// inputSpd: move speed, steps/second.
// inputAcc: acceleration, steps/second^2.
void RoArmM3_singleJointAbsCtrl(byte jointInput, double inputRad, u16 inputSpd, u8 inputAcc){
  switch(jointInput){
  case BASE_JOINT:
    RoArmM3_baseJointCtrlRad(1, inputRad, inputSpd, inputAcc);
    BASE_JOINT_RAD = inputRad;
    break;
  case SHOULDER_JOINT:
    RoArmM3_shoulderJointCtrlRad(1, inputRad, inputSpd, inputAcc);
    SHOULDER_JOINT_RAD = inputRad;
    break;
  case ELBOW_JOINT:
    RoArmM3_elbowJointCtrlRad(1, inputRad, inputSpd, inputAcc);
    ELBOW_JOINT_RAD = inputRad;
    break;
  case WRIST_JOINT:
    RoArmM3_wristJointCtrlRad(1, inputRad, inputSpd, inputAcc);
    WRIST_JOINT_RAD = inputRad;
    break;
  case ROLL_JOINT:
    RoArmM3_rollJointCtrlRad(1, inputRad, inputSpd, inputAcc);
    ROLL_JOINT_RAD = inputRad;
    break;
  case EOAT_JOINT:
    RoArmM3_handJointCtrlRad(1, inputRad, inputSpd, inputAcc);
    EOAT_JOINT_RAD = inputRad;
    break;
  }
  RoArmM3_computePosbyJointRad(BASE_JOINT_RAD, SHOULDER_JOINT_RAD, ELBOW_JOINT_RAD, WRIST_JOINT_RAD, ROLL_JOINT_RAD, EOAT_JOINT_RAD);
}


// ctrl all joints together.
// when all joints in initPos(middle position), it looks like below.
//    -------L3------------O==L2B===
//                         ^       |
//                         |       |
//                   ELBOW_JOINT   |
//                                L2A
//                                 |
//                                 |
//           ^                     |
//           |   SHOULDER_JOINT -> O
//           Z+                    |
//           |                    L1
//           |                     |
//   <---X+--Y+      BASE_JOINT -> X
//
//
//    -------L3------------O==L2B==O  <- BASE_JOINT
//                         ^       
//   <---X+--Z+            |       
//           |       ELBOW_JOINT   
//           Y+
//           |
//           v
void RoArmM3_allJointAbsCtrl(double inputBase, double inputShoulder, double inputElbow, double inputWrist, double inputRoll, double inputHand, u16 inputSpd, u8 inputAcc){
  RoArmM3_baseJointCtrlRad(0, inputBase, inputSpd, inputAcc);
  RoArmM3_shoulderJointCtrlRad(0, inputShoulder, inputSpd, inputAcc);
  RoArmM3_elbowJointCtrlRad(0, inputElbow, inputSpd, inputAcc);
  RoArmM3_wristJointCtrlRad(0, inputWrist, inputSpd, inputAcc);
  RoArmM3_rollJointCtrlRad(0, inputRoll, inputSpd, inputAcc);
  RoArmM3_handJointCtrlRad(0, inputHand, inputSpd, inputAcc);
  for (int i = 0;i < 7;i++) {
    moveSpd[i] = inputSpd;
    moveAcc[i] = inputAcc;
  }
  st.SyncWritePosEx(servoID, 7, goalPos, moveSpd, moveAcc);
  for (int i = 0;i < 7;i++) {
    moveSpd[i] = 0;
    moveAcc[i] = 0;
  }
}


// ctrl the movement in a smooth way.
// |                 ..  <-numEnd
// |             .    |
// |           .    
// |         .        |
// |        .
// |      .           |
// |. . <-numStart
// ----------------------
// 0                  1 rateInput
double besselCtrl(double numStart, double numEnd, double rateInput){
  double numOut;
  numOut = (numEnd - numStart)*((cos(rateInput*M_PI+M_PI)+1)/2) + numStart;
  return numOut;
}
   

// use this function to get the max deltaSteps.
// get the max offset between [goal] and [last] position.
double maxNumInArray(){
  double deltaPos[6] = {abs(goalX - lastX),
                        abs(goalY - lastY),
                        abs(goalZ - lastZ),
                        abs(goalT - lastT),
                        abs(goalR - lastR)};
  double maxVal = deltaPos[0];
  for(int i = 0; i < (sizeof(deltaPos) / sizeof(deltaPos[0])); i++){
    maxVal = max(deltaPos[i],maxVal);
  }
  return maxVal;
}


// use this function to move the end of the arm to the goal position.
void RoArmM3_movePosGoalfromLast(float spdInput){
  double deltaSteps = maxNumInArray();

  double bufferX;
  double bufferY;
  double bufferZ;
  double bufferT;
  double bufferR;
  // double bufferG;

  static double bufferLastX;
  static double bufferLastY;
  static double bufferLastZ;
  static double bufferLastT;
  static double bufferLastR;
  // static double bufferLastG;

  for(double i=0;i<=1;i+=(1/(deltaSteps*1))*spdInput){
    // Serial.print("bufferX:");Serial.print(bufferX);
    // Serial.print(" - goalX:");Serial.print(goalX);
    // Serial.print(" - lastX:");Serial.println(lastX);
    bufferX = besselCtrl(lastX, goalX, i);
    bufferY = besselCtrl(lastY, goalY, i);
    bufferZ = besselCtrl(lastZ, goalZ, i);
    bufferT = besselCtrl(lastT, goalT, i);
    bufferR = besselCtrl(lastR, goalR, i);
    // bufferG = besselCtrl(lastG, goalG, i);
    RoArmM3_baseCoordinateCtrl(bufferX, bufferY, bufferZ, bufferT, bufferR, goalG);
    if(nanIK){
      // Serial.println("nanIK XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX                           xxxxxxxxxxxxxxxxxxxxxxxx");
      // IK failed
      goalX = bufferLastX;
      goalY = bufferLastY;
      goalZ = bufferLastZ;
      goalT = bufferLastT;
      goalR = bufferLastR;
      // goalG = bufferLastG;
      RoArmM3_baseCoordinateCtrl(goalX, goalY, goalZ, goalT, goalR, goalG);
      RoArmM3_goalPosMove();
      RoArmM3_lastPosUpdate();
      return;
    }
    else{
      // IK succeed.
      bufferLastX = bufferX;
      bufferLastY = bufferY;
      bufferLastZ = bufferZ;
      bufferLastT = bufferT;
      bufferLastR = bufferR;
      // bufferLastG = bufferG;
    }
    RoArmM3_goalPosMove();
    delay(2);
  }
  RoArmM3_baseCoordinateCtrl(goalX, goalY, goalZ, goalT, goalR, goalG);
  RoArmM3_goalPosMove();
  RoArmM3_lastPosUpdate();
  // Serial.print("--- --- ---");
  // Serial.print("bufferX:");Serial.print(bufferX);
  // Serial.print(" - goalX:");Serial.print(goalX);
  // Serial.print(" - lastX:");Serial.println(lastX);
}


// ctrl a single axi abs pos(mm).
// the init position is
// axiInput: 1-X, posInput:initX
//           2-Y, posInput:initY
//           3-Z, posInput:initZ
//           4-T, posInput:initT
// initX = l3+l2B
// initY = 0
// initZ = l2A
// initT = M_PI
// default inputSpd = 0.25
void RoArmM3_singlePosAbsBesselCtrl(byte axiInput, double posInput, double inputSpd){
  switch(axiInput){
    case 1: goalX = posInput;break;
    case 2: goalY = posInput;break;
    case 3: goalZ = posInput;break;
    case 4: goalT = posInput;break;
    case 5: goalR = posInput;break;
    case 6: goalG = posInput;break;
  }
  RoArmM3_movePosGoalfromLast(inputSpd);
}


// ctrl all axis abs position.
// initX = l3+l2B
// initY = 0
// initZ = l2A
// initT = M_PI
// default inputSpd = 0.36
void RoArmM3_allPosAbsBesselCtrl(double inputX, double inputY, double inputZ, double inputT, double inputR, double inputG, double inputSpd){
  goalX = inputX;
  goalY = inputY;
  goalZ = inputZ;
  goalT = inputT;
  goalR = inputR;
  goalG = inputG;
  RoArmM3_movePosGoalfromLast(inputSpd);
}


// AI prompt:
// I need a function that inputs the center coordinate point, 
// radius and scale(t), and when the scale(t) changes from 0 to 1, 
// the coordinate points output by the function can form a complete circle.
void getCirclePointYZ(double cx, double cy, double r, double t) {
  double theta = t * 2 * M_PI;
  goalY = cx + r * cos(theta);
  goalZ = cy + r * sin(theta);
}


// delay cmd.
void RoArmM3_delayMillis(int inputTime) {
  delay(inputTime);
}


// set the P&I/PID of a joint.
void RoArmM3_setJointPID(byte jointInput, float inputP, float inputI) {
  switch (jointInput) {
  case BASE_JOINT:
        st.writeByte(BASE_SERVO_ID, ST_PID_P_ADDR, inputP);
        st.writeByte(BASE_SERVO_ID, ST_PID_I_ADDR, inputI);
        break;
  case SHOULDER_JOINT:
        st.writeByte(SHOULDER_DRIVING_SERVO_ID, ST_PID_P_ADDR, inputP);
        st.writeByte(SHOULDER_DRIVING_SERVO_ID, ST_PID_I_ADDR, inputI);

        st.writeByte(SHOULDER_DRIVEN_SERVO_ID, ST_PID_P_ADDR, inputP);
        st.writeByte(SHOULDER_DRIVEN_SERVO_ID, ST_PID_I_ADDR, inputI);
        break;
  case ELBOW_JOINT:
        st.writeByte(ELBOW_SERVO_ID, ST_PID_P_ADDR, inputP);
        st.writeByte(ELBOW_SERVO_ID, ST_PID_I_ADDR, inputI);
        break;

  case WRIST_JOINT:
        st.writeByte(WRIST_SERVO_ID, ST_PID_P_ADDR, inputP);
        st.writeByte(WRIST_SERVO_ID, ST_PID_I_ADDR, inputI);
        break;
  case ROLL_JOINT:
        st.writeByte(ROLL_SERVO_ID, ST_PID_P_ADDR, inputP);
        st.writeByte(ROLL_SERVO_ID, ST_PID_I_ADDR, inputI);
        break;

  case EOAT_JOINT:
        st.writeByte(GRIPPER_SERVO_ID, ST_PID_P_ADDR, inputP);
        st.writeByte(GRIPPER_SERVO_ID, ST_PID_I_ADDR, inputI);
        break;
  }
}


// reset the P&I/PID of RoArm-M2.
void RoArmM3_resetPID() {
  RoArmM3_setJointPID(BASE_JOINT, 16, 0);
  RoArmM3_setJointPID(SHOULDER_JOINT, 16, 0);
  RoArmM3_setJointPID(ELBOW_JOINT, 16, 0);
  RoArmM3_setJointPID(WRIST_JOINT, 16, 0);
  RoArmM3_setJointPID(ROLL_JOINT, 16, 0);
  RoArmM3_setJointPID(EOAT_JOINT, 16, 0);
}


// input the angle in deg, and it returns the number of servo steps.
int calculatePosByDeg(double degInput) {
  return round((degInput / 360) * ARM_SERVO_POS_RANGE);
}


double ang2deg(double inputAng) {
  return (inputAng / 180) * M_PI;
}


// ctrl a single joint abs angle.
// jointInput: 1-BASE_JOINT
//             2-SHOULDER_JOINT
//             3-ELBOW_JOINT
//             4-HAND_JOINT
// inputRad: input the goal angle in deg of the joint.
// inputSpd: move speed, angle/second.
// inputAcc: acceleration, angle/second^2.
void RoArmM3_singleJointAngleCtrl(byte jointInput, double inputAng, u16 inputSpd, u8 inputAcc){
  // Serial.println("---");
  // Serial.print(jointInput);Serial.print("\t");Serial.print(inputAng);Serial.print("\t");
  // Serial.print(inputSpd);Serial.print("\t");Serial.print(inputAcc);Serial.println();
  inputSpd = abs(inputSpd);
  inputAcc = abs(inputAcc);
  switch(jointInput){
  case BASE_JOINT:
    BASE_JOINT_ANG = inputAng;
    Serial.println(inputAng);
    BASE_JOINT_RAD = ang2deg(inputAng);
    Serial.println(BASE_JOINT_RAD);
    RoArmM3_baseJointCtrlRad(1, BASE_JOINT_RAD, calculatePosByDeg(inputSpd), calculatePosByDeg(inputAcc));
    break;
  case SHOULDER_JOINT:
    SHOULDER_JOINT_ANG = inputAng;
    SHOULDER_JOINT_RAD = ang2deg(inputAng);
    RoArmM3_shoulderJointCtrlRad(1, SHOULDER_JOINT_RAD, calculatePosByDeg(inputSpd), calculatePosByDeg(inputAcc));
    break;
  case ELBOW_JOINT:
    ELBOW_JOINT_ANG = inputAng;
    ELBOW_JOINT_RAD = ang2deg(inputAng);
    RoArmM3_elbowJointCtrlRad(1, ELBOW_JOINT_RAD, calculatePosByDeg(inputSpd), calculatePosByDeg(inputAcc));
    break;
  case WRIST_JOINT:
    WRIST_JOINT_ANG = inputAng;
    WRIST_JOINT_RAD = ang2deg(inputAng);
    RoArmM3_wristJointCtrlRad(1, WRIST_JOINT_RAD, calculatePosByDeg(inputSpd), calculatePosByDeg(inputAcc));
    break;
  case ROLL_JOINT:
    ROLL_JOINT_ANG = inputAng;
    ROLL_JOINT_RAD = ang2deg(inputAng);
    RoArmM3_rollJointCtrlRad(1, ROLL_JOINT_RAD, calculatePosByDeg(inputSpd), calculatePosByDeg(inputAcc));
    break;
  case EOAT_JOINT:
    EOAT_JOINT_ANG = inputAng;
    EOAT_JOINT_RAD = ang2deg(inputAng);
    RoArmM3_handJointCtrlRad(1, EOAT_JOINT_RAD, calculatePosByDeg(inputSpd), calculatePosByDeg(inputAcc));
    break;
  }
  RoArmM3_computePosbyJointRad(BASE_JOINT_RAD, SHOULDER_JOINT_RAD, ELBOW_JOINT_RAD, WRIST_JOINT_RAD, ROLL_JOINT_RAD, EOAT_JOINT_RAD);
}


// ctrl all joints together.
// when all joints in initPos(middle position), it looks like below.
//    -------L3------------O==L2B===
//                         ^       |
//                         |       |
//                   ELBOW_JOINT   |
//                                L2A
//                                 |
//                                 |
//           ^                     |
//           |   SHOULDER_JOINT -> O
//           Z+                    |
//           |                    L1
//           |                     |
//   <---X+--Y+      BASE_JOINT -> X
//
//
//    -------L3------------O==L2B==O  <- BASE_JOINT
//                         ^       
//   <---X+--Z+            |       
//           |       ELBOW_JOINT   
//           Y+
//           |
//           v
void RoArmM3_allJointsAngleCtrl(double inputBase, double inputShoulder, double inputElbow, double inputWrist, double inputRoll, double inputHand, u16 inputSpd, u8 inputAcc){
  BASE_JOINT_ANG = inputBase;
  BASE_JOINT_RAD = ang2deg(inputBase);

  SHOULDER_JOINT_ANG = inputShoulder;
  SHOULDER_JOINT_RAD = ang2deg(inputShoulder);

  ELBOW_JOINT_ANG = inputElbow;
  ELBOW_JOINT_RAD = ang2deg(inputElbow);
  

  WRIST_JOINT_ANG = inputWrist;
  WRIST_JOINT_RAD = ang2deg(inputWrist);

  ROLL_JOINT_ANG = inputRoll;
  ROLL_JOINT_RAD = ang2deg(inputRoll);


  EOAT_JOINT_ANG = inputHand;
  EOAT_JOINT_RAD = ang2deg(inputHand);

  RoArmM3_baseJointCtrlRad(0, BASE_JOINT_RAD, 0, 0);
  RoArmM3_shoulderJointCtrlRad(0, SHOULDER_JOINT_RAD, 0, 0);
  RoArmM3_elbowJointCtrlRad(0, ELBOW_JOINT_RAD, 0, 0);
  RoArmM3_wristJointCtrlRad(0, WRIST_JOINT_RAD, 0, 0);
  RoArmM3_rollJointCtrlRad(0, ROLL_JOINT_RAD, 0, 0);
  RoArmM3_handJointCtrlRad(0, EOAT_JOINT_RAD, 0, 0);
  inputSpd = abs(calculatePosByDeg(inputSpd));
  inputAcc = abs(calculatePosByDeg(inputAcc));
  for (int i = 0;i < 7;i++) {
    moveSpd[i] = inputSpd;
    moveAcc[i] = inputAcc;
  }
  st.SyncWritePosEx(servoID, 7, goalPos, moveSpd, moveAcc);
}


void constantCtrl(byte inputMode, byte inputAxis, byte inputCmd, byte inputSpd) {
  const_mode = inputMode;
  if (const_mode == CONST_ANGLE) {
    const_spd = abs(inputSpd) * 0.0005;
  } else if (const_mode == CONST_XYZT) {
    const_spd = abs(inputSpd) * 0.1;
  }

  switch (inputAxis) {
  case BASE_JOINT:
          const_cmd_base_x = inputCmd;
          break;
  case SHOULDER_JOINT:
          const_cmd_shoulder_y = inputCmd;
          break;
  case ELBOW_JOINT:
          const_cmd_elbow_z = inputCmd;
          break;
  case WRIST_JOINT:
          const_cmd_wrist_t = inputCmd;
          break;
  case ROLL_JOINT:
          const_cmd_roll_r = inputCmd;
          break;
  case EOAT_JOINT:
          const_cmd_eoat_g = inputCmd;
          break;
  }
}


void constantHandle() {
  if (!const_cmd_base_x && !const_cmd_shoulder_y && !const_cmd_elbow_z && !const_cmd_wrist_t && !const_cmd_roll_r && !const_cmd_eoat_g) {
    const_goal_base = radB;
    const_goal_shoulder = radS;
    const_goal_elbow = radE;
    const_goal_wrist = radT;
    const_goal_roll = radR;
    const_goal_eoat = radG;

    goalX = lastX;
    goalY = lastY;
    goalZ = lastZ;
    goalT = lastT;
    goalR = lastR;
    goalG = lastG;
    return;
  }

  if (const_cmd_base_x == MOVE_INCREASE) {
    if (const_mode == CONST_ANGLE) {
      const_goal_base += const_spd;
      if (const_goal_base > M_PI) {
        const_goal_base = M_PI;
        const_cmd_base_x = MOVE_STOP;
      }
    }
    else if (const_mode == CONST_XYZT) {
      goalX += const_spd;
    }
  } else if (const_cmd_base_x == MOVE_DECREASE) {
    if (const_mode == CONST_ANGLE) {
      const_goal_base -= const_spd;
      if (const_goal_base < -M_PI) {
        const_goal_base = -M_PI;
        const_cmd_base_x = MOVE_STOP;
      }
    }
    else if (const_mode == CONST_XYZT) {
      goalX -= const_spd;
    }
  }


  if (const_cmd_shoulder_y == MOVE_INCREASE) {
    if (const_mode == CONST_ANGLE) {
      const_goal_shoulder += const_spd;
      if (const_goal_shoulder > M_PI/2) {
        const_goal_shoulder = M_PI/2;
        const_cmd_shoulder_y = MOVE_STOP;
      }
    }
    else if (const_mode == CONST_XYZT) {
      goalY += const_spd;
    }
  } else if (const_cmd_shoulder_y == MOVE_DECREASE) {
    if (const_mode == CONST_ANGLE) {
      const_goal_shoulder -= const_spd;
      if (const_goal_shoulder < -M_PI/2) {
        const_goal_shoulder = -M_PI/2;
        const_cmd_shoulder_y = MOVE_STOP;
      }
    }
    else if (const_mode == CONST_XYZT) {
      goalY -= const_spd;
    }
  }


  if (const_cmd_elbow_z == MOVE_INCREASE) {
    if (const_mode == CONST_ANGLE) {
      const_goal_elbow += const_spd;
      if (const_goal_elbow > M_PI) {
        const_goal_elbow = M_PI;
        const_cmd_elbow_z = MOVE_STOP;
      }
    }
    else if (const_mode == CONST_XYZT) {
      goalZ += const_spd;
    }
  } else if (const_cmd_elbow_z == MOVE_DECREASE) {
    if (const_mode == CONST_ANGLE) {
      const_goal_elbow -= const_spd;
      if (const_goal_elbow < -M_PI/4) {
        const_goal_elbow = -M_PI/4;
        const_cmd_elbow_z = MOVE_STOP;
      }
    }
    else if (const_mode == CONST_XYZT) {
      goalZ -= const_spd;
    }
  }



  if (const_cmd_wrist_t == MOVE_INCREASE) {
    if (const_mode == CONST_ANGLE) {
      const_goal_wrist += const_spd;
      if (const_goal_wrist > M_PI/2) {
        const_goal_wrist = M_PI/2;
        const_cmd_wrist_t = MOVE_STOP;
      }
    }
    else if (const_mode == CONST_XYZT) {
      goalT += const_spd/200;
    }
  } else if (const_cmd_wrist_t == MOVE_DECREASE) {
    if (const_mode == CONST_ANGLE) {
      const_goal_wrist -= const_spd;
      if (const_goal_wrist < -M_PI/2) {
        const_goal_wrist = -M_PI/2;
        const_cmd_wrist_t = MOVE_STOP;
      }
    }
    else if (const_mode == CONST_XYZT) {
      goalT -= const_spd/200;
    }
  }

  if (const_cmd_roll_r == MOVE_INCREASE) {
    if (const_mode == CONST_ANGLE) {
      const_goal_roll += const_spd;
      if (const_goal_roll > M_PI) {
        const_goal_roll = M_PI;
        const_cmd_roll_r = MOVE_STOP;
      }
    }
    else if (const_mode == CONST_XYZT) {
      goalR += const_spd/200;
    }
  } else if (const_cmd_roll_r == MOVE_DECREASE) {
    if (const_mode == CONST_ANGLE) {
      const_goal_roll -= const_spd;
      if (const_goal_roll < -M_PI) {
        const_goal_roll = -M_PI;
        const_cmd_roll_r = MOVE_STOP;
      }
    }
    else if (const_mode == CONST_XYZT) {
      goalR -= const_spd/200;
    }
  }



  if (const_cmd_eoat_g == MOVE_INCREASE) {
    if (const_mode == CONST_ANGLE) {
      const_goal_eoat += const_spd;
      if (const_goal_eoat > M_PI*7/4) {
        const_goal_eoat = M_PI*7/4;
        const_cmd_eoat_g = MOVE_STOP;
      }
    }
    else if (const_mode == CONST_XYZT) {
      goalG += const_spd/200;
    }
  } else if (const_cmd_eoat_g == MOVE_DECREASE) {
    if (const_mode == CONST_ANGLE) {
      const_goal_eoat -= const_spd;
      if (const_goal_eoat < -M_PI/4) {
        const_goal_eoat = -M_PI/4;
        const_cmd_eoat_g = MOVE_STOP;
      }
    }
    else if (const_mode == CONST_XYZT) {
      goalG -= const_spd/200;
    }
  }

  if (const_mode == CONST_ANGLE) {
    RoArmM3_allJointAbsCtrl(const_goal_base, const_goal_shoulder, const_goal_elbow, const_goal_wrist, const_goal_roll, const_goal_eoat, 0, 0);
  } else if (const_mode == CONST_XYZT) {

    static double bufferLastX;
    static double bufferLastY;
    static double bufferLastZ;
    static double bufferLastT;
    static double bufferLastR;
    static double bufferLastG;

    RoArmM3_baseCoordinateCtrl(goalX, goalY, goalZ, goalT, goalR, goalG);
    if (nanIK) {
      // IK failed
      goalX = bufferLastX;
      goalY = bufferLastY;
      goalZ = bufferLastZ;
      goalT = bufferLastT;
      goalR = bufferLastR;
      goalG = bufferLastG;
      RoArmM3_baseCoordinateCtrl(bufferLastX, bufferLastY, bufferLastZ, bufferLastT, bufferLastR, bufferLastG);
      RoArmM3_goalPosMove();
      RoArmM3_lastPosUpdate();
      return;
    }
    else  {
      bufferLastX = goalX;
      bufferLastY = goalY;
      bufferLastZ = goalZ;
      bufferLastT = goalT;
      bufferLastR = goalR;
      bufferLastG = goalG;
    }
    RoArmM3_goalPosMove();
    RoArmM3_lastPosUpdate();
  }
}