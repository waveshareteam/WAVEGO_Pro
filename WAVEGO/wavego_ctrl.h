// cmd type, 0 - stop
//           1 - moving
//           2 - steady
//           3 - functions
byte cmd_type = 0;

// moving and posture args.
float cmd_x_spd = 0;
float cmd_y_spd = 0;

float cmd_x_ang = 0;
float cmd_y_ang = 0;
float cmd_z_ang = 0;


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
// the args of gait:
double walk_height_max  = 110;
double walk_height_min  = 75;
double walk_height      = 95;
double walk_lift		= 9; // walk_height + walk_lift <= walk_height_max.
double walk_range		= 40;
double walk_acc			= 5;
double walk_extended_x  = 16;
double walk_extended_z  = 25;
double walk_side_max    = 30;
double walk_mass_adjust = 21;
double stand_height     = 95;

// --- --- --- --- --- ---
// walking status args:
float walk_cycle_global = 0; // 0~1
float walk_lift_prop    = 0; // walk_lift_prop < 1
float global_step  = 0;
int   step_delay   = 4;
float step_iterate = 0.04;

// --- --- --- --- --- ---
// bus servo args:
int bus_servo_middle = 511;
int bus_servo_min    = 23;
int bus_servo_max    = 1000;

#define LF_A 0
#define LF_B 1
#define LF_H 2

#define LH_A 3
#define LH_B 4
#define LH_H 5

#define RF_A 6
#define RF_B 7
#define RF_H 8

#define RH_A 9
#define RH_B 10
#define RH_H 11

int joint_id[12] = {LEG_LF_A, LEG_LF_B, LEG_LF_H,
					LEG_LH_A, LEG_LH_B, LEG_LH_H,
					LEG_RF_A, LEG_RF_B, LEG_RF_H,
					LEG_RH_A, LEG_RH_B, LEG_RH_H};

int feedback_pos[12] = {LEG_LF_A, LEG_LF_B, LEG_LF_H,
						LEG_LH_A, LEG_LH_B, LEG_LH_H,
						LEG_RF_A, LEG_RF_B, LEG_RF_H,
						LEG_RH_A, LEG_RH_B, LEG_RH_H};

int servo_middle_pos[16] = {bus_servo_middle, bus_servo_middle, bus_servo_middle, bus_servo_middle,
                           	bus_servo_middle, bus_servo_middle, bus_servo_middle, bus_servo_middle,
                           	bus_servo_middle, bus_servo_middle, bus_servo_middle, bus_servo_middle,
                           	bus_servo_middle, bus_servo_middle, bus_servo_middle, bus_servo_middle};

int goal_pos[16] = {bus_servo_middle, bus_servo_middle, bus_servo_middle, bus_servo_middle,
                    bus_servo_middle, bus_servo_middle, bus_servo_middle, bus_servo_middle,
                    bus_servo_middle, bus_servo_middle, bus_servo_middle, bus_servo_middle,
                    bus_servo_middle, bus_servo_middle, bus_servo_middle, bus_servo_middle};

#define W_RXD 18
#define W_TXD 19

// Instantiate a servo control object.
SCSCL sc;

// initialize bus servo libraris for WAVEGO legs.
void initWavegoBusServo() {
  Serial1.begin(1000000, SERIAL_8N1, W_RXD, W_TXD);
  sc.pSerial = &Serial1;
  while(!Serial1) {}
  if(InfoPrint == 1){Serial.println("WAVEGO ServoCtrl init succeed.");}
}

// move all bus servos to bus_servo_middle.
void moveAllBusServoMiddle() {
  sc.RegWritePos(254, bus_servo_middle, 0, 0);
  sc.RegWriteAction();
}

// change the bus servo id.
bool changeBusServoID(byte old_id, byte new_id) {
  if(!getFeedback(old_id, true)) {
    if(InfoPrint == 1) {Serial.print("change: ");Serial.print(old_id);Serial.println("failed");}
    return false;
  }
  else {
    sc.unLockEprom(old_id);
    sc.writeByte(old_id, SMS_STS_ID, new_id);
    sc.LockEprom(new_id);
    if(InfoPrint == 1) {Serial.print("change: ");Serial.print(old_id);Serial.println("succeed");}
    return true;
  }
}

// call this trigger to move all bus servos.
void moveTrigger() {
	sc.RegWriteAction();
}

// input the rad and output the steps.
double getServoPosByRad(double rad_input) {
	return rad_input * 195.569572506687;
}

// bus servo torque ctrl.
void busServoTorque(byte servo_id, byte torque_cmd) {
	if (torque_cmd == 0) {
		torque_cmd = 0;
	} else {
		torque_cmd = 1;
	}
	sc.EnableTorque(servo_id, torque_cmd);
}

// servo middle pos debug.
void busServoFeedback() {
  for(int i = 0; i < (sizeof(joint_id) / sizeof(joint_id[0])); i++){
    feedback_pos[i] = sc.ReadPos(joint_id[i]);
  }

  jsonInfoHttp.clear();
  jsonInfoHttp["T"] = FB_WAVEGO_STATUS;

  jsonInfoHttp["LFA"] = feedback_pos[LF_A];
  jsonInfoHttp["LFB"] = feedback_pos[LF_B];
  jsonInfoHttp["LFH"] = feedback_pos[LF_H];

  jsonInfoHttp["LHA"] = feedback_pos[LH_A];
  jsonInfoHttp["LHB"] = feedback_pos[LH_B];
  jsonInfoHttp["LHH"] = feedback_pos[LH_H];

  jsonInfoHttp["RFA"] = feedback_pos[RF_A];
  jsonInfoHttp["RFB"] = feedback_pos[RF_B];
  jsonInfoHttp["RFH"] = feedback_pos[RF_H];

  jsonInfoHttp["RHA"] = feedback_pos[RH_A];
  jsonInfoHttp["RHB"] = feedback_pos[RH_B];
  jsonInfoHttp["RHH"] = feedback_pos[RH_H];

  String getInfoJsonString;
  serializeJson(jsonInfoHttp, getInfoJsonString);
  Serial.println(getInfoJsonString);
}

// wavego status feedback.
void statusFeedback() {
	
}

// set the current pos as the middle pos.
void setBusServoMiddle() {
  for(int i = 0; i < (sizeof(joint_id) / sizeof(joint_id[0])); i++){
    servo_middle_pos[i] = feedback_pos[i];
  }
}