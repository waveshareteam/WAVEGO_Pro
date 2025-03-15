// for Arduino IDE
#include <ArduinoJson.h>
StaticJsonDocument<256> jsonCmdReceive;
StaticJsonDocument<256> jsonInfoSend;
StaticJsonDocument<256> jsonInfoHttp;

#include <SCServo.h>
#include <Preferences.h>
#include <nvs_flash.h>
#include <esp_system.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <WebServer.h>
#include <esp_now.h>
#include <nvs_flash.h>

// config.
#include "wavego_config.h"

// functions for oled.
#include "oled_ctrl.h"

// functions for RoArm-M3 ctrl.(there may be a robotic arm on wavego)
#include "RoArm-M3_module.h"

// functions for pneumatic modules and lights ctrl. 
#include "switch_module.h"

// define json cmd.
#include "json_cmd.h"

// functions for WAVEGO ctrl.
#include "wavego_ctrl.h"

// functions for editing the files in flash.
#include "files_ctrl.h"

// advance functions for RoArm-M3 ctrl.
#include "mission_advance.h"

// functions for wifi ctrl.
#include "wifi_ctrl.h"

// functions for esp-now.
#include "esp_now_ctrl.h"

// functions for uart json ctrl.
#include "uart_ctrl.h"

// functions for http & web server.
#include "http_server.h"


void setup() {
  Serial.begin(115200);
  Wire.begin(S_SDA, S_SCL);
  while(!Serial) {}

  delay(1200);

  initOLED();
  screenLine_0 = "WAVEGO";
  screenLine_1 = "version: 0.9";
  screenLine_2 = "starting...";
  screenLine_3 = "";
  oled_update();

  // init the littleFS funcs in files_ctrl.h
  screenLine_2 = screenLine_3;
  screenLine_3 = "Initialize LittleFS";
  oled_update();
  if(InfoPrint == 1){Serial.println("Initialize LittleFS for Flash files ctrl.");}
  initFS();

  // init the funcs in switch_module.h
  // screenLine_2 = screenLine_3;
  // screenLine_3 = "Initialize 12V-switch ctrl";
  // oled_update();
  // if(InfoPrint == 1){Serial.println("Initialize the pins used for 12V-switch ctrl.");}
  // switchPinInit();

  // servos power up
  screenLine_2 = screenLine_3;
  screenLine_3 = "Power up the servos";
  oled_update();
  if(InfoPrint == 1){Serial.println("Power up the servos.");}
  delay(500);




  initWavegoBusServo();

  
  // init servo ctrl functions.
  // screenLine_2 = screenLine_3;
  // screenLine_3 = "ServoCtrl init UART2TTL...";
  // oled_update();
  // if(InfoPrint == 1){Serial.println("ServoCtrl init UART2TTL...");}
  // RoArmM3_servoInit();

  // check the status of the servos.
  // screenLine_2 = screenLine_3;
  // screenLine_3 = "Bus servos status check...";
  // oled_update();
  // if(InfoPrint == 1){Serial.println("Bus servos status check...");}
  // RoArmM3_initCheck(false);

  // if(InfoPrint == 1 && RoArmM3_initCheckSucceed){
  //   Serial.println("All bus servos status checked.");
  // }
  // if(RoArmM3_initCheckSucceed) {
  //   screenLine_2 = "Bus servos: succeed";
  // } else {
  //   screenLine_2 = "Bus servos: " + 
  //   servoFeedback[BASE_SERVO_ID - 11].status +
  //   servoFeedback[SHOULDER_DRIVING_SERVO_ID - 11].status +
  //   servoFeedback[SHOULDER_DRIVEN_SERVO_ID - 11].status +
  //   servoFeedback[ELBOW_SERVO_ID - 11].status +
  //   servoFeedback[GRIPPER_SERVO_ID - 11].status;
  // }
  // screenLine_3 = ">>> Moving to init pos...";
  // oled_update();
  // RoArmM3_resetPID();
  // RoArmM3_moveInit();

  // screenLine_3 = "Reset joint torque to ST_TORQUE_MAX";
  // oled_update();
  // if(InfoPrint == 1){Serial.println("Reset joint torque to ST_TORQUE_MAX.");}
  // RoArmM3_dynamicAdaptation(0, ST_TORQUE_MAX, ST_TORQUE_MAX, ST_TORQUE_MAX, ST_TORQUE_MAX, ST_TORQUE_MAX, ST_TORQUE_MAX);

  screenLine_3 = "WiFi init";
  oled_update();
  if(InfoPrint == 1){Serial.println("WiFi init.");}
  initWifi();

  screenLine_3 = "http & web init";
  oled_update();
  if(InfoPrint == 1){Serial.println("http & web init.");}
  initHttpWebServer();

  screenLine_3 = "ESP-NOW init";
  oled_update();
  if(InfoPrint == 1){Serial.println("ESP-NOW init.");}
  initEspNow();

  screenLine_3 = "WAVEGO started";
  oled_update();
  if(InfoPrint == 1){Serial.println("WAVEGO started.");}

  getThisDevMacAddress();

  updateOledWifiInfo();

  screenLine_2 = String("MAC:") + macToString(thisDevMac);
  oled_update();

  if(InfoPrint == 1){Serial.println("Application initialization settings.");}
  createMission("boot", "these cmds run automatically at boot.");
  missionPlay("boot", 1);

  // RoArmM3_handTorqueCtrl(300);

  // RoArmM3_allPosAbsBesselCtrl(l2B + l3A + ARM_L4_LENGTH_MM_A, 0, l2A - ARM_L4_LENGTH_MM_B, 0, 0, 3.14, 0.25);

  moveAllBusServoMiddle();
  delay(500);
  busServoTorque(254, 0);
}


void loop() {
  serialCtrl();
  server.handleClient();

  // unsigned long curr_time = millis();
  // if (curr_time - prev_time >= 10){
  //   constantHandle();
  //   prev_time = curr_time;
  // }

  // RoArmM3_getPosByServoFeedback();
  
  // esp-now flow ctrl as a flow-leader.
  switch(espNowMode) {
  case 1: espNowGroupDevsFlowCtrl();break;
  case 2: espNowSingleDevFlowCtrl();break;
  }

  // if (InfoPrint == 2) {
  //   RoArmM3_infoFeedback();
  // }

  if(runNewJsonCmd) {
    jsonCmdReceiveHandler();
    jsonCmdReceive.clear();
    runNewJsonCmd = false;
  }

  busServoFeedback();
  statusFeedback();
  // sc.RegWritePos(254, bus_servo_middle, 0, 0);
  // sc.RegWriteAction();
  // delay(1000);
  // sc.RegWritePos(254, bus_servo_middle + 30, 0, 0);
  // sc.RegWriteAction();
  // delay(1000);
}
