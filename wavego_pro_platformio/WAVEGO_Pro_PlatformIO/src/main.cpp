// for PlatformIO. This is the main file of the project.
// This file is the entry point of the program.
#include <Arduino.h>
#include <ArduinoJson.h>
#include "Config.h"
#include "RGBLight.h"
#include "BodyCtrl.h"
#include "FilesCtrl.h"

JsonDocument jsonCmdReceive;
JsonDocument jsonFeedback;
DeserializationError err;
String outputString;
RGBLight led;
BodyCtrl bodyCtrl;
FilesCtrl filesCtrl;

int jointsZeroPos[12];
int jointsCurrentPos[12];
void jsonCmdReceiveHandler(const JsonDocument& jsonCmdInput);
void runMission(String missionName, int intervalTime, int loopTimes);

void setup() {
  Serial.begin(BAUD_RATE);
  Serial.println("device starting...");

  led.init();
  filesCtrl.init();

  bodyCtrl.init();
  bodyCtrl.jointMiddle();

  if(!filesCtrl.checkMission("boot")) {
    filesCtrl.createMission("boot", "this is the boot mission.");
  } 
  runMission("boot", 0, 1);
  bodyCtrl.stand(); // need to check T105 first
}

bool runStep(String missionName, int step) {
  outputString = filesCtrl.readStep(missionName, step);
  err = deserializeJson(jsonCmdReceive, outputString);
  if (err == DeserializationError::Ok) {
    jsonCmdReceiveHandler(jsonCmdReceive);
    return true;
  } else {
    Serial.print("JSON parsing error (this is a normal output when booting or running Mission): ");
    Serial.println(err.c_str());
    return false;
  }
}

void runMission(String missionName, int intervalTime, int loopTimes) {
  intervalTime = intervalTime - timeOffset;
  if (intervalTime < 0) {intervalTime = 0;}
  int j = 1;
  while (true) {
    Serial.print("Running loop: ");
    Serial.println(j);
    int i = 1;
    while (true) {
      if (Serial.available() > 0) {
        break;
      }
      if (runStep(missionName, i)) {
        Serial.print("Step: ");
        Serial.println(i);
        i++;
        delay(intervalTime);
      } else {
        Serial.println("Mission Completed.");
        break;
      }
    }
    j++;
    if (j > loopTimes && j != -1) {
      break;
    }
  }
}

void jsonCmdReceiveHandler(const JsonDocument& jsonCmdInput){
  int cmdType;
  cmdType = jsonCmdInput["T"].as<int>();
  switch(cmdType){
	case CMD_JOINT_MIDDLE:
                        bodyCtrl.jointMiddle();
                        break;
  case CMD_RELEASE_TORQUE:
                        bodyCtrl.releaseTorque();
                        break;
  case CMD_SINGLE_SERVO_CTRL:
                        bodyCtrl.singleServoCtrl(jsonCmdInput["id"], 
                                                 jsonCmdInput["goal"], 
                                                 jsonCmdInput["time"], 
                                                 jsonCmdInput["spd"]);
                        break;
  case CMD_GET_JOINTS_ZERO:
                        memcpy(jointsZeroPos, bodyCtrl.getJointsZeroPosArray(), sizeof(jointsZeroPos));
                        for (int i = 0; i < 12; i++) {  
                          Serial.print("Joint ");
                          Serial.print(i);
                          Serial.print(": ");
                          Serial.println(jointsZeroPos[i]);
                        }
                        break;
  case CMD_SET_JOINTS_ZERO:
                        for (int i = 0; i < 12; i++) {
                          jointsZeroPos[i] = jsonCmdInput["set"][i];
                        }
                        bodyCtrl.setJointsZeroPosArray(jointsZeroPos);
                        break;
  case CMD_GET_CURRENT_POS:
                        memcpy(jointsCurrentPos, bodyCtrl.getServoFeedback(), sizeof(jointsCurrentPos));
                        jsonFeedback.clear();
                        jsonFeedback["T"] = - CMD_GET_CURRENT_POS;
                        for (int i = 0; i < 12; i++) {
                          jsonFeedback["fb"][i] = jointsCurrentPos[i];
                        }
                        serializeJson(jsonFeedback, outputString);
                        Serial.println(outputString);
                        break;
  case CMD_SET_CURRENT_POS_ZERO:
                        bodyCtrl.setCurrentPosZero();
                        memcpy(jointsZeroPos, bodyCtrl.getJointsZeroPosArray(), sizeof(jointsZeroPos));
                        jsonFeedback.clear();
                        jsonFeedback["T"] = CMD_SET_JOINTS_ZERO;
                        for (int i = 0; i < 12; i++) {  
                          Serial.print("Joint ");
                          Serial.print(i);
                          Serial.print(": ");
                          Serial.println(jointsZeroPos[i]);
                          jsonFeedback["set"][i] = jointsZeroPos[i];
                        }
                        serializeJson(jsonFeedback, outputString);
                        Serial.println(outputString);
                        filesCtrl.appendStep("boot", outputString);
                        break;
  case CMD_CTRL_JOINT_ANGLE:
                        bodyCtrl.jointAngle(jsonCmdInput["joint"], jsonCmdInput["angle"]);
                        bodyCtrl.moveTrigger();
                        break;
  case CMD_CTRL_JOINT_RAD:
                        bodyCtrl.jointRad(jsonCmdInput["joint"], jsonCmdInput["rad"]);
                        bodyCtrl.moveTrigger();
                        break;



	case CMD_SET_COLOR: 
                        led.setColor(jsonCmdInput["set"][0], 
                                     jsonCmdInput["set"][1], 
                                     jsonCmdInput["set"][2], 
                                     jsonCmdInput["set"][3]);
												break;
  


  case CMD_SCAN_FILES:
                        filesCtrl.scan();
                        break;
  case CMD_CREATE_MISSION:
                        filesCtrl.createMission(jsonCmdInput["name"], 
                                                jsonCmdInput["intro"]);
                        break;
  case CMD_FORMAT_FLASH:
                        filesCtrl.flash();
                        break;
  case CMD_MISSION_CONTENT:
                        filesCtrl.missionContent(jsonCmdInput["name"]);
                        break;
  case CMD_APPEND_SETP_JSON:
                        filesCtrl.appendStep(jsonCmdInput["name"],
                                             jsonCmdInput["json"]);
                        break;
  case CMD_INSERT_SETP_JSON:
                        filesCtrl.insertStep(jsonCmdInput["name"],
                                             jsonCmdInput["step"],
                                             jsonCmdInput["json"]);
                        break;
  case CMD_REPLACE_SETP_JSON:
                        filesCtrl.replaceStep(jsonCmdInput["name"],
                                              jsonCmdInput["step"],
                                              jsonCmdInput["json"]);
                        break;
  case CMD_DELETE_SETP:
                        filesCtrl.deleteStep(jsonCmdInput["name"],
                                             jsonCmdInput["step"]);
                        break;
  case CMD_RUN_STEP:    
                        runStep(jsonCmdInput["name"], jsonCmdInput["step"]);
                        break;
  case CMD_RUN_MISSION: 
                        runMission(jsonCmdInput["name"], 
                                   jsonCmdInput["interval"], 
                                   jsonCmdInput["loop"]);
                        break;
  case CMD_DELETE_MISSION:
                        filesCtrl.deleteMission(jsonCmdInput["name"]);
                        break;
  


  case ESP32_REBOOT:
                        ESP.restart();
                        break;
  }
}




void serialCtrl() {
  static String receivedData;

  while (Serial.available() > 0) {
    char receivedChar = Serial.read();
    receivedData += receivedChar;

    // Detect the end of the JSON string based on a specific termination character
    if (receivedChar == '\n') {
      // Now we have received the complete JSON string
      DeserializationError err = deserializeJson(jsonCmdReceive, receivedData);
      if (err == DeserializationError::Ok) {
  			if (InfoPrint == 1) {
  				Serial.print(receivedData);
  			}
        jsonCmdReceiveHandler(jsonCmdReceive);
      } else {
        // Handle JSON parsing error here
        if (InfoPrint == 1) {
          Serial.print("JSON parsing error: ");
          Serial.println(err.c_str());
        }
      }
      // Reset the receivedData for the next JSON string
      receivedData = "";
    }
  }
}


void loop() {
  // put your main code here, to run repeatedly:
  serialCtrl();
}

