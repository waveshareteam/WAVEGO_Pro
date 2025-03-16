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
String outputString;
RGBLight led;
BodyCtrl bodyCtrl;
FilesCtrl filesCtrl;

int jointsZeroPos[12];
int jointsCurrentPos[12];

void setup() {
  Serial.begin(BAUD_RATE);
  Serial.println("device starting...");

  led.init();
  led.setColor(0, 255, 0, 16); // id 0 -> left LED
  led.setColor(1, 0, 32, 255); // id 1 -> right LED

  filesCtrl.init();

  bodyCtrl.init();
  bodyCtrl.jointMiddle();
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
  				Serial.println(receivedData);
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

