// for PlatformIO. This is the main file of the project.
// This file is the entry point of the program.
#include <Arduino.h>
#include <ArduinoJson.h>
#include "Config.h"
#include "RGBLight.h"
#include "BodyCtrl.h"

JsonDocument jsonCmdReceive;
JsonDocument jsonFeedback;
RGBLight led;
BodyCtrl bodyCtrl;

int jointsZeroPos[12];

void setup() {
  Serial.begin(BAUD_RATE);
  Serial.println("device starting...");

  led.init();
  led.setColor(0, 255, 0, 16); // id 0 -> left LED
  led.setColor(1, 0, 32, 255); // id 1 -> right LED

  bodyCtrl.init();
  bodyCtrl.jointMiddle();
}



void jsonCmdReceiveHandler(const JsonDocument& jsonCmdInput){
	int cmdType = jsonCmdInput["T"].as<int>();
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




	case CMD_SET_COLOR: 
                        led.setColor(jsonCmdInput["set"][0], 
                                     jsonCmdInput["set"][1], 
                                     jsonCmdInput["set"][2], 
                                     jsonCmdInput["set"][3]);
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

