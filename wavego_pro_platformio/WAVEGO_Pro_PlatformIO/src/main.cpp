// for PlatformIO. This is the main file of the project.
// This file is the entry point of the program.
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <nvs_flash.h>
#include "Config.h"
#include "RGBLight.h"
#include "BodyCtrl.h"
#include "FilesCtrl.h"
#include "Wireless.h"
#include "ScreenCtrl.h"
#include "web_page.h"
#include <WebServer.h>
#include "devs.h"

JsonDocument jsonCmdReceive;
JsonDocument jsonFeedback;
DeserializationError err;
String outputString;
RGBLight led;
BodyCtrl bodyCtrl;
FilesCtrl filesCtrl;
ScreenCtrl screenCtrl;
Wireless wireless;

int ServoMiddlePWM[12];
int jointsCurrentPos[12];
void jsonCmdReceiveHandler(const JsonDocument& jsonCmdInput);
void runMission(String missionName, int intervalTime, int loopTimes);
void buzzerCtrl(int freq, int duration);
bool steadyMode = 0; // steady mode
unsigned long previousMillisFB = 0;
const unsigned long intervalFB = 5000;

// Create AsyncWebServer object on port 80
WebServer server(80);

void handleRoot(){
  server.send(200, "text/html", index_html); //Send web page
}

void webCtrlServer(){
  server.on("/", handleRoot);

  server.on("/js", [](){
    String jsonCmdWebString = server.arg(0);
    deserializeJson(jsonCmdReceive, jsonCmdWebString);
    jsonCmdReceiveHandler(jsonCmdReceive);
    serializeJson(jsonFeedback, outputString);
    Serial.println(jsonCmdWebString);
    server.send(200, "text/plane", outputString);
    outputString = "";
    jsonFeedback.clear();
    jsonCmdReceive.clear();
  });

  // Start server
  server.begin();
  Serial.println("Server Starts.");
}

void setup() {
  delay(1000);
  Serial.begin(BAUD_RATE);
  Serial.println("device starting...");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  screenCtrl.init();
  screenCtrl.displayText("WAVESHARE", 0, 0, 2);
  screenCtrl.displayText("Robotics", 0, 16, 2);

  InitICM20948();
  InitINA219();

  
  led.init();
  led.setColor(0, 255, 0, 64);
  led.setColor(1, 64, 0, 255);

  filesCtrl.init();
  
  bodyCtrl.init();

  /* 
  <<<<<<<<<<=========Wire Debug Init=========>>>>>>>>>
  [SHOW] DebugMode via wire config.
           [ . . . o o ]  LED G21 G15 G12 3V3
           [ . . . . . ]  TX  RX  GND  5V  5V
              <SWITCH>
  connect this two pins, and the robot go into debug mode.
  */
  pinMode(DEBUG_PIN, INPUT_PULLDOWN);
  
  pinMode(BUZZER_PIN, OUTPUT);
  buzzerCtrl(2000, 50);

  if(!filesCtrl.checkMission("boot")) {
    filesCtrl.createMission("boot", "this is the boot mission.");
    bodyCtrl.jointMiddle();
    filesCtrl.appendStep("boot", "{\"T\":400,\"mode\":1,\"ap_ssid\":\"WAVEGO\",\"ap_password\":\"12345678\",\"channel\":1,\"sta_ssid\":\"\",\"sta_password\":\"\"}");
  } else {
    if (filesCtrl.checkStepByType("boot", CMD_SET_JOINTS_ZERO)) {
      Serial.println("Already set joints zero pos.");
    } else {
      bodyCtrl.jointMiddle();
      Serial.println("Haven't set joints zero pos yet.");
    }
    if (filesCtrl.checkStepByType("boot", CMD_SET_WIFI_MODE)) {
      Serial.println("Already set wifi mode.");
    } else {
      filesCtrl.appendStep("boot", "{\"T\":400,\"mode\":1,\"ap_ssid\":\"WAVEGO\",\"ap_password\":\"12345678\",\"channel\":1,\"sta_ssid\":\"\",\"sta_password\":\"\"}");
      Serial.println("Haven't set wifi mode yet.");
    }
  }
  runMission("boot", 0, 1);

  wifi_mode_t mode;
  esp_err_t err = esp_wifi_get_mode(&mode);
  if (err != ESP_ERR_WIFI_NOT_INIT) {
    wireless.espnowInit(false);
    wireless.setJsonCommandCallback(jsonCmdReceiveHandler);
  }

  // wireless.setAP("WAVEGO", "12345678", 1);

  webCtrlServer();


  // ServoMiddlePWM[0] = 368;
  // ServoMiddlePWM[1] = 623;
  // ServoMiddlePWM[2] = 521;
  // ServoMiddlePWM[3] = 495;

  // ServoMiddlePWM[4] = 364;
  // ServoMiddlePWM[5] = 672;
  // ServoMiddlePWM[6] = 665;
  // ServoMiddlePWM[7] = 417;

  // ServoMiddlePWM[8] = 497;
  // ServoMiddlePWM[9] = 529;
  // ServoMiddlePWM[10] = 654;
  // ServoMiddlePWM[11] = 374;

  // // {"T":105,"set":[368,623,521,495,364,672,665,417,497,529,654,374]}

  // bodyCtrl.setJointsZeroPosArray(ServoMiddlePWM);
}


void buzzerCtrl(int freq, int duration) {
  tone(BUZZER_PIN, freq);
  delay(duration);
  noTone(BUZZER_PIN);
  digitalWrite(BUZZER_PIN, HIGH);
}


void wireDebugDetect(){
  if(digitalRead(DEBUG_PIN) == HIGH){
    bodyCtrl.jointMiddle();

    led.setColor(0, 255, 64, 0);
    led.setColor(1, 255, 64, 0);

    while(digitalRead(DEBUG_PIN) == HIGH){
      delay(100);
    }
    delay(1000);
    led.setColor(0, 255, 0, 64);
    led.setColor(1, 64, 0, 255);
  }
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
  case CMD_UGV_CTRL:
                        bodyCtrl.ugvCtrl(jsonCmdInput["L"],
                                         jsonCmdInput["R"]);
                        break;
  case CMD_PT_CTRL:
                        bodyCtrl.ptCtrl(jsonCmdInput["X"],
                                        jsonCmdInput["Y"]);
                        break;

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
                        memcpy(ServoMiddlePWM, bodyCtrl.getJointsZeroPosArray(), sizeof(ServoMiddlePWM));
                        for (int i = 0; i < 12; i++) {  
                          Serial.print("Joint ");
                          Serial.print(i);
                          Serial.print(": ");
                          Serial.println(ServoMiddlePWM[i]);
                        }
                        break;
  case CMD_SET_JOINTS_ZERO:
                        for (int i = 0; i < 12; i++) {
                          ServoMiddlePWM[i] = jsonCmdInput["set"][i];
                        }
                        bodyCtrl.setJointsZeroPosArray(ServoMiddlePWM);
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
                        memcpy(ServoMiddlePWM, bodyCtrl.getJointsZeroPosArray(), sizeof(ServoMiddlePWM));
                        jsonFeedback.clear();
                        jsonFeedback["T"] = CMD_SET_JOINTS_ZERO;
                        for (int i = 0; i < 12; i++) {  
                          Serial.print("Joint ");
                          Serial.print(i);
                          Serial.print(": ");
                          Serial.println(ServoMiddlePWM[i]);
                          jsonFeedback["set"][i] = ServoMiddlePWM[i];
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
  case CMD_STAND_UP:
                        bodyCtrl.stand();
                        break;
  case CMD_BASIC_MOVE:
                        bodyCtrl.inputCmd(jsonCmdInput["FB"], jsonCmdInput["LR"]);
                        break;
  case CMD_BASIC_FUNC:  
                        if (jsonCmdInput["func"] == 1) {
                          bodyCtrl.functionStayLow();
                        } else if (jsonCmdInput["func"] == 2) {
                          bodyCtrl.functionHandshake();
                        } else if (jsonCmdInput["func"] == 3) {
                          bodyCtrl.functionJump();
                        } else if (jsonCmdInput["func"] == 4) {
                          steadyMode = 1;
                        } else if (jsonCmdInput["func"] == 5) {
                          steadyMode = 0;
                          bodyCtrl.standUp(95);
                        }
                        break;



  case CMD_SINGLE_LEG_CTRL:
                        bodyCtrl.singleLegCtrl(jsonCmdInput["leg"], 
                                               jsonCmdInput["x"], 
                                               jsonCmdInput["y"], 
                                               jsonCmdInput["z"]);
                        bodyCtrl.moveTrigger();
                        break;
  case CMD_STAND_UP_HEIGHT:
                        bodyCtrl.standUp(jsonCmdInput["h"]);
                        break;
  case CMD_SET_INTERPOLATION_PARAMS:
                        bodyCtrl.setInterpolationParams(jsonCmdInput["delay"], 
                                                        jsonCmdInput["iterate"]);
                        break;
  case CMD_SET_GAIT_PARAMS:
                        bodyCtrl.setGaitParams(jsonCmdInput["maxHeight"],
                                               jsonCmdInput["minHeight"], 
                                               jsonCmdInput["height"],
                                               jsonCmdInput["lift"], 
                                               jsonCmdInput["range"], 
                                               jsonCmdInput["acc"], 
                                               jsonCmdInput["extendedX"], 
                                               jsonCmdInput["extendedZ"], 
                                               jsonCmdInput["sideMax"], 
                                               jsonCmdInput["massAdjust"]);
                        break;



	case CMD_SET_COLOR: 
                        led.setColor(jsonCmdInput["set"][0], 
                                     jsonCmdInput["set"][1], 
                                     jsonCmdInput["set"][2], 
                                     jsonCmdInput["set"][3]);
												break;
  case CMD_DISPLAY_SINGLE:
                        screenCtrl.changeSingleLine(jsonCmdInput["line"], 
                                                    jsonCmdInput["text"], 
                                                    jsonCmdInput["update"]);
                        break;
  case CMD_DISPLAY_UPDATE:
                        screenCtrl.updateFrame();
                        break;
                        
  case CMD_DISPLAY_FRAME:
                        screenCtrl.changeSingleLine(1, jsonCmdInput["l1"], false);
                        screenCtrl.changeSingleLine(2, jsonCmdInput["l2"], false);
                        screenCtrl.changeSingleLine(3, jsonCmdInput["l3"], false);
                        screenCtrl.changeSingleLine(4, jsonCmdInput["l4"], true);
                        break;
  case CMD_DISPLAY_CLEAR:
                        screenCtrl.clearDisplay();
                        break;
  case CMD_BUZZER_CTRL:
                        tone(BUZZER_PIN, jsonCmdInput["freq"]);
                        buzzerCtrl(jsonCmdInput["freq"], jsonCmdInput["duration"]);
                        break;
  case CMD_GET_BATTERY_VOLTAGE:
                        InaDataUpdate();
                        jsonFeedback.clear();
                        jsonFeedback["T"] = - CMD_GET_BATTERY_VOLTAGE;
                        jsonFeedback["voltage"] = loadVoltage_V;
                        serializeJson(jsonFeedback, outputString);
                        Serial.println(outputString);
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

                        

  case CMD_SET_WIFI_MODE:
                        // Serial.println("lickLWFeetB");
                        if (jsonCmdInput["mode"] == 0) {
                          wireless.setWifiMode(jsonCmdInput["mode"], "", "", 0, "", "");
                          jsonFeedback.clear();
                          jsonFeedback["T"] = CMD_SET_WIFI_MODE;
                          jsonFeedback["mode"] = 0;
                          serializeJson(jsonFeedback, outputString);
                          filesCtrl.checkReplaceStep("boot", outputString);
                        } else if (jsonCmdInput["mode"] == 1) {
                          if (wireless.setWifiMode(jsonCmdInput["mode"], 
                                                   jsonCmdInput["ap_ssid"], 
                                                   jsonCmdInput["ap_password"], 
                                                   jsonCmdInput["channel"], 
                                                   jsonCmdInput["sta_ssid"], 
                                                   jsonCmdInput["sta_password"])) {
                              jsonFeedback.clear();
                              jsonFeedback["T"] = CMD_SET_WIFI_MODE;
                              jsonFeedback["mode"] = 1;
                              jsonFeedback["ap_ssid"] = jsonCmdInput["ap_ssid"];
                              jsonFeedback["ap_password"] = jsonCmdInput["ap_password"];
                              jsonFeedback["channel"] = jsonCmdInput["channel"];
                              jsonFeedback["sta_ssid"] = jsonCmdInput["sta_ssid"];
                              jsonFeedback["sta_password"] = jsonCmdInput["sta_password"];
                              serializeJson(jsonFeedback, outputString);
                              filesCtrl.checkReplaceStep("boot", outputString);
                          } else {
                              jsonFeedback.clear();
                              jsonFeedback["T"] = CMD_SET_WIFI_MODE;
                              jsonFeedback["mode"] = 1;
                              jsonFeedback["ap_ssid"] = jsonCmdInput["ap_ssid"];
                              jsonFeedback["ap_password"] = jsonCmdInput["ap_password"];
                              jsonFeedback["channel"] = jsonCmdInput["channel"];
                              jsonFeedback["sta_ssid"] = "";
                              jsonFeedback["sta_password"] = "";
                              serializeJson(jsonFeedback, outputString);
                              filesCtrl.checkReplaceStep("boot", outputString);
                          }
                        }
                        break;
  case CMD_WIFI_INFO:
                        outputString = filesCtrl.findCmdByType("boot", CMD_SET_WIFI_MODE);
                        Serial.println(outputString);
                        break;
  case CMD_GET_AP_IP:
                        outputString = wireless.getAPIP();
                        Serial.println(outputString);
                        break;
  case CMD_GET_STA_IP:
                        outputString = wireless.getSTAIP();
                        Serial.println(outputString);
                        break;



  case CMD_INIT_ESP_NOW:
                        wireless.espnowInit(jsonCmdInput["longrange"]);
                        break;
  case CMD_SET_ESP_NOW_MODE:
                        wireless.setEspNowMode(jsonCmdInput["mode"]);
                        break;
  case CMD_GET_MAC:
                        outputString = wireless.macToString(wireless.getMac());
                        Serial.println(outputString);
                        break;
  case CMD_ESP_NOW_SEND:
                        wireless.sendEspNow(jsonCmdInput["mac"], jsonCmdInput["data"]);
                        break;
  case CMD_ADD_MAC:
                        wireless.addMacToPeerString(jsonCmdInput["mac"]);
                        break;


  case ESP32_REBOOT:
                        ESP.restart();
                        break;
  case CMD_CLEAR_NVS:
                        nvs_flash_erase();
                        delay(1000);
                        nvs_flash_init();
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
  server.handleClient();

  if (steadyMode == 1) {
    accXYZUpdate();
    bodyCtrl.balancing(ACC_X, ACC_Y);
  } else {
    bodyCtrl.robotCtrl();
  }
  wireDebugDetect();

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillisFB >= intervalFB) {
    previousMillisFB = currentMillis;
    InaDataUpdate();
    jsonFeedback.clear();
    jsonFeedback["T"] = 1001;
    jsonFeedback["L"] = 0;
    jsonFeedback["R"] = 0;
    jsonFeedback["r"] = 0;
    jsonFeedback["p"] = 0;
    jsonFeedback["v"] = loadVoltage_V;
    jsonFeedback["pan"] = 0;
    jsonFeedback["tilt"] = 0;
    serializeJson(jsonFeedback, outputString);
    Serial.println(outputString);
  }

  // accXYZUpdate();
  // Serial.print("ACC_X: ");Serial.println(ACC_X);
  // Serial.print("ACC_Y: ");Serial.println(ACC_Y);

  // bodyCtrl.singleLegCtrl(1, WALK_EXTENDED_X, cmdInput, WALK_EXTENDED_Z);

  // bodyCtrl.standUp(95);
  // Serial.println("Current pos: ");
  // bodyCtrl.massCenerAdjustTestLoop();
}