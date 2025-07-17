// --- PIN Configuration ---
// config for the RGB LED
#define RGB_PIN 26
#define RGB_NUM 2

// config for the bus servo
#define BUS_SERVO_RX 18
#define BUS_SERVO_TX 19

// pin for I2C
#define I2C_SDA 32
#define I2C_SCL 33

// pin for the buzzer
#define BUZZER_PIN 21

// pin for the debug
#define DEBUG_PIN 12



// --- Debug Configuration ---
// 1: print debug info
// 0: do not print debug info
static int InfoPrint = 0;

// serial baud rate
#define BAUD_RATE 115200

// processing time adjustment
static int timeOffset = 50;



// --- Command Configuration ---
// UGV ctrl interface
// {"T":1,"L":0,"R":0}
#define CMD_UGV_CTRL 1
// PT interface
// {"T":133,"X":0,"Y":0}
#define CMD_PT_CTRL 133

// Body Ctrl
// {"T":101}
#define CMD_JOINT_MIDDLE 101
// Release Torque
// {"T":102}
#define CMD_RELEASE_TORQUE 102
// single servo ctrl
// {"T":103,"id":1,"goal":511,"time":0,"spd":0}
#define CMD_SINGLE_SERVO_CTRL 103
// get joints zero pos array - [Coupling function]
// {"T":104}
#define CMD_GET_JOINTS_ZERO 104
// set joints zero pos array - [Coupling function]
// {"T":105,"set":[511,511,511,511,511,511,511,511,511,511,511,511]}
#define CMD_SET_JOINTS_ZERO 105
// get the current pos of all servos - [Coupling function]
// {"T":106}
// feedback: {"T":-106,"fb":[511,511,511,511,511,511,511,511,511,511,511,511]}
#define CMD_GET_CURRENT_POS 106
// set the current pos as the zero pos
// {"T":107}
#define CMD_SET_CURRENT_POS_ZERO 107

// ctrl single joint angle
// {"T":108,"joint":1,"angle":45}
#define CMD_CTRL_JOINT_ANGLE 108
// ctrl single joint radian
// {"T":109,"joint":1,"rad":0.785}
#define CMD_CTRL_JOINT_RAD 109
// stand up
// {"T":110}
#define CMD_STAND_UP 110

// basic movtion ctrl
// {"T":111,"FB":0,"LR":0}
#define CMD_BASIC_MOVE 111
// basic functions
// 1 -> stayLow
// 2 -> handShake
// 3 -> jump
// 4 -> steady on
// 5 -> steady off
// {"T":112,"func":2}
#define CMD_BASIC_FUNC 112

// test
// single leg ctrl
// {"T":113,"leg":1,"x":16,"y":90,"z":25}
#define CMD_SINGLE_LEG_CTRL 113
// stand up height ctrl
// {"T":114,"h":95}
#define CMD_STAND_UP_HEIGHT 114
// set interpolaytion params
// {"T":115,"delay":5,"iterate":0.02}
#define CMD_SET_INTERPOLATION_PARAMS 115
// set gait params
// {"T":116,"maxHeight":110,"minHeight":75,"height":95,"lift":9,"range":40,"acc":5,"extendedX":16,"extendedZ":25,"sideMax":30,"massAdjust":21}
#define CMD_SET_GAIT_PARAMS 116


// id 0 -> left LED
// id 1 -> right LED
// JSON cmds: [T:201, set:[id, r, g, b]]
// {"T":201,"set":[0,9,0,0]}
#define CMD_SET_COLOR 201
// ctrl oled display a single line
// updateFlag = 0 -> no update, updateFlag = 1 -> update
// {"T":202,"line":1,"text":"Hello, world!","update":1}
#define CMD_DISPLAY_SINGLE 202
// oled display update
// {"T":203}
#define CMD_DISPLAY_UPDATE 203
// ctrl oled display a frame
// {"T":204,"l1":"Hello, world!","l2":"Hello, world!","l3":"Hello, world!","l4":"Hello, world!"}
#define CMD_DISPLAY_FRAME 204
// oled display clear
// {"T":205}
#define CMD_DISPLAY_CLEAR 205
// buzzer ctrl
// {"T":206,"freq":1000,"duration":10}
#define CMD_BUZZER_CTRL 206
// get battery voltage
// {"T":207}
#define CMD_GET_BATTERY_VOLTAGE 207

// scan files
// {"T":300}
#define CMD_SCAN_FILES 300
// create mission
// {"T":301,"name":"mission1","intro":"introduction for this mission file."}
#define CMD_CREATE_MISSION 301
// show the content of the mission
// {"T":302,"name":"boot"}
#define CMD_MISSION_CONTENT 302
// append a new json cmd to the mission file
// {"T":303,"name":"boot","json":"{\"T\":201,\"set\":[0,9,0,0]}"}
#define CMD_APPEND_SETP_JSON 303
// insert a new json cmd to the mission file under the stepNum
// {"T":304,"name":"boot","step":2,"json":"{\"T\":201,\"set\":[0,9,0,0]}"}
#define CMD_INSERT_SETP_JSON 304
// replace a new json cmd to the mission file under the stepNum
// {"T":305,"name":"boot","step":2,"json":"{\"T\":201,\"set\":[0,9,0,0]}"}
#define CMD_REPLACE_SETP_JSON 305
// delete a step from the mission file
// {"T":306,"name":"boot","step":2}
#define CMD_DELETE_SETP 306
// run a single step - [Coupling function]
// {"T":307,"name":"boot","step":2}
#define CMD_RUN_STEP 307
// run the whole mission - [Coupling function]
// {"T":308,"name":"boot","interval":1000,"loop":1}
#define CMD_RUN_MISSION 308
// delete mission
// {"T":309,"name":"boot"}
#define CMD_DELETE_MISSION 309
// format flash (clear all)
// {"T":399}
#define CMD_FORMAT_FLASH 399

// --- Wireless Configuration ---
// 0 -> none, 1[default] -> AP+STA
// {"T":400,"mode":1,"ap_ssid":"LYgion","ap_password":"12345678","channel":1,"sta_ssid":"ssid","sta_password":"password"}
#define CMD_SET_WIFI_MODE 400
// {"T":401}
#define CMD_WIFI_INFO 401
// {"T":402}
#define CMD_GET_AP_IP 402
// {"T":403}
#define CMD_GET_STA_IP 403

// {"T":410,"longrange":0}
#define CMD_INIT_ESP_NOW 410
// 0 -> no esp-now receive, 1 -> start esp-now[default]
// {"T":411,"mode":1}
#define CMD_SET_ESP_NOW_MODE 411
// {"T":412}
#define CMD_GET_MAC 412
// {"T":413,"mac":"FF,FF,FF,FF,FF,FF","data":"{\"T\":205,\"freq\":500,\"duration\":30}"}
#define CMD_ESP_NOW_SEND 413
// {"T":414,"mac":"FF:FF:FF:FF:FF:FF"}
#define CMD_ADD_MAC 414


// --- System Configuration ---
// the time for ESP32 to reboot after receiving a reboot command
// {"T":600}
#define ESP32_REBOOT 600
// clear nvs
// {"T":601}
#define CMD_CLEAR_NVS 601