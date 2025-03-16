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
static int InfoPrint = 1;

// serial baud rate
#define BAUD_RATE 921600



// --- Command Configuration ---
// Body Ctrl
// {"T":101}
#define CMD_JOINT_MIDDLE 101
// Release Torque
// {"T":102}
#define CMD_RELEASE_TORQUE 102
// single servo ctrl
// {"T":103,"id":1,"goal":512,"time":0,"spd":0}
#define CMD_SINGLE_SERVO_CTRL 103
// get joints zero pos array
// {"T":104}
#define CMD_GET_JOINTS_ZERO 104
// set joints zero pos array
// {"T":105,"set":[512,512,512,512,512,512,512,512,512,512,512,512]}
#define CMD_SET_JOINTS_ZERO 105
// get the current pos of all servos
// {"T":106}
// feedback: {"T":-106,"fb":[512,512,512,512,512,512,512,512,512,512,512,512]}
#define CMD_GET_CURRENT_POS 106
// set the current pos as the zero pos
// {"T":107}
#define CMD_SET_CURRENT_POS_ZERO 107



// JSON cmds: [T:201, set:[id, r, g, b]]
// {"T":201,"set":[0,9,0,0]}
#define CMD_SET_COLOR 201


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
#define CMD_INSERT_SETP_JSON 304
#define CMD_REPLACE_SETP_JSON 305
#define CMD_DELETE_SETP 306
#define CMD_MOVE_TO_STEP 307
#define CMD_RUN_MISSION 308
// format flash (clear all)
// {"T":399}
#define CMD_FORMAT_FLASH 399