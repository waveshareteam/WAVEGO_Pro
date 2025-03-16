// --- PIN Configuration ---
// config for the RGB LED
#define RGB_PIN 26
#define RGB_NUM 2

// pin for the bus servo
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


// JSON cmds: [T:201, set:[id, r, g, b]]
// {"T":201,"set":[0,9,0,0]}
#define CMD_SET_COLOR 201