#ifndef CONST_DEFINES_H
#define CONST_DEFINES_H

#define PORT_VIDEO 3490
#define PORT_COMMAND 3491
#define PORT_TELEMETRY 3492

#define IMAGE_W 640
#define IMAGE_H 480

//For sending commands to robot
#define COM_MASK 248
#define STOP 0
#define GO 1
#define HEADING_LEFT 2
#define HEADING_RIGHT 3
#define ROTATE_LEFT 4
#define ROTATE_RIGHT 5
#define SEND_INFO 7
#define MAX_SPEED 200


//defines for arduino command codes
#define SEND_YAW 15
#define SEND_PITCH 23
#define RESET_YAW 31
#define INCREASE_PID_P 39
#define INCREASE_PID_I 47
#define INCREASE_PID_D 55
#define DECREASE_PID_P 63
#define DECREASE_PID_I 71
#define DECREASE_PID_D 79
#define INCREASE_BALANCE 87
#define DECREASE_BALANCE 95
#define SET_BAL_NONE 103
#define SET_BAL_RESTING 111
#define SET_BAL_ACTIVE 119
#define SEND_SETTINGS 127
#define RESET_ARDUINO 135

//codes for arduino sending to the Pi
#define REC_PID_P 131
#define REC_PITCH 130
#define REC_YAW 129
#define REC_PID_I 132
#define REC_PID_D 133
#define REC_BAL_SET 144
#define REC_BAL_MODE 145
#define SET_CONTROLLER_IP 254
#define REC_ERROR 255
//INCREMENTS for settables
//PID ranges for balance
//P 0 - 198
//I 0 - 198
//D 0 - 99
#define RANGE_PID_P 2
#define RANGE_PID_I 2
#define RANGE_PID_D 1
#define BAL_MOVE 0.5 // amount balance gets moved in degrees

#define BAL_MODE_RESTING 0
#define BAL_MODE_NONE 1
#define BAL_MODE_ACTIVE 2

//for pi
#define BLOCK_SIZE 640
#define NUM_OF_TRIES 10
#define UPDATE_TIME 10
#endif // CONST_DEFINES_H
