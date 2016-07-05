

#ifndef HERON_CONSTANTS_H

//TODO: Change defines to static const and enums
//Boat Constants
#define BOAT_WIDTH 0.381 //m, ~15inches
#define MAX_FWD_THRUST 35.0 //Newtons
#define MAX_BCK_THRUST 20.0 //Newtons
#define MAX_FWD_VEL 4 //m/s
#define MAX_BCK_VEL 0.5 //m/s
#define MAX_OUTPUT 1
#define MAX_YAW_RATE 0.5 //rad/s

//Control Modes
#define NO_CONTROL 0
#define WRENCH_CONTROL 1
#define YAW_RATE_CONTROL 2
#define YAW_CONTROL 3

#define PI 3.14159

#endif
