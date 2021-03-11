/**
Software License Agreement (BSD)

\file      heron_constants.h
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#ifndef HERON_CONSTANTS_H

//TODO: Change defines to static const and enums
//Boat Constants
#define BOAT_WIDTH 0.7366 //m, ~15inches
#define MAX_FWD_THRUST 45.0 //Newtons
#define MAX_BCK_THRUST 25.0 //Newtons
#define MAX_FWD_VEL 2 //m/s
#define MAX_BCK_VEL 0.5 //m/s
#define MAX_OUTPUT 1
#define MAX_YAW_RATE 0.5 //rad/s

//Control Modes
#define NO_CONTROL 0
#define WRENCH_CONTROL 1
#define HELM_CONTROL 2
#define COURSE_CONTROL 3
#define TWIST_CONTROL 4
#define TWIST_LIN_CONTROL 5

#define PI 3.14159

#endif
