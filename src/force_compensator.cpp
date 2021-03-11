/**
Software License Agreement (BSD)

\file      force_compensator.cpp
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
#include "heron_controller/force_compensator.h"


ForceCompensator::ForceCompensator(ros::NodeHandle &n): node_(n) {
            cmd_pub_ = node_.advertise<heron_msgs::Drive>("cmd_drive",1000);
            eff_pub_ = node_.advertise<geometry_msgs::Wrench>("eff_wrench",1000);
}

// Take in a thrust requirement and return the electronic input (into the motor controller) required to achieve given thrust.
//TODO:Once better thruster models are known, this should be changed to something more complex than just a linear scaling.
double ForceCompensator::calculate_motor_setting (double thrust) {
    //saturate

    double output = 0;
    if (thrust > 0)
        output = thrust * (MAX_OUTPUT/MAX_FWD_THRUST);
    else if (thrust < 0)
        output = thrust * (MAX_OUTPUT/MAX_BCK_THRUST);
    return output;
}

double ForceCompensator::saturate_thrusters (double thrust) {
    thrust = std::min(MAX_FWD_THRUST,thrust);
    thrust = std::max(-1*MAX_BCK_THRUST,thrust);
    return thrust;
}


//Take in wrench command and output cmd_drive messages to achieve the given wrench command
void ForceCompensator::pub_thrust_cmd (geometry_msgs::Wrench output) {
    heron_msgs::Drive cmd_output;
    double fx = output.force.x;
    double tauz = output.torque.z;


    //yaw torque maxed out at max torque achievable with the help of reverse thrust
    double max_tauz = MAX_BCK_THRUST*2*BOAT_WIDTH;
    tauz = std::min(tauz, max_tauz);
    tauz = std::max(tauz, -max_tauz);

    //Guarantee atleast max yaw torque
    double left_thrust = -tauz/(2*BOAT_WIDTH);
    double right_thrust = tauz/(2*BOAT_WIDTH);

    //Provide maximum allowable thrust after yaw torque is guaranteed
    double max_fx = 0;
    if (tauz >= 0) {
        if (fx >= 0) { //forward thrust on the left thruster will be limiting factor
            max_fx = (MAX_FWD_THRUST - left_thrust) * 2;
            fx = std::min(max_fx,fx);
        }
        else { //backward thrust on the right thruster will be limiting factor
            max_fx = (-MAX_BCK_THRUST - right_thrust) * 2;
            fx = std::max(max_fx,fx);
        }
    }
    else {
        if (fx >= 0 ) {
            max_fx = (MAX_FWD_THRUST - right_thrust) * 2;
            fx = std::min(max_fx,fx);
        }
        else {
            max_fx = (-MAX_BCK_THRUST - left_thrust) * 2;
            fx = std::max(max_fx,fx);
        }
    }

    left_thrust += fx/2.0;
    right_thrust += fx/2.0;

    left_thrust = saturate_thrusters (left_thrust);
    right_thrust = saturate_thrusters (right_thrust);

    cmd_output.left = calculate_motor_setting (left_thrust);
    cmd_output.right = calculate_motor_setting (right_thrust);
    cmd_pub_.publish(cmd_output);

    pub_effective_wrench(left_thrust, right_thrust);
}


//take in left and right thrusts (0...1) settings and back calculate what the effective wrench being sent out is. This is a reverse calculation of what is done in "update_forces" and shows the user what the limitations of the thrust settings are.
void ForceCompensator::pub_effective_wrench(double left_thrust,double right_thrust) {
    geometry_msgs::Wrench effective_output;
    effective_output.force.x = left_thrust + right_thrust;
    effective_output.torque.z = (right_thrust - left_thrust)*BOAT_WIDTH;
    eff_pub_.publish(effective_output);
}
