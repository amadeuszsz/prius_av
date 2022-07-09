// Copyright 2020 Amadeusz Szymko <amadeuszszymko@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "prius_control/control.h"


PriusControl::PriusControl() : m_nh()
{
    m_joy_sub = m_nh.subscribe("/joy", 1, &PriusControl::joy_callback, this);
    m_control_pub = m_nh.advertise<prius_msgs::Control>("/prius", 1);
}

void PriusControl::joy_callback(const sensor_msgs::Joy::ConstPtr &msg)
{
    // CASE: autonomous mode AND manual mode button is not pressed -> skip joy callback
    if (m_mode_driving != prius_msgs::Control::STATE_DRIVING_MANUAL and
        msg->axes[prius_msgs::Control::AXIS_MODE_DRIVING] != -1)
        return;

    prius_msgs::Control control;
    control.header = msg->header;
    //////////////////// Mode ////////////////////
    // Set driving mode
    if (msg->axes[prius_msgs::Control::AXIS_MODE_DRIVING] == -1)
        m_mode_driving = prius_msgs::Control::STATE_DRIVING_MANUAL;
    if (msg->axes[prius_msgs::Control::AXIS_MODE_DRIVING] == 1)
        m_mode_driving = prius_msgs::Control::STATE_DRIVING_AUTO;
    control.mode_driving = m_mode_driving;

    // Set collection mode
    if (msg->axes[prius_msgs::Control::AXIS_MODE_COLLECTION] == 1)
        m_mode_collection = prius_msgs::Control::STATE_COLLECTION_CAMERA;
    if (msg->axes[prius_msgs::Control::AXIS_MODE_COLLECTION] == -1)
        m_mode_collection = prius_msgs::Control::STATE_COLLECTION_SIGN;
    control.mode_collection = m_mode_collection;

    // Turn on/off collecting data
    if (msg->buttons[prius_msgs::Control::COLLECTION_OFF])
        m_collect = false;
    if (msg->buttons[prius_msgs::Control::COLLECTION_ON])
        m_collect = true;
    control.collect = m_collect;

    //////////////////// Control ////////////////////
    // Throttle & Brake axis
    double acceleration = (msg->axes[prius_msgs::Control::AXIS_BRAKE] -
            msg->axes[prius_msgs::Control::AXIS_THROTTLE]) / 2.0;
    if (acceleration >= 0.0) {
        control.throttle = acceleration;
        control.brake = 0.0;
    }
    if (acceleration < 0.0) {
        control.brake = -acceleration;
        control.throttle = 0.0;
    }

    // Steering axis
    control.steer = msg->axes[prius_msgs::Control::AXIS_STEERING];

    // Gears
    if (msg->buttons[prius_msgs::Control::SHIFT_REVERSE])
        control.shift_gears = prius_msgs::Control::REVERSE;
    else if (msg->buttons[prius_msgs::Control::SHIFT_NEUTRAL])
        control.shift_gears = prius_msgs::Control::NEUTRAL;
    else if (msg->buttons[prius_msgs::Control::SHIFT_FORWARD])
        control.shift_gears = prius_msgs::Control::FORWARD;
    else
        control.shift_gears = prius_msgs::Control::NO_COMMAND;

    m_control_pub.publish(control);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "prius_control_node");
    PriusControl d;
    ros::spin();
    return 0;
}
