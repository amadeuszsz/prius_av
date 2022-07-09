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

#ifndef PRIUS_CONTROL_VELOCITY_PID_H
#define PRIUS_CONTROL_VELOCITY_PID_H

#include <ros/ros.h>
#include "prius_control/pid.h"
#include <std_msgs/UInt8.h>
#include "prius_msgs/Control.h"
#include "prius_msgs/State.h"


class PriusVelocityPID {
private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_control_sub;
    ros::Subscriber m_state_sub;
    ros::Subscriber m_sign_sub;
    ros::Subscriber m_predictions_sub;
    ros::Publisher m_control_pub;
    ros::Timer m_timer;
    PID m_pid;
    double m_velocity_current = 0.0;
    double m_velocity_predicted = 0.0;
    double m_steer_predicted = 0.0;
    double m_kp, m_ki, m_kd;
    uint8_t m_mode_driving = prius_msgs::Control::STATE_DRIVING_MANUAL;
    uint8_t m_mode_collection = prius_msgs::Control::STATE_COLLECTION_CAMERA;
    bool m_collect = false;

public:
    PriusVelocityPID();
    void control_callback(const prius_msgs::Control::ConstPtr& msg);
    void state_callback(const prius_msgs::State::ConstPtr& msg);
    void predictions_callback(const prius_msgs::State::ConstPtr& msg);
    // Pass predicted steering signal and predicted velocity as throttle/brake signal
    void control_publisher();
};

#endif //PRIUS_CONTROL_VELOCITY_PID_H
