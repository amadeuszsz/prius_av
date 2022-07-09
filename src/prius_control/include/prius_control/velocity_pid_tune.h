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

#ifndef PRIUS_CONTROL_VELOCITY_PID_TUNE_H
#define PRIUS_CONTROL_VELOCITY_PID_TUNE_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include "prius_control/pid.h"
#include "prius_msgs/Control.h"
#include "prius_control/pidConfig.h"


class PriusVelocityPIDTune {
private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_odom_sub;
    ros::Publisher m_control_pub;
    ros::Publisher m_control_effort_pub;
    ros::Publisher m_setpoint_pub;
    ros::Publisher m_state_pub;
    ros::Timer m_setpoint_timer;
    dynamic_reconfigure::Server<prius_control::pidConfig> m_server{};
    dynamic_reconfigure::Server<prius_control::pidConfig>::CallbackType m_f{};
    PID m_pid;
    std_msgs::Float64 m_setpoint;
    double m_setpoint_first{}, m_setpoint_second{};
    double m_Kp{}, m_Ki{}, m_Kd{};
    ros::Rate m_rate;

public:
    PriusVelocityPIDTune();
    // Connect with reconfigure GUI
    void reconfigure_callback(prius_control::pidConfig &config, uint32_t level);
    // Get current vehicle velocity
    void odom_callback(const nav_msgs::OdometryConstPtr& msg);
    // Set goal velocity
    void setpoint_callback();
};

#endif //PRIUS_CONTROL_VELOCITY_PID_TUNE_H
