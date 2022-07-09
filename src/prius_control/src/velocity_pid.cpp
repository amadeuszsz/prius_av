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

#include "prius_control/velocity_pid.h"


PriusVelocityPID::PriusVelocityPID() : m_nh(),
m_pid(0.01, 1.0, -1.0, 0.0, 0.0, 0.0)
{
    m_nh.getParam("/prius_velocity_pid_node/kp", m_kp);
    m_nh.getParam("/prius_velocity_pid_node/ki", m_ki);
    m_nh.getParam("/prius_velocity_pid_node/kd", m_kd);
    m_pid.set_gains(m_kp, m_ki, m_kd);
    m_control_sub = m_nh.subscribe("/prius", 1, &PriusVelocityPID::control_callback, this);
    m_state_sub = m_nh.subscribe("/prius/state", 1, &PriusVelocityPID::state_callback, this);
    m_predictions_sub = m_nh.subscribe("/prius/predictions", 1, &PriusVelocityPID::predictions_callback, this);
    m_control_pub = m_nh.advertise<prius_msgs::Control>("/prius", 1);
    m_timer = m_nh.createTimer(ros::Duration(0.02), std::bind(&PriusVelocityPID::control_publisher, this));
}

void PriusVelocityPID::control_callback(const prius_msgs::Control::ConstPtr &msg)
{
    m_mode_driving = msg->mode_driving;
    m_mode_collection = msg->mode_collection;
    m_collect = msg->collect;
}

void PriusVelocityPID::state_callback(const prius_msgs::State::ConstPtr& msg)
{
    m_velocity_current = msg->velocity;
}

void PriusVelocityPID::predictions_callback(const prius_msgs::State::ConstPtr &msg)
{
    m_velocity_predicted = msg->velocity;
    m_steer_predicted = msg->steer;
}

void PriusVelocityPID::control_publisher()
{
    if (m_mode_driving == prius_msgs::Control::STATE_DRIVING_AUTO)
    {
        double velocity, steer;
        velocity = m_velocity_predicted;
        steer = m_steer_predicted;
        
        prius_msgs::Control control;
        control.header.stamp = ros::Time::now();
        control.header.frame_id = "pid";

        double control_effort = m_pid.calculate(velocity, m_velocity_current);
        if (control_effort >= 0.0)
        {
            control.throttle = control_effort;
            control.brake = 0.0;
        }
        if (control_effort < 0.0)
        {
            control.brake = -control_effort;
            control.throttle = 0.0;
        }
        control.steer = steer;
        control.shift_gears = prius_msgs::Control::FORWARD;
        control.mode_driving = m_mode_driving;
        control.mode_collection = m_mode_collection;
        control.collect = m_collect;
        m_control_pub.publish(control);
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "prius_velocity_pid_node");
    PriusVelocityPID pvp;
    ros::spin();
    return 0;
}
