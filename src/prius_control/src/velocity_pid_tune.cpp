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

#include "prius_control/velocity_pid_tune.h"


PriusVelocityPIDTune::PriusVelocityPIDTune() : m_nh(),
m_pid(0.01, 1.0, -1.0, 0.0, 0.0, 0.0),
m_rate(1 / 0.01)
{
    m_odom_sub = m_nh.subscribe("/base_pose_ground_truth", 1, &PriusVelocityPIDTune::odom_callback, this);
    m_control_pub = m_nh.advertise<prius_msgs::Control>("/prius", 1);
    m_control_effort_pub = m_nh.advertise<std_msgs::Float64>("/control_effort", 1);
    m_setpoint_pub = m_nh.advertise<std_msgs::Float64>("/setpoint", 1);
    m_state_pub = m_nh.advertise<std_msgs::Float64>("/state", 1);
    m_f = boost::bind(&PriusVelocityPIDTune::reconfigure_callback, this, _1, _2);
    m_server.setCallback(m_f);
    m_setpoint_timer = m_nh.createTimer(ros::Duration(5), std::bind(&PriusVelocityPIDTune::setpoint_callback, this));
}

void PriusVelocityPIDTune::reconfigure_callback(prius_control::pidConfig &config, uint32_t level)
{
    ROS_INFO("Reconfigure Request: Kp=%f Ki=%f Kd=%f Setpoint_low=%f Setpoint_high=%f",
             config.Kp, config.Ki, config.Kd, config.Setpoint_first, config.Setpoint_second);
    m_Kp = config.Kp;
    m_Ki = config.Ki;
    m_Kd = config.Kd;
    m_setpoint_first = config.Setpoint_first;
    m_setpoint_second = config.Setpoint_second;
}

void PriusVelocityPIDTune::odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
    // Set PID parameters from rqt gui
    m_pid.set_gains(m_Kp, m_Kd, m_Ki);

    // Calculate control effort and publish
    std_msgs::Float64 control_effort;
    double velocity = abs(msg->twist.twist.linear.x) + abs(msg->twist.twist.linear.y) + abs(msg->twist.twist.linear.z);
    control_effort.data = m_pid.calculate(m_setpoint.data, velocity);
    m_control_effort_pub.publish(control_effort);

    // Publish control effort as prius throttle/brake message
    prius_msgs::Control command;
    command.header.stamp = ros::Time::now();
    command.shift_gears = command.FORWARD;
    command.steer = 0;
    if (control_effort.data >= 0) {
        command.throttle = control_effort.data;
        command.brake = 0.0;
    } else if (control_effort.data < 0){
        command.brake = -control_effort.data;
        command.throttle = 0.0;
    }
    m_control_pub.publish(command);

    // Publish state
    std_msgs::Float64 state;
    state.data = velocity;
    m_state_pub.publish(state);

    m_rate.sleep();
}

void PriusVelocityPIDTune::setpoint_callback()
{
    // Publish twice to get a step
    if (m_setpoint.data != m_setpoint_first)
    {
        m_setpoint_pub.publish(m_setpoint);
        m_setpoint.data = m_setpoint_first;
        m_setpoint_pub.publish(m_setpoint);
    }
    else
    {
        m_setpoint_pub.publish(m_setpoint);
        m_setpoint.data = m_setpoint_second;
        m_setpoint_pub.publish(m_setpoint);
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "prius_velocity_pid_node");
    PriusVelocityPIDTune pvpt;
    ros::spin();
    return 0;
}
