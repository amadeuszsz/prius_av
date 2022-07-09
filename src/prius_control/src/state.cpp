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

#include "prius_control/state.h"


PriusState::PriusState() : m_nh(),
m_odom_sub(m_nh, "/base_pose_ground_truth", 1),
m_control_sub(m_nh, "/prius", 1),
m_sync(MySyncPolicy(10), m_odom_sub, m_control_sub)
{
    m_state_pub = m_nh.advertise<prius_msgs::State>("/prius/state", 1);
    m_sync.registerCallback(boost::bind( &PriusState::state_callback, this, _1, _2) );
}

void PriusState::state_callback(const nav_msgs::OdometryConstPtr& odom_msg,
                                const prius_msgs::ControlConstPtr& control_msg) {
    prius_msgs::State state;
    state.header.stamp = ros::Time::now();
    state.header.frame_id = "odom";
    double x_vel = abs(odom_msg->twist.twist.linear.x);
    double y_vel = abs(odom_msg->twist.twist.linear.y);
    double z_vel = abs(odom_msg->twist.twist.linear.z);
    state.velocity = x_vel + y_vel + z_vel;  // meters per second
    state.steer = control_msg->steer;
    m_state_pub.publish(state);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "prius_state_node");
    PriusState ps;
    ros::spin();
    return 0;
}
