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

#ifndef PRIUS_CONTROL_STATE_H
#define PRIUS_CONTROL_STATE_H

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include "prius_msgs/Control.h"
#include "prius_msgs/State.h"


class PriusState {
private:
    ros::NodeHandle m_nh;
    message_filters::Subscriber<prius_msgs::Control> m_control_sub;
    message_filters::Subscriber<nav_msgs::Odometry> m_odom_sub;
    ros::Publisher m_state_pub;

    typedef message_filters::sync_policies::ApproximateTime<
    nav_msgs::Odometry, prius_msgs::Control
    > MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> m_sync;

public:
    PriusState();
    void state_callback(const nav_msgs::OdometryConstPtr& odom_msg,
                        const prius_msgs::ControlConstPtr& control_msg);
};

#endif //PRIUS_CONTROL_STATE_H
