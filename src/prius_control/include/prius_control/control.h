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

#ifndef PRIUS_CONTROL_CONTROL_H
#define PRIUS_CONTROL_CONTROL_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "prius_msgs/Control.h"


class PriusControl {
private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_joy_sub;
    ros::Publisher m_control_pub;
    u_int8_t m_mode_driving = 0;
    u_int8_t m_mode_collection = 0;
    bool m_collect = false;

public:
    PriusControl();
    void joy_callback(const sensor_msgs::Joy::ConstPtr &msg);
};

#endif //PRIUS_CONTROL_CONTROL_H
