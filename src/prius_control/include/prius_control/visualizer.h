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

#ifndef PRIUS_CONTROL_VISUALIZER_H
#define PRIUS_CONTROL_VISUALIZER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iomanip>
#include <sstream>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt8.h>
#include "prius_msgs/Control.h"
#include "prius_msgs/State.h"


class PriusVisualizer {
private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_collected_frames_front_camera_sub;
    ros::Subscriber m_collected_frames_sign_sub;
    ros::Subscriber m_control_sub;
    ros::Subscriber m_sign_type_sub;
    ros::Subscriber m_sign_image_sub;
    ros::Subscriber m_state_sub;
    image_transport::ImageTransport m_it;
    image_transport::Subscriber m_image_sub;

    cv::Mat m_last_sign;
    std::string m_velocity, m_steer, m_throttle, m_brake = "0.00";
    std::string m_gear = "Gear: Forward";
    std::string m_collect = "Data collection: OFF";
    std::string m_mode_driving = "Mode driving: Manual";
    std::string m_no_front_camera = "Collected: 0";
    std::string m_no_sign = "Collected: 0";
    std::string m_no = "Collected: 0";
    std::string m_mode_collection = "Mode collection: camera";
    std::string m_sign_type = "Speed limit: None";

public:
    PriusVisualizer();
    ~PriusVisualizer();
    void image_callback(const sensor_msgs::Image::ConstPtr& msg);
    void collected_frames_front_camera_callback(const std_msgs::UInt32::ConstPtr& msg);
    void collected_frames_sign_callback(const std_msgs::UInt32::ConstPtr& msg);
    void control_callback(const prius_msgs::Control::ConstPtr& msg);
    void sign_type_callback(const std_msgs::UInt8::ConstPtr& msg);
    void sign_image_callback(const sensor_msgs::Image::ConstPtr& msg);
    void state_callback(const prius_msgs::State::ConstPtr& msg);
    // Write vehicle parameters on the front camera frame
    static cv::Mat odometer_dashboard(const cv::Mat& img_color, int x, int y, std::vector<std::string> texts);
};

#endif //PRIUS_CONTROL_VISUALIZER_H