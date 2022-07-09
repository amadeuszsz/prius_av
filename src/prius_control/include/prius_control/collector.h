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

#ifndef PRIUS_CONTROL_COLLECTOR_H
#define PRIUS_CONTROL_COLLECTOR_H

#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <iomanip>
#include <sstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/UInt32.h>
#include "prius_msgs/Control.h"
#include "prius_msgs/State.h"


class PriusCollector {
private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_control_sub;
    ros::Publisher m_collected_frames_camera_pub;
    ros::Publisher m_collected_frames_sign_pub;
    image_transport::ImageTransport m_it;
    message_filters::Subscriber<prius_msgs::State> m_state_sub;
    cv::Rect m_roi;
    std::string m_path_data;
    std::ofstream m_csv_camera, m_csv_sign;
    u_int m_collected_camera_frames, m_collected_sign_frames;
    uint8_t m_mode_collection = prius_msgs::Control::STATE_COLLECTION_CAMERA;
    bool m_collect = false;

    typedef image_transport::SubscriberFilter ImageSubscriber;
    image_transport::Subscriber m_image_sign_sub;
    ImageSubscriber m_image_camera_sub;
    typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::Image, prius_msgs::State
    > MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> m_sync;

public:
    PriusCollector();
    ~PriusCollector();
    // Get the current mode of vehicle
    void control_callback(const prius_msgs::Control::ConstPtr& msg);
    // Image callbacks
    void camera_collector_callback(const sensor_msgs::ImageConstPtr& img_msg,
                                   const prius_msgs::StateConstPtr& control_msg);
    void sign_collector_callback(const sensor_msgs::ImageConstPtr& img_msg);
    static void counter_publisher(const ros::Publisher& pub, uint& counter);
    // Initialize CSV file with appropriate sequential frame number
    void init_collector_dir(const std::string& dataset, uint& counter, std::ofstream& csv_stream);
    // Check if file already exists
    static bool exists_file(const std::string& filename);
};

#endif //PRIUS_CONTROL_COLLECTOR_H
