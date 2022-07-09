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

#include "prius_control/visualizer.h"

PriusVisualizer::PriusVisualizer() : m_it(m_nh)
{
    m_image_sub = m_it.subscribe("/prius/front_camera/image_raw", 1, boost::bind(&PriusVisualizer::image_callback, this, _1));
    m_collected_frames_front_camera_sub = m_nh.subscribe("/prius/front_camera/collected_frames", 1, &PriusVisualizer::collected_frames_front_camera_callback, this);
    m_collected_frames_sign_sub = m_nh.subscribe("/prius/sign/collected_frames", 1, &PriusVisualizer::collected_frames_sign_callback, this);
    m_control_sub = m_nh.subscribe("/prius", 1, &PriusVisualizer::control_callback, this);
    m_sign_type_sub = m_nh.subscribe("/prius/sign/type", 1, &PriusVisualizer::sign_type_callback, this);
    m_sign_image_sub = m_nh.subscribe("/prius/sign/image_raw", 1, &PriusVisualizer::sign_image_callback, this);
    m_state_sub = m_nh.subscribe("/prius/state", 1, &PriusVisualizer::state_callback, this);
}

PriusVisualizer::~PriusVisualizer()
{
    cv::destroyWindow("PriusVisualizer");
}

void PriusVisualizer::image_callback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        cv::Mat dest_roi; // ROI of predicted sign image
        // Image processing
        cv_bridge::CvImageConstPtr cvImagePtr = cv_bridge::toCvShare(msg);
        cv::Mat img_color(cvImagePtr->image.size(), cv::IMREAD_COLOR);
        cv::cvtColor(cvImagePtr->image, img_color, CV_RGB2BGR);

        // Fill dashboard
        std::vector<std::string> left_dashboard{m_velocity, m_steer, m_throttle, m_brake, m_gear};
        std::vector<std::string> right_dashboard{m_collect, m_mode_collection, m_no, m_mode_driving, m_sign_type};
        img_color = odometer_dashboard(img_color, 10, 650, left_dashboard);
        img_color = odometer_dashboard(img_color, 400, 650, right_dashboard);

        // Append last detected sign
        try{
            dest_roi = img_color(cv::Rect(10, 10, m_last_sign.cols, m_last_sign.rows));
        } catch (cv::Exception& e)
        {
            ROS_ERROR("Trying to create roi out of image boundaries.");
        }
        m_last_sign.copyTo(dest_roi);

        cv::imshow("PriusVisualizer", img_color);
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void PriusVisualizer::sign_image_callback(const sensor_msgs::Image::ConstPtr &msg)
{
    cv_bridge::CvImageConstPtr cvImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    m_last_sign = cvImagePtr->image;
}

cv::Mat PriusVisualizer::odometer_dashboard(const cv::Mat& img_color, int x, int y, std::vector<std::string> texts)
{
    for (int i = 0; i < texts.size(); i++)
    {
        cv::putText(img_color,
                    texts[i],
                    cv::Point(x,y + 30 * i),
                    cv::FONT_HERSHEY_DUPLEX,
                    1,
                    CV_RGB(255, 255, 255),
                    1);
    }
    return img_color;
}

void PriusVisualizer::collected_frames_front_camera_callback(const std_msgs::UInt32::ConstPtr &msg) {
    // Number of images
    std::string number;
    number = std::to_string(msg->data);
    m_no_front_camera = "Collected: " + number;
}

void PriusVisualizer::collected_frames_sign_callback(const std_msgs::UInt32::ConstPtr &msg) {
    // Number of images
    std::string number;
    number = std::to_string(msg->data);
    m_no_sign = "Collected: " + number;
}

void PriusVisualizer::sign_type_callback(const std_msgs::UInt8::ConstPtr &msg) {
    // Sign
    std::string sign_type;
    sign_type = std::to_string(msg->data);
    m_sign_type = "Speed limit: " + sign_type;
}

void PriusVisualizer::control_callback(const prius_msgs::Control::ConstPtr &msg)
{
    // Steer
    std::stringstream steer_stream;
    steer_stream << std::fixed << std::setprecision(2) << msg->steer;
    m_steer = "Steer: " + steer_stream.str();

    // Throttle
    std::stringstream throttle_stream;
    throttle_stream << std::fixed << std::setprecision(2) << msg->throttle;
    m_throttle = "Throttle: " + throttle_stream.str();

    // Brake
    std::stringstream brake_stream;
    brake_stream << std::fixed << std::setprecision(2) << msg->brake;
    m_brake = "Brake: " + brake_stream.str();

    // Gears
    if (msg->shift_gears == msg->FORWARD)
        m_gear = "Gear: Forward";
    if (msg->shift_gears == msg->NEUTRAL)
        m_gear = "Gear: Neutral";
    if (msg->shift_gears == msg->REVERSE)
        m_gear = "Gear: Reverse";

    // Collection mode
    if (msg->mode_collection == msg->STATE_COLLECTION_CAMERA) {
        m_mode_collection = "Mode collection: camera";
        m_no = m_no_front_camera;
    }
    if (msg->mode_collection == msg->STATE_COLLECTION_SIGN) {
        m_mode_collection = "Mode collection: sign";
        m_no = m_no_sign;
    }
    if (msg->collect == true)
        m_collect = "Data collection: ON";
    if (msg->collect == false)
        m_collect = "Data collection: OFF";

    // Driving mode
    if (msg->mode_driving == msg->STATE_DRIVING_MANUAL)
        m_mode_driving = "Mode driving: Manual";
    if (msg->mode_driving == msg->STATE_DRIVING_AUTO)
        m_mode_driving = "Mode driving: Auto";
}

void PriusVisualizer::state_callback(const prius_msgs::State::ConstPtr &msg)
{
    std::stringstream velocity_stream;
    velocity_stream << std::fixed << std::setprecision(0) << msg->velocity * 3.6;
    m_velocity = "Velocity: " + velocity_stream.str() + " km/h";
}


int main (int argc, char** argv)
{
    ros::init(argc, argv, "prius_visualizer_node");
    PriusVisualizer pv;
    ros::spin();
    return 0;
}
