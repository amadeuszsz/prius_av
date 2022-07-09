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

#include "prius_control/collector.h"

PriusCollector::PriusCollector() : m_it(m_nh),
m_image_camera_sub(m_it, "/prius/front_camera/image_raw", 1),
m_state_sub(m_nh, "/prius/state", 1),
m_sync(MySyncPolicy(10), m_image_camera_sub, m_state_sub),
m_roi(0, 282, 800, 264) // first pixel above car hood at y = 546
{
    m_control_sub = m_nh.subscribe("/prius", 1, &PriusCollector::control_callback, this);
    m_image_sign_sub = m_it.subscribe("/prius/sign/image_raw", 1, boost::bind(&PriusCollector::sign_collector_callback, this, _1));
    m_collected_frames_camera_pub = m_nh.advertise<std_msgs::UInt32>("/prius/front_camera/collected_frames", 1);
    m_collected_frames_sign_pub = m_nh.advertise<std_msgs::UInt32>("/prius/sign/collected_frames", 1);
    m_path_data = ros::package::getPath("prius_vision") + "/data/";
    init_collector_dir("front_camera", m_collected_camera_frames, m_csv_camera);
    init_collector_dir("sign", m_collected_sign_frames, m_csv_sign);
    m_sync.registerCallback(boost::bind( &PriusCollector::camera_collector_callback, this, _1, _2));
}

PriusCollector::~PriusCollector()
{
    m_csv_camera.close();
    m_csv_sign.close();
}

void PriusCollector::init_collector_dir(const std::string& dataset, uint& counter, std::ofstream& csv_stream)
{
    std::string dataset_path = m_path_data + dataset;
    if (!boost::filesystem::is_directory(dataset_path))
        boost::filesystem::create_directories(dataset_path);

    std::string path_csv = dataset_path + "/data.csv";
    if (exists_file(path_csv)) {
        // Get last number of collected JPG file
        std::ifstream in_file(path_csv, std::ios::in);
        if (!in_file) exit(-1);
        std::string last_line;
        while (!in_file.eof())
            in_file >> last_line;

        in_file.close();
        std::string delimiter = ",";
        std::string token = last_line.substr(0, last_line.find(delimiter));

        // Initialize number of collected frames
        try {
            counter = std::stoi(token) + 1;
        }
        catch (const std::invalid_argument &ia) {
            counter = 0;
        }
        csv_stream.open(path_csv.c_str(), std::ios::out | std::ios::app);
    }
    else
    {
        // Initialize number of collected frames
        counter = 0;
        csv_stream.open(path_csv.c_str(), std::ios::out | std::ios::app);

        // No collected frames -> No CSV file -> create header for new csv file
        csv_stream << "no.,sim_time,steer,velocity\n";
    }
}

bool PriusCollector::exists_file(const std::string& filename)
{
    std::ifstream f(filename.c_str());
    return f.good();
}

void PriusCollector::control_callback(const prius_msgs::Control::ConstPtr &msg)
{
    m_collect = msg->collect;
    m_mode_collection = msg->mode_collection;
}

void PriusCollector::camera_collector_callback(const sensor_msgs::ImageConstPtr& img_msg,
                                               const prius_msgs::StateConstPtr& state_msg)
{
    if (!m_collect)
        return;
    if (m_mode_collection != prius_msgs::Control::STATE_COLLECTION_CAMERA)
        return;

    // Sequential number - first column
    std::stringstream seq_ss;
    seq_ss.str("");
    seq_ss << std::setw(10) << std::setfill('0') << m_collected_camera_frames;
    std::string seq = seq_ss.str();
    m_collected_camera_frames++;

    // Timestamp - second column
    std::string sim_time = std::to_string(ros::Time::now().toSec());

    // Saving data to csv file
    m_csv_camera << seq << "," << sim_time << "," << state_msg->steer << "," << state_msg->velocity << "\n";

    try
    {
        // Image processing
        cv_bridge::CvImageConstPtr cvImagePtr = cv_bridge::toCvShare(img_msg);
        cv::Mat img_color(cvImagePtr->image.size(), cv::IMREAD_COLOR);
        cv::cvtColor(cvImagePtr->image, img_color, CV_RGB2BGR);
        img_color = img_color(m_roi);
        // Saving image as jpg file
        cv::imwrite(m_path_data + "front_camera/" + seq + ".jpg", img_color);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    // Publish sequential number as a number of collected frames
    counter_publisher(m_collected_frames_camera_pub, m_collected_camera_frames);
}

void PriusCollector::sign_collector_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    if (!m_collect)
        return;
    if (m_mode_collection != prius_msgs::Control::STATE_COLLECTION_SIGN)
        return;

    // Sequential number - first column
    std::stringstream seq_ss;
    seq_ss.str("");
    seq_ss << std::setw(10) << std::setfill('0') << m_collected_sign_frames;
    std::string seq = seq_ss.str();
    m_collected_sign_frames++;

    // Timestamp - second column
    std::string sim_time = std::to_string(ros::Time::now().toSec());

    // Saving data to csv file
    m_csv_sign << seq << "," << sim_time <<  "\n";

    try
    {
        // Image processing
        cv_bridge::CvImageConstPtr cvImagePtr = cv_bridge::toCvShare(img_msg);
        cv::Mat img_color(cvImagePtr->image.size(), cv::IMREAD_COLOR);
        img_color = cvImagePtr->image;
        // Saving image as jpg file
        cv::imwrite(m_path_data + "sign/" + seq + ".jpg", img_color);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    // Publish sequential number as a number of collected frames
    counter_publisher(m_collected_frames_sign_pub, m_collected_sign_frames);
}

void PriusCollector::counter_publisher(const ros::Publisher& pub, uint& counter)
{
    std_msgs::UInt32 collected_frames_msg;
    collected_frames_msg.data = counter;
    pub.publish(collected_frames_msg);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "prius_collector_node");
    PriusCollector pc;
    ros::spin();
    return 0;
}
