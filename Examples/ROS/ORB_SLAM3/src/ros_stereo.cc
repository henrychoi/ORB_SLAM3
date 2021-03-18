/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<mutex>
#include<queue>
#include<thread>
#include<vector>

#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>

//#include <opencv2/core.hpp>
//#include<opencv2/core/core.hpp>

#include"include/System.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

using rclcpp::executors::MultiThreadedExecutor;


class bufnMut
{
public:
    queue<sensor_msgs::msg::Image::SharedPtr> imgLeftBuf, imgRightBuf;
    std::mutex mBufMutexLeft,mBufMutexRight;
};


class Image_Grabber : public rclcpp::Node
{
public:
    Image_Grabber(bufnMut* data) : rclcpp::Node("Image_Grabber"), m_data(data)
    {
        L_img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/gray/front/left",10,std::bind(&Image_Grabber::GrabImageLeft, this, std::placeholders::_1));
        R_img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/gray/front/right",10,std::bind(&Image_Grabber::GrabImageRight, this, std::placeholders::_1));
    };

    ~Image_Grabber()
    {
        return;
    };

    void GrabImageLeft(const sensor_msgs::msg::Image::SharedPtr msg);
    void GrabImageRight(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr L_img_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr R_img_sub_;

    bufnMut* m_data;
};

void Image_Grabber::GrabImageLeft(const sensor_msgs::msg::Image::SharedPtr L_img_msg)
{
    m_data->mBufMutexLeft.lock();
    if (!m_data->imgLeftBuf.empty())
        m_data->imgLeftBuf.pop();

    m_data->imgLeftBuf.push(L_img_msg);
    m_data->mBufMutexLeft.unlock();
}

void Image_Grabber::GrabImageRight(const sensor_msgs::msg::Image::SharedPtr R_img_msg)
{
    m_data->mBufMutexRight.lock();
    if (!m_data->imgRightBuf.empty())
        m_data->imgRightBuf.pop();

    m_data->imgRightBuf.push(R_img_msg);
    m_data->mBufMutexRight.unlock();
}

class Stereo_SlamNode : public rclcpp::Node
{
public:
    Stereo_SlamNode(ORB_SLAM3::System* pSLAM, const bool bClahe, bufnMut* data) : rclcpp::Node("ORB_SLAM_Stereo"), m_SLAM(pSLAM), mbClahe(bClahe), m_data(data)
    {
        cout << "Sync start..." << endl;
    };

    ~Stereo_SlamNode()
    {
        m_SLAM->Shutdown();
        m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    };


    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    ORB_SLAM3::System* m_SLAM;
    bufnMut* m_data;

    cv::Mat GetImage(const sensor_msgs::msg::Image::SharedPtr img_msg);
    void SyncWithImg();
    cv_bridge::CvImagePtr m_cvImPtr;
};

cv::Mat Stereo_SlamNode::GetImage(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    m_cvImPtr = cv_bridge::toCvCopy(img_msg);
    return m_cvImPtr->image;
}

void Stereo_SlamNode::SyncWithImg()
{
    const double maxTimeDiff = 0.01;
    while(1)
    {
        cv::Mat imLeft, imRight;
        double tImLeft = 0, tImRight = 0;
        if (!m_data->imgLeftBuf.empty()&&!m_data->imgRightBuf.empty())
        {
            tImLeft = m_data->imgLeftBuf.front()->header.stamp.sec + m_data->imgLeftBuf.front()->header.stamp.nanosec / 1e9;
            tImRight = m_data->imgRightBuf.front()->header.stamp.sec + m_data->imgRightBuf.front()->header.stamp.nanosec / 1e9;

            m_data->mBufMutexRight.lock();
            while((tImLeft-tImRight)>maxTimeDiff && m_data->imgRightBuf.size()>1)
            {
                m_data->imgRightBuf.pop();
                tImRight = m_data->imgRightBuf.front()->header.stamp.sec + m_data->imgRightBuf.front()->header.stamp.nanosec / 1e9;
            }
            m_data->mBufMutexRight.unlock();

            m_data->mBufMutexLeft.lock();
            while((tImRight-tImLeft)>maxTimeDiff && m_data->imgLeftBuf.size()>1)
            {
                m_data->imgLeftBuf.pop();
                tImLeft = m_data->imgLeftBuf.front()->header.stamp.sec + m_data->imgLeftBuf.front()->header.stamp.nanosec / 1e9;
            }
            m_data->mBufMutexLeft.unlock();
            
            if((tImLeft-tImRight)>maxTimeDiff || (tImRight-tImLeft)>maxTimeDiff)
            {
        	// std::cout << "big time difference" << std::endl;
        	continue;
     	    }
          
            m_data->mBufMutexLeft.lock();
            imLeft = GetImage(m_data->imgLeftBuf.front());
            m_data->imgLeftBuf.pop();
            m_data->mBufMutexLeft.unlock();

            m_data->mBufMutexRight.lock();
            imRight = GetImage(m_data->imgRightBuf.front());
            m_data->imgRightBuf.pop();
            m_data->mBufMutexRight.unlock();
#if 0
            cv::Mat position_mat = m_SLAM->GetCurrentPosition();
            if(!position_mat.empty())
            {
            	cv::Mat translation(3,1,CV_32F);
            	translation = position_mat.rowRange(0,3).col(3);
            	const double y_rot = 0.0;
        	const double x_rot = 0.0;
        	const double pi = 3.1415926535897932384626438328/180.0;
        	const double c_x = cos(x_rot * pi);
        	const double s_x = sin(x_rot * pi);
        	const double c_y = cos(y_rot * pi);
        	const double s_y = sin(y_rot * pi);
            	cout <<" x_pos :" << translation.at<float> (0) * c_y + translation.at<float> (2) * s_y;//translation.at<float> (0);
            	cout <<"        y_pos :" << translation.at<float> (0) * s_x * s_y + translation.at<float> (1) * c_x - translation.at<float> (2) * s_x * c_y;
            	cout <<"        z_pos :" << -1.0 * translation.at<float> (0) * c_x * s_y + translation.at<float> (1) * s_x + translation.at<float> (2) * c_x * c_y << endl;
            }
#endif
            if(mbClahe)
                {
                    mClahe->apply(imLeft,imLeft);
                    mClahe->apply(imRight,imRight);
                }

                cv::Mat Tcw = m_SLAM->TrackStereo(imLeft,imRight,tImLeft);
                std::chrono::milliseconds tSleep(1);
                std::this_thread::sleep_for(tSleep);            
        }
    }
}



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    bool bEqual = false;
    if(argc <4 || argc > 5)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings" << endl;
        rclcpp::shutdown();
        return 1;
    }
    if(argc==5)
    {
        std::string sbEqual(argv[4]);
        if(sbEqual == "true")
            bEqual = true;
    }

    bufnMut data;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::STEREO,true);
    
    rclcpp::executors::MultiThreadedExecutor executor;
    
    auto image_grab = std::make_shared<Image_Grabber>(&data);
        
    Stereo_SlamNode node(&SLAM, bEqual, &data);
    
    executor.add_node(image_grab);

    std::thread sync_thread(&Stereo_SlamNode::SyncWithImg, &node);
    
    executor.spin();

    return 0;
}
