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
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>

//#include <opencv2/core.hpp>
//#include<opencv2/core/core.hpp>

#include"include/System.h"
#include"include/ImuTypes.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

using rclcpp::executors::MultiThreadedExecutor;


class bufnMut
{
public:
    queue<sensor_msgs::msg::Image::SharedPtr> imgLeftBuf, imgRightBuf;
    queue<sensor_msgs::msg::Imu::SharedPtr> imuBuf;
    std::mutex mBufMutexLeft,mBufMutexRight;
    std::mutex mBufMutex;
};



class IMU_Grabber : public rclcpp::Node
{
public:
    IMU_Grabber(bufnMut* data) : rclcpp::Node("IMU_Grabber"), m_data(data)
    {
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
                "/imu",1000,std::bind(&IMU_Grabber::GrabImu, this, std::placeholders::_1));
    };
    ~IMU_Grabber()
    {
        return;
    };

    void GrabImu(const sensor_msgs::msg::Imu::SharedPtr imu_msg);

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    bufnMut* m_data;
};

void IMU_Grabber::GrabImu(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    m_data->mBufMutex.lock();
    m_data->imuBuf.push(imu_msg);
    m_data->mBufMutex.unlock();
    return;
}




class Image_Grabber : public rclcpp::Node
{
public:
    Image_Grabber(bufnMut* data) : rclcpp::Node("Image_Grabber"), m_data(data)
    {
        L_img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/gray/front/left",100,std::bind(&Image_Grabber::GrabImageLeft, this, std::placeholders::_1));
        R_img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/gray/front/right",100,std::bind(&Image_Grabber::GrabImageRight, this, std::placeholders::_1));
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

class IMU_Stereo_SlamNode : public rclcpp::Node
{
public:
    IMU_Stereo_SlamNode(ORB_SLAM3::System* pSLAM, const bool bClahe, bufnMut* data) : rclcpp::Node("ORB_SLAM_IMU_Stereo"), m_SLAM(pSLAM), mbClahe(bClahe), m_data(data)
    {
        cout << "Sync start..." << endl;
    };

    ~IMU_Stereo_SlamNode()
    {
        m_SLAM->Shutdown();
        m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    };


    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    ORB_SLAM3::System* m_SLAM;

    bufnMut* m_data;

    cv::Mat GetImage(const sensor_msgs::msg::Image::SharedPtr img_msg);
    void SyncWithImu();
    
    cv_bridge::CvImagePtr m_cvImPtr;
};

cv::Mat IMU_Stereo_SlamNode::GetImage(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    m_cvImPtr = cv_bridge::toCvCopy(img_msg);
    return m_cvImPtr->image;
}

void IMU_Stereo_SlamNode::SyncWithImu()
{
    const double maxTimeDiff = 0.01;
    while(1)
    {
    	//cout << "3" << endl;
        cv::Mat imLeft, imRight;
        double tImLeft = 0, tImRight = 0;
        if (!m_data->imgLeftBuf.empty()&&!m_data->imgRightBuf.empty()&&!m_data->imuBuf.empty())
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
     	    if(tImLeft>(m_data->imuBuf.back()->header.stamp.sec + m_data->imuBuf.back()->header.stamp.nanosec / 1e9))
          	continue;
          
            m_data->mBufMutexLeft.lock();
            imLeft = GetImage(m_data->imgLeftBuf.front());
            m_data->imgLeftBuf.pop();
            m_data->mBufMutexLeft.unlock();

            m_data->mBufMutexRight.lock();
            imRight = GetImage(m_data->imgRightBuf.front());
            m_data->imgRightBuf.pop();
            m_data->mBufMutexRight.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            m_data->mBufMutex.lock();
            if(!m_data->imuBuf.empty())
            {
            	// Load imu measurements from buffer
        	vImuMeas.clear();
        	while(!m_data->imuBuf.empty() && (m_data->imuBuf.front()->header.stamp.sec  + m_data->imuBuf.front()->header.stamp.nanosec / 1e9)<=tImLeft)
        	{
          		double t = m_data->imuBuf.front()->header.stamp.sec + m_data->imuBuf.front()->header.stamp.nanosec / 1e9;
          		cv::Point3f acc(m_data->imuBuf.front()->linear_acceleration.x, m_data->imuBuf.front()->linear_acceleration.y, m_data->imuBuf.front()->linear_acceleration.z);
          		cv::Point3f gyr(m_data->imuBuf.front()->angular_velocity.x, m_data->imuBuf.front()->angular_velocity.y, m_data->imuBuf.front()->angular_velocity.z);
          		vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
#if 0
          		cv::Mat position_mat = m_SLAM->GetCurrentPosition();
                if(!position_mat.empty())
                {
                    cv::Mat translation(3,1,CV_32F);
                    translation = position_mat.rowRange(0,3).col(3);
                    //cout <<" x_pos :" << translation.at<float> (0);// * cos(-25.0 * 3.141592/180) + translation.at<float> (2) * sin(-25.0 * 3.141592/180);//translation.at<float> (0);
                    //cout <<"        y_pos :" << translation.at<float> (1);
                    //cout <<"        z_pos :" << translation.at<float> (2) << endl; //-1.0 * translation.at<float> (0) * sin(-20.0 * 3.141592/180) + translation.at<float> (2) * cos(-2.0 * 3.141592/180) << endl;
                    //cout << "x_accel :" << -1.0 * m_data->imuBuf.front()->linear_acceleration.y;
                    //cout << "       y_accel :" << -1.0 * m_data->imuBuf.front()->linear_acceleration.z;
                    //cout << "       z_accel :" << -1.0 * m_data->imuBuf.front()->linear_acceleration.x << endl;
                }
#endif
          		m_data->imuBuf.pop();
        	}
            }
            m_data->mBufMutex.unlock();
            if(mbClahe)
                {
                    mClahe->apply(imLeft,imLeft);
                    mClahe->apply(imRight,imRight);
                }

                cv::Mat Tcw = m_SLAM->TrackStereo(imLeft,imRight,tImLeft,vImuMeas);
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
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_STEREO,true);
    
    rclcpp::executors::MultiThreadedExecutor executor;
    
    auto imu_grab = std::make_shared<IMU_Grabber>(&data);
    auto image_grab = std::make_shared<Image_Grabber>(&data);
        
    IMU_Stereo_SlamNode node(&SLAM, bEqual, &data);
    
    executor.add_node(imu_grab);
    executor.add_node(image_grab);

    std::thread sync_thread(&IMU_Stereo_SlamNode::SyncWithImu, &node);
    
    executor.spin();

    return 0;
}
