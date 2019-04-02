/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include "include/System.h"
#include "depth2cloud.hpp"

using namespace std;

const std::string topicColor = "/kinect2/sd/image_color_rect";
const std::string topicCap = "/kinect2/cap/cap_bgr";
const std::string topicIr = "/kinect2/sd/image_ir_rect";
const std::string topicDepth = "/kinect2/sd/image_depth_rect";
bool add_hotspot_flag = true;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::Image::ConstPtr& msgCap,const sensor_msgs::Image::ConstPtr& msgIr,const sensor_msgs::ImageConstPtr& msgDepth);

    void addHotSpot(cv::Mat input_cap, cv::Mat input_ir, cv::Mat input_depth,double timestamp);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD_SLAM");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    
    // cv::namedWindow("frame",1);
    // cv::startWindowThread();

    //从yaml文件中读取相机参数（IR相机内参，RGB相机与IR相机的外参以及单应矩阵）
    // cv::FileStorage fs("/home/communicationgroup/catkin_ws/temp/calib_pose.yaml",cv::FileStorage::READ);
    // if(!fs.isOpened())
    // {
    //     std::cout<<"Failed to open calib_pose.yaml."<<std::endl;
    //     ros::shutdown();
    //     return 1;
    // }
    
    // fs["homography"] >> homographyMatrix;
    // std::cout<<std::endl<<"\e[1;32;41m Homography Matrix：\n" << homographyMatrix << "\e[0m" << std::endl;

    loadyamlfiles("/home/communicationgroup/catkin_ws/temp/");

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, topicColor, 1);
    message_filters::Subscriber<sensor_msgs::Image> cap_sub(nh, topicCap, 1);
    message_filters::Subscriber<sensor_msgs::Image> ir_sub(nh, topicIr, 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, topicDepth, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,cap_sub,ir_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2,_3,_4));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD (const sensor_msgs::ImageConstPtr& msgRGB,
                            const sensor_msgs::Image::ConstPtr& msgCap,
                            const sensor_msgs::Image::ConstPtr& msgIr,
                            const sensor_msgs::ImageConstPtr& msgDepth)
    {
        // Copy the ros image message to cv::Mat.
        //RGB
        cv_bridge::CvImageConstPtr cv_ptrRGB;
        try
        {
            cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        //Cap
        cv_bridge::CvImageConstPtr cv_ptrCap;
        try
        {
            cv_ptrCap = cv_bridge::toCvShare(msgCap);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        //Ir
        cv_bridge::CvImageConstPtr cv_ptrIr;
        try
        {
            cv_ptrIr = cv_bridge::toCvShare(msgIr);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        //Depth
        cv_bridge::CvImageConstPtr cv_ptrDepth;
        try
        {
            cv_ptrDepth = cv_bridge::toCvShare(msgDepth);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
  
        mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrDepth->image,cv_ptrRGB->header.stamp.toSec());

        // if(add_hotspot_flag==true)
        // {
        //     add_hotspot_flag = false;
        //     ROS_INFO("addHotSpot...");
        //     addHotSpot(cv_ptrCap->image,cv_ptrIr->image,cv_ptrDepth->image,cv_ptrCap->header.stamp.toSec());
        // }
        
    }


void ImageGrabber::addHotSpot(cv::Mat input_cap, cv::Mat input_ir, cv::Mat input_depth,double timestamp)
{
    if(input_cap.empty()||input_ir.empty()||input_depth.empty())
    {
        ROS_INFO("Image is empty...");
        add_hotspot_flag = true;
        return;
    }
    else
    {    
        cv::Mat capImg,irImg,depthImg,transImg;
        //调整至与深度图相同大小
        cv::resize(input_cap, capImg,cv::Size(512,424));
        cv::resize(input_ir, irImg, cv::Size(512,424));
        // cv::imshow("cap image", capImg);
        // cv::imshow("ir image", irImg);
        // cv::imshow("depth image", depthImg);
        //cv::waitKey(30);
        // if(cv::waitKey(10)=='s')
        // {
        //     mpSLAM->mpPointCloudMapping->saveCloudMapFlag=true;
        // }

        ROS_INFO("Run Cap image to IR image...");
        runRGB2IR(capImg,irImg,homographyMatrix,transImg);

        ROS_INFO("Run depth map to color pointcloud...");
        runDepth2cloud(depthImg, transImg, RT, color_cloud);
            
        //保存点云数据
        char current_absolute_path[64];
        if( NULL==getcwd(current_absolute_path,64) )
        {
            cerr<<"Can not get current_absolute_path "<<endl; 
            return;
        }
        pcl::io::savePCDFileASCII (std::string(current_absolute_path)+"/"+ std::to_string(timestamp)+"_hotSpotCloud.pcd", *color_cloud);
        cout<<endl<<"\e[1;32;41mHotSpotCloud saved in"<<std::string(current_absolute_path)+"/"+std::to_string(timestamp)+"_hotSpotCloud.pcd\e[0m"<<endl<<endl;
        //保存图像数据
        cv::imwrite(std::string(current_absolute_path)+"/"+std::to_string(timestamp)+"_cap.png",capImg);
        cv::imwrite(std::string(current_absolute_path)+"/"+std::to_string(timestamp)+"_ir.png",irImg);
        cv::imwrite(std::string(current_absolute_path)+"/"+std::to_string(timestamp)+"_depth.png",depthImg);
        cv::imwrite(std::string(current_absolute_path)+"/"+std::to_string(timestamp)+"_trans.png",transImg);
        cout<<endl<<"\e[1;32;41mImages saved in"<<std::string(current_absolute_path)+"/"+std::to_string(timestamp)+"_*.png\e[0m"<<endl<<endl;
    }
}
