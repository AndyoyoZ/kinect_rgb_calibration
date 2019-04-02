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

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

using namespace std;

#define HOTSPOT

const std::string topicColor = "/kinect2/sd/image_color_rect";
const std::string topicDepth = "/kinect2/sd/image_depth_rect";
#ifdef HOTSPOT
const std::string topicCap = "/kinect2/cap/cap_bgr";
const std::string topicIr    = "/kinect2/sd/image_ir_rect";
#endif

std::string save_path;
ofstream frgb;
ofstream fdepth;
#ifdef HOTSPOT
ofstream fcap;
ofstream fir;
ofstream fdepth_;
#endif

#ifdef HOTSPOT
void grabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgDepth,const sensor_msgs::ImageConstPtr& msgCap,const sensor_msgs::ImageConstPtr& msgIr);
#else
void grabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgDepth);
#endif

int main(int argc, char **argv)
{
    ros::init(argc, argv, "saveTUMDataset");
    ros::start();

    if(argc != 2)
    {
        cerr << endl << "Usage: saveTUMDataset save_path" << endl;        
        ros::shutdown();
        return 1;
    } 
    save_path = argv[1];

    frgb.open(save_path + "/rgb.txt");
    fdepth.open(save_path + "/depth.txt");
    #ifdef HOTSPOT
    fcap.open(save_path + "/cap.txt");
    fir.open(save_path + "/ir.txt");
    fdepth_.open(save_path + "/depth_.txt");
    #endif

    ros::NodeHandle nh;

    //显示图像
    cv::namedWindow("RGB", cv::WINDOW_AUTOSIZE); 
    cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE);
    #ifdef HOTSPOT
    cv::namedWindow("Cap", cv::WINDOW_AUTOSIZE); 
    cv::namedWindow("Ir", cv::WINDOW_AUTOSIZE);
    #endif

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, topicColor, 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, topicDepth, 1);
    #ifdef HOTSPOT
    message_filters::Subscriber<sensor_msgs::Image> cap_sub(nh, topicCap, 1);
    message_filters::Subscriber<sensor_msgs::Image> ir_sub(nh, topicIr, 1);
    #endif

    #ifdef HOTSPOT
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub,cap_sub,ir_sub);
    sync.registerCallback(boost::bind(&grabRGBD,_1,_2,_3,_4));
    #else
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&grabRGBD,_1,_2));
    #endif

    ros::spin();

    ros::shutdown();

    return 0;
}

#ifdef HOTSPOT
void grabRGBD (const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgDepth,const sensor_msgs::ImageConstPtr& msgCap,const sensor_msgs::ImageConstPtr& msgIr)
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

    cv::Mat rgbImg   = cv_ptrRGB->image;
    cv::Mat depthImg = cv_ptrDepth->image; 
    cv::Mat capImg   = cv_ptrCap->image;
    cv::Mat irImg    = cv_ptrIr->image; 

    cv::imshow("RGB",rgbImg);
    cv::imshow("Depth",depthImg);
    cv::imshow("Cap",capImg);
    cv::imshow("Ir",irImg);
    char key = cv::waitKey(10);
    
    std::string picname_rgb   = to_string( cv_ptrRGB->header.stamp.toSec());
    std::string picname_depth = to_string( cv_ptrDepth->header.stamp.toSec()); 
    std::string picname_cap   = to_string( cv_ptrCap->header.stamp.toSec());
    std::string picname_ir    = to_string( cv_ptrIr->header.stamp.toSec());

    picname_rgb   =save_path + "/rgb/"   + picname_rgb + ".png"; 
    picname_depth =save_path + "/depth/" + picname_depth + ".png";
    picname_cap   =save_path + "/cap/"   + picname_cap + ".png"; 
    picname_ir    =save_path + "/ir/"    + picname_ir + ".png";

    cv::imwrite(picname_rgb, cv_ptrRGB->image);//保存rgb图片
    cv::imwrite(picname_depth,  cv_ptrDepth->image);//保存深度图片

    std::string lie = to_string( cv_ptrRGB->header.stamp.toSec()); 
    frgb << lie << " " << "rgb/" << lie << ".png" << endl;

    lie = to_string( cv_ptrDepth->header.stamp.toSec()); 
    fdepth << lie << " " << "depth/" << lie << ".png" << endl;

    if(key==32 || key=='s')//空格键或者s
    {
        cv::imwrite(picname_cap, cv_ptrCap->image);//保存gama图片
        cv::imwrite(picname_ir,  cv_ptrIr->image);//保存IR图片 

        lie = to_string( cv_ptrCap->header.stamp.toSec()); 
        fcap << lie << " " << "cap/" << lie << ".png" << endl;

        lie = to_string( cv_ptrIr->header.stamp.toSec()); 
        fir << lie << " " << "ir/" << lie << ".png" << endl;

        lie = to_string( cv_ptrDepth->header.stamp.toSec()); 
        fdepth_ << lie << " " << "depth/" << lie << ".png" << endl;
        
        std::cout << "\n\n\tGama图片保存成功。\n\n";
    }
    else if(key == 27 || key == 'q')//ESC或者q
    {
        frgb.close();
        fdepth.close();
        std::cout << "保存图片成功。\n\n";
        ros::shutdown();
        return;
    }
          
}

#else
void grabRGBD (const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgDepth)
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

    cv::Mat rgbImg = cv_ptrRGB->image;
    cv::Mat depthImg = cv_ptrDepth->image; 


    cv::imshow("RGB",rgbImg);
    cv::imshow("Depth",depthImg);
    cv::waitKey(1);
    
    std::string picname_rgb = to_string( cv_ptrRGB->header.stamp.toSec());
    std::string picname_depth =to_string( cv_ptrDepth->header.stamp.toSec()); 
    picname_rgb =save_path + "/rgb/" + picname_rgb + ".png"; 
    picname_depth =save_path + "/depth/" + picname_depth + ".png";

    cv::imwrite(picname_rgb, cv_ptrRGB->image);//保存rgb图片
    cv::imwrite(picname_depth,  cv_ptrDepth->image);//保存深度图片

    std::string lie = to_string( cv_ptrRGB->header.stamp.toSec()); 
    frgb << lie << " " << "rgb/" << lie << ".png" << endl;

    
    lie = to_string( cv_ptrDepth->header.stamp.toSec()); 
    fdepth << lie << " " << "depth/" << lie << ".png" << endl;

    
    if(cv::waitKey(1) == 27)
    {
        frgb.close();
        fdepth.close();
        std::cout << "保存图片成功。\n\n";
        ros::shutdown();
        return;
    }
          
}
#endif