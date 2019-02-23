
#include <iostream>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "depth2cloud.hpp"

// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
cv::Mat rgb,ir,depth;
bool saveimages_flag = false;
int imagecount=0;
std::vector<int> params;

void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) 
{
    cv_bridge::CvImageConstPtr pCvImage;
    pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
    pCvImage->image.copyTo(image);
}
void irimageCB(const sensor_msgs::Image::ConstPtr msgImage)
{
    readImage(msgImage,ir);
    cv::imshow("ir image", ir);
    cv::waitKey(10);
}

void rgbimageCB(const sensor_msgs::Image::ConstPtr msgImage)
{
    readImage(msgImage,rgb);
    cv::imshow("rgb image", rgb);
    cv::waitKey(10);
}

void depthimageCB(const sensor_msgs::Image::ConstPtr msgImage)
{
    readImage(msgImage,depth);
    cv::imshow("depth image", depth);
    cv::waitKey(10);
}

// void pointcloudShow(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
// {
//     //显示点云
//     // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointXYZRGB);  
//     viewer->setBackgroundColor (0, 0, 0);
//     pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_(cloud);
//     viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb_, "color cloud");
//     viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "color cloud");
//     viewer->addCoordinateSystem (1.0);
//     viewer->initCameraParameters ();
// }

void saveimages()
{
    if(saveimages_flag)
    {
        saveimages_flag = false;
        cv::imwrite("myrgb.png",rgb,params);
        cv::imwrite("myir.png",ir,params);
        cv::imwrite("mydepth.png",depth,params);
        ROS_INFO("Images saved...");
    }
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "pcl_write");
    ros::NodeHandle nh;
    cv::namedWindow("rgb image");
    cv::namedWindow("ir image");
    cv::namedWindow("depth image");
    cv::startWindowThread();//开线程刷新图片显示窗口

    params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    params.push_back(9);

    image_transport::ImageTransport it(nh);  
    image_transport::Subscriber rgbimg_sub = it.subscribe("/kinect2/cap/cap_bgr_rect", 1, rgbimageCB);
    image_transport::Subscriber irimg_sub = it.subscribe("/kinect2/sd/image_ir_rect", 1, irimageCB);
    image_transport::Subscriber depthimg_sub = it.subscribe("/kinect2/sd/image_depth_rect", 1, depthimageCB);

    ros::Rate rate(1.0);

    while (ros::ok() /*&& ! viewer->wasStopped()*/)
    {
        if(cv::waitKey(10)=='s')
        {
            saveimages_flag = true;
            saveimages();
        }
        else if(cv::waitKey(10)==27)
        {
            ROS_INFO("EXIT...");
            return 0;
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}