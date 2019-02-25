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

int main (int argc, char** argv)
{
    char key;
    ros::init(argc, argv, "pcl_write");
    ros::NodeHandle nh;

    //图片显示窗口
    cv::namedWindow("rgb image");
    cv::namedWindow("ir image");
    cv::namedWindow("depth image");
    cv::startWindowThread();//开线程刷新图片显示窗口

    //点云显示窗口
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_(color_cloud);
    // viewer->addPointCloud<pcl::PointXYZRGB> (color_cloud, rgb_, "color cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "color cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    //从yaml文件中读取相机参数（IR相机内参，RGB相机与IR相机的外参以及单应矩阵）
    trans_flag = loadyamlfiles();

    image_transport::ImageTransport it(nh);  
    image_transport::Subscriber rgbimg_sub = it.subscribe("/kinect2/cap/cap_bgr_rect", 1, rgbimageCB);
    image_transport::Subscriber irimg_sub = it.subscribe("/kinect2/sd/image_ir_rect", 1, irimageCB);
    image_transport::Subscriber depthimg_sub = it.subscribe("/kinect2/sd/image_depth_rect", 1, depthimageCB);

    ros::Rate rate(10.0);

    while (ros::ok() && ! viewer->wasStopped())
    {
        key = cv::waitKey(30);
        switch(key)
        {
            case 't':
                trans_flag = true;
                break;
            case 's':
                saveimages_flag = true;
                savecloud_flag = true;
                break;
            default:  
                break;  
        }

        if(trans_flag==true)
        {
            trans_flag = false;
            saveimages_flag = true;
            savecloud_flag = true;
            if(!rgb.empty() && !depth.empty() && !ir.empty())
            {
                ROS_INFO("Run RGB image to IR image...");
                runRGB2IR(rgb,ir,homographyMatrix,trans);
                ROS_INFO("Run depth map to color point cloud...");
                runDepth2cloud(depth, trans, RT, color_cloud);
                // viewer->removePointCloud("color cloud");
                // viewer->addPointCloud<pcl::PointXYZRGB> (color_cloud, rgb_, "color cloud");
                pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_(color_cloud);
                viewer->addPointCloud<pcl::PointXYZRGB> (color_cloud, rgb_, "color cloud");
                ROS_INFO("Depth map to point cloud complete!");
                ss << imagecount++;
                saveimages();
            }
            else
            {
                ROS_INFO("Image is empty...");
            }   
        }

        viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

//TODO:点云拼接