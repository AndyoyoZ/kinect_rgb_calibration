#pragma once

#include <vector>
#include <iostream>
#include <ctime>
#include <time.h>
#include <iostream>
#include <fstream> 
#include <string>
#include <math.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
//#include <opencv2/core/eigen.hpp>
#include "cv_eigen.hpp"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace std;
using namespace cv;
    cv::Mat trans;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    bool saveimages_flag = false;
    bool savecloud_flag = false;
    bool trans_flag = false;
    bool show_cloud = false;
    int imagecount=0;
    cv::String path="./temp/savedata/";
    std::stringstream ss;
    cv::Mat rgb, depth, ir; 
    cv::Mat ircameraMatrix, irdistCoeffs;
    cv::Mat colorcameraMatrix, colordistCoeffs;  
    cv::Mat rotationMatrix,translationMatrix,essentialMatrix,fundamentalMatrix,homographyMatrix;  
    cv::Mat rVec,tVec;
    Eigen::Matrix4d RT;
    double depthshift;
    double ircamera_factor;
    double ircamera_fx;
    double ircamera_fy;
    double ircamera_cx;
    double ircamera_cy;

    
    void structureRT(Eigen::MatrixXd &R, Eigen::MatrixXd &t, Eigen::Matrix4d &RT);
    bool loadyamlfiles();
    void getHomographyMatrixformChessboardCorners(cv::Mat &img1,cv::Mat &img2,cv::Mat &H,int method);
    bool calc_HomographyMatrix(cv::Mat &homographyMatrix);
    void runRGB2IR(cv::Mat &input_rgb,cv::Mat &input_ir,cv::Mat &input_homographyMatrix,cv::Mat &output_trans);
    void runDepth2cloud(cv::Mat &input_depth, cv::Mat &input_rgbtrans, Eigen::Matrix4d &RT_Matrix, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output_cloud);

void runDepth2cloud(cv::Mat &input_depth, cv::Mat &input_rgbtrans, Eigen::Matrix4d &RT_Matrix, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output_cloud)
{
    // 点云变量
    // 使用智能指针，创建一个空点云。这种指针用完会自动释放。
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud   (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_t (new pcl::PointCloud<pcl::PointXYZRGB>);
    // 遍历深度图
    for (int m = 0; m < input_depth.rows; m++)
        for (int n=0; n < input_depth.cols; n++)
        {
            // 获取深度图中(m,n)处的值
            ushort d = input_depth.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            pcl::PointXYZRGB p;

            // 计算这个点的空间坐标
            p.z = (double(d)+depthshift)/ ircamera_factor;  
            p.x = (n - ircamera_cx) * p.z / ircamera_fx;
            p.y = (m - ircamera_cy) * p.z / ircamera_fy;
            
            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            //方式1
            p.b = input_rgbtrans.ptr<uchar>(m)[n*3];
            p.g = input_rgbtrans.ptr<uchar>(m)[n*3+1];
            p.r = input_rgbtrans.ptr<uchar>(m)[n*3+2];
            ////方式2
            // p.b = rgb.at<cv::Vec3b>(m,n)[0];
            // p.g = rgb.at<cv::Vec3b>(m,n)[1];
            // p.r = rgb.at<cv::Vec3b>(m,n)[2];
            
            // 把p加入到点云中
            cloud_t->points.push_back( p );
        }

    pcl::transformPointCloud(*cloud_t, *output_cloud, RT_Matrix);
    
    if(savecloud_flag)
    {
        savecloud_flag=false;
        // 设置并保存点云
        output_cloud->height = 1;
        output_cloud->width = output_cloud->points.size();
        cout<<"point cloud size = "<<output_cloud->points.size()<<endl;
        output_cloud->is_dense = false;
        pcl::io::savePCDFile( path+ss.str()+"color_PointCloud.pcd", *output_cloud );
        cout<<"Point cloud saved."<<endl;   
    }

}

    //从ros消息中读出图像数据
//使用copyTo的方式可以防止新的消息到来覆盖前面的内容
void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) 
{
    cv_bridge::CvImageConstPtr pCvImage;
    pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
    pCvImage->image.copyTo(image);
}

//IR图像消息回调函数
void irimageCB(const sensor_msgs::Image::ConstPtr msgImage)
{
    readImage(msgImage,ir);
    //调整至与深度图相同大小
    cv::resize(ir,ir,cv::Size(512,424));
    cv::imshow("ir image", ir);
    cv::waitKey(10);
}
//RGB图像消息回调函数
void rgbimageCB(const sensor_msgs::Image::ConstPtr msgImage)
{
    readImage(msgImage,rgb);
    //调整至与深度图相同大小
    cv::resize(rgb,rgb,cv::Size(512,424));
    cv::imshow("rgb image", rgb);
    cv::waitKey(10);
}
//depth图像消息回调函数
void depthimageCB(const sensor_msgs::Image::ConstPtr msgImage)
{
    readImage(msgImage,depth);
    cv::imshow("depth image", depth);
    cv::waitKey(10);
}

void saveimages()
{
    if(saveimages_flag)
    {
        saveimages_flag = false;
        cv::imwrite(path+ss.str()+"_rgb.png",rgb);
        cv::imwrite(path+ss.str()+"_ir.png",ir);
        cv::imwrite(path+ss.str()+"_depth.png",depth);
        ROS_INFO("Images saved...");
    }
}

void runRGB2IR(cv::Mat &input_rgb,cv::Mat &input_ir,cv::Mat &input_homographyMatrix,cv::Mat &output_trans)
{
    //计算透视投影 color to ir
    int img_height = input_rgb.rows;
	int img_width  = input_rgb.cols;
    std::vector<cv::Point2f> ponits, points_trans;
	for(int i=0;i<img_height;i++){
		for(int j=0;j<img_width;j++){
			ponits.push_back(Point2f(j,i));
		}
	}
    cv::perspectiveTransform( ponits, points_trans, input_homographyMatrix);//透视投影
	cv::Mat img_trans = cv::Mat::zeros(img_height,img_width,CV_8UC3);
	int count = 0;
	for(int i=0;i<img_height;i++){
		uchar* p = input_rgb.ptr<uchar>(i);//color
		for(int j=0;j<img_width;j++){
			int y = points_trans[count].y;
			int x = points_trans[count].x;
			uchar* t = img_trans.ptr<uchar>(y);
			t[x*3]  = p[j*3];
			t[x*3+1]  = p[j*3+1];
			t[x*3+2]  = p[j*3+2];
			count++;
		}
	}
    img_trans.copyTo(output_trans);
    //在IR图像中框出与RGB图像重合的部分
    cv::Mat ir_draw;
    input_ir.copyTo(ir_draw);
    //得到rgb图像的四个顶点
    std::vector<cv::Point2f> obj_corners(4);
	obj_corners[0] = cv::Point(0,0); 
    obj_corners[1] = cv::Point(input_rgb.cols, 0 );
	obj_corners[2] = cv::Point( input_rgb.cols, input_rgb.rows ); 
    obj_corners[3] = cv::Point( 0, input_rgb.rows );
	std::vector<cv::Point2f> scene_corners(4);
	cv::perspectiveTransform( obj_corners, scene_corners, input_homographyMatrix);//计算rgb图像中4个顶点在ir图像中的投影坐标
	cv::line( ir_draw, scene_corners[0], scene_corners[1], cv::Scalar( 0, 255, 0), 4 );
	cv::line( ir_draw, scene_corners[1], scene_corners[2], cv::Scalar( 0, 255, 0), 4 );
	cv::line( ir_draw, scene_corners[2], scene_corners[3], cv::Scalar( 0, 255, 0), 4 );
	cv::line( ir_draw, scene_corners[3], scene_corners[0], cv::Scalar( 0, 255, 0), 4 );
    cv::imshow("trans",img_trans);
    cv::imshow("rgb2ir",ir_draw);
}


    //根据旋转矩阵R和平移向量t构造变换矩阵RT
    void structureRT(Eigen::MatrixXd &R, Eigen::MatrixXd &t, Eigen::Matrix4d &RT)
    {  
    //  Reminder: how transformation matrices work :

    //        |-------> This column is the translation
    // | 1 0 0 x |  \
    // | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
    // | 0 0 1 z |  /
    // | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

    // METHOD #1: Using a Matrix4f
    // This is the "manual" method, perfect to understand but error prone !
    RT = Eigen::MatrixXd::Identity(4, 4);//先初始化为单位阵
    //根据旋转矩阵和平移向量构造变换矩阵
    for(size_t i=0; i<3; i++)
    {
        for(size_t j=0; j<3; j++)
        {
            RT(i,j)=R(i,j);
        }
        RT(i,3)=t(i,0);
    }
    }  

    //从yaml文件中获取参数
    bool loadyamlfiles()
    {
    // ircamera
    cv::FileStorage fs1("src/depth2cloudV0.2/data/022151343547/calib_ir.yaml", cv::FileStorage::READ);
    if(!fs1.isOpened())
    {
        std::cout<<"Failed to open calib_ir.yaml."<<std::endl;
        return false;
    }
	fs1["cameraMatrix"] >> ircameraMatrix;
	fs1["distortionCoefficients"] >> irdistCoeffs;
	std::cout << "ir camera matrix：\n"<<ircameraMatrix<<std::endl
		      << "ir distortion coeffs：\n" << irdistCoeffs << std::endl;
    
    ircamera_factor = 1000;
    ircamera_fx = ircameraMatrix.at<double>(0,0);
    ircamera_fy = ircameraMatrix.at<double>(1,1);
    ircamera_cx = ircameraMatrix.at<double>(0,2);
    ircamera_cy = ircameraMatrix.at<double>(1,2);
    //color camera
    cv::FileStorage fs2("src/depth2cloudV0.2/data/022151343547/calib_color.yaml", cv::FileStorage::READ);
    if(!fs2.isOpened())
    {
        std::cout<<"Failed to open calib_color.yaml."<<std::endl;
        return false;
    }
	
	fs2["cameraMatrix"] >> colorcameraMatrix;
	fs2["distortionCoefficients"] >> colordistCoeffs;
 
	std::cout << "color camera matrix：\n"<<ircameraMatrix<<std::endl
		      << "color distortion coeffs：\n" << irdistCoeffs << std::endl;

    //calib_pose
    cv::FileStorage fs3("src/depth2cloudV0.2/data/022151343547/calib_pose.yaml",cv::FileStorage::READ);
    if(!fs3.isOpened())
    {
        std::cout<<"Failed to open calib_pose.yaml."<<std::endl;
        return false;
    }
    
    fs3["rotation"] >> rotationMatrix;
    fs3["translation"] >> translationMatrix;
    fs3["essential"] >> essentialMatrix;
    fs3["fundamental"] >> fundamentalMatrix;
    fs3["homography"] >> homographyMatrix;
    
    std::cout << "rotation matrix：\n"<<rotationMatrix<<std::endl
		      << "translation matrix：\n" << translationMatrix << std::endl
              << "essential matrix：\n"<<essentialMatrix<<std::endl
		      << "fundamental matrix：\n" << fundamentalMatrix << std::endl
              << "homography matrix：\n" << homographyMatrix << std::endl;

              
    cv::Rodrigues(rotationMatrix,rVec); //罗德里格斯公式将旋转矩阵转化为旋转向量
    tVec = translationMatrix;
    std::cout << "rotation vector:\n" << rVec << std::endl
		      << "translation vector:\n" << tVec << std::endl;
    Eigen::MatrixXd R(3,3);
    Eigen::MatrixXd T(3,1);
    
    cv::cv2eigen(rotationMatrix,R);
    cv::cv2eigen(translationMatrix,T); 
    structureRT(R,T,RT);

    //depth shift
    cv::FileStorage fs4("src/depth2cloudV0.2/data/022151343547/calib_depth.yaml",cv::FileStorage::READ);
    if(!fs4.isOpened())
    {
        std::cout<<"Failed to open calib_depth.yaml."<<std::endl;
        return false;
    }
    
    fs4["depthShift"] >> depthshift;
    std::cout << "depthShift："<<depthshift<<std::endl;
    return true;
    }



    //根据棋盘格计算单应矩阵
    void getHomographyMatrixformChessboardCorners(cv::Mat &img1,cv::Mat &img2,cv::Mat &H,int method)
    {
    cv::Size boardSize(11, 8);//设置棋盘格
    int img_height = img1.rows;
	int img_width  = img1.cols;
    std::vector<cv::Point2f> imgCorners_1,imgCorners_2;
    bool found_1 = cv::findChessboardCorners(img1, boardSize, imgCorners_1);
    bool found_2 = cv::findChessboardCorners(img2, boardSize, imgCorners_2);

    std::vector<cv::Point2f> corners(4);
    std::vector<cv::Point2f> corners_trans(4);
    corners[0]=imgCorners_1[0];
    corners[1]=imgCorners_1[10];
    corners[2]=imgCorners_1[77];
    corners[3]=imgCorners_1[87];
    corners_trans[0]=imgCorners_2[0];
    corners_trans[1]=imgCorners_2[10];
    corners_trans[2]=imgCorners_2[77];
    corners_trans[3]=imgCorners_2[87];
    //计算单应矩阵
    if(method==0)//采用RANSAC计算多组点对的单应矩阵
    {
        std::vector<uchar> m;
        H = cv::findHomography(imgCorners_1,imgCorners_2,m,cv::RANSAC);
    }
    else if(method==1)//采用SVD计算4组点对的单应矩阵
    {
        H = cv::getPerspectiveTransform(corners,corners_trans);
    }
    
    std::cout<<"H:"<<H<<std::endl;
    fstream f1;
    f1.open("../data/Homography.txt",ios::app); 
    if (f1.is_open())  
    { 
         f1 << "\n"; 
         f1 << H;
         f1 << "\n"; 
         f1.close(); 
     }  

    std::vector<cv::Point2f> ponits, points_trans;
	for(int i=0;i<img_height;i++){
		for(int j=0;j<img_width;j++){
			ponits.push_back(Point2f(j,i));
		}
	}
    cv::perspectiveTransform( ponits, points_trans, H);
	cv::Mat img_trans = cv::Mat::zeros(img_height,img_width,CV_8UC3);
	int count = 0;
	for(int i=0;i<img_height;i++){
		uchar* p = img1.ptr<uchar>(i);
		for(int j=0;j<img_width;j++){
			int y = points_trans[count].y;
			int x = points_trans[count].x;
			uchar* t = img_trans.ptr<uchar>(y);
			t[x*3]  = p[j*3];
			t[x*3+1]  = p[j*3+1];
			t[x*3+2]  = p[j*3+2];
			count++;
		}
	}

    std::vector<cv::Point2f> obj_corners(4);
	obj_corners[0] = cv::Point(0,0); obj_corners[1] = cv::Point(img1.cols, 0 );
	obj_corners[2] = cv::Point( img1.cols, img1.rows ); obj_corners[3] = cv::Point( 0, img1.rows );
	std::vector<cv::Point2f> scene_corners(4);
	cv::perspectiveTransform( obj_corners, scene_corners, H);
	cv::line( img2, scene_corners[0], scene_corners[1], cv::Scalar( 0, 255, 0), 4 );
	cv::line( img2, scene_corners[1], scene_corners[2], cv::Scalar( 0, 255, 0), 4 );
	cv::line( img2, scene_corners[2], scene_corners[3], cv::Scalar( 0, 255, 0), 4 );
	cv::line( img2, scene_corners[3], scene_corners[0], cv::Scalar( 0, 255, 0), 4 );
    cv::imshow("trans1",img_trans);
    //绘制角点
    cv::drawChessboardCorners(img1, boardSize, imgCorners_1, found_1);
    cv::drawChessboardCorners(img2, boardSize, imgCorners_2, found_2);
    cv::imshow("findChessboardCorners1", img1);
    cv::imshow("findChessboardCorners2", img2);
    cv::waitKey(10);
    }


    //计算单应矩阵均值
    bool calc_HomographyMatrix(cv::Mat &homographyMatrix)
    {
    Eigen::MatrixXd H_t_Matrix(3,3);
    Eigen::MatrixXd sum_Matrix(3,3);
    Eigen::MatrixXd aver_Matrix(3,3);
    //通过20组图像计算单应矩阵的均值
    for(int img_num=0;img_num<20;img_num++)
    {
        char num_buf[4];
        sprintf(num_buf,"%04d",img_num);
        std::stringstream ss;
        ss << num_buf;
    
        cv::String color_img_path="temp/"+ss.str()+"_sync_color.png";
        cv::String ir_img_path="temp/"+ss.str()+"_sync_ir.png";
        cv::String depth_img_path="temp/"+ss.str()+"_sync_depth.png";
        // 读取../data/rgb.png和../data/depth.png，并转化为点云
        
        ir  = cv::imread(ir_img_path);
        rgb = cv::imread(color_img_path);
        // rgb 图像是8UC3的彩色图像
        // depth 是16UC1的单通道图像，注意flags设置-1,表示读取原始数据不做任何修改
        depth = cv::imread(depth_img_path, -1 );
        // std::cout<<"depthdata:"<<depth<<std::endl;

        if(rgb.empty()||depth.empty()||ir.empty())
        {
            std::cout << "Can not open image file..." <<std::endl;
            return false;
        }
        cv::resize(rgb,rgb,cv::Size(512,424));
        cv::resize(ir,ir,cv::Size(512,424));
        cv::imshow("ir",ir);
        cv::imshow("rgb",rgb);
        //计算单应矩阵
        getHomographyMatrixformChessboardCorners(rgb,ir,homographyMatrix,0);

        cv::cv2eigen(homographyMatrix,H_t_Matrix);
        sum_Matrix=sum_Matrix+H_t_Matrix;
        aver_Matrix=sum_Matrix/(img_num+1);
        std::cout<<"aver_H_Matrix"<< aver_Matrix<<std::endl;
        cv::waitKey(0);
    }
    return true;
    }










