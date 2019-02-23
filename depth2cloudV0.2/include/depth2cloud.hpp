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

using namespace std;
using namespace cv;

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
    int img_num;
    for(img_num=0;img_num<20;img_num++)
    {
        char num_buf[4];
        sprintf(num_buf,"%04d",img_num);
        std::stringstream ss;
        ss << num_buf;
    
        cv::String color_img_path="/home/communicationgroup/catkin_ws/temp/"+ss.str()+"_sync_color.png";
        cv::String ir_img_path="/home/communicationgroup/catkin_ws/temp/"+ss.str()+"_sync_ir.png";
        cv::String depth_img_path="/home/communicationgroup/catkin_ws/temp/"+ss.str()+"_sync_depth.png";
        // 读取../data/rgb.png和../data/depth.png，并转化为点云
        cv::Mat rgb, depth, ir;
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
