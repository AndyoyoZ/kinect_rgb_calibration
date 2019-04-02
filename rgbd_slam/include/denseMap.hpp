
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

//定义终端输出信息的颜色
#define RESET "\033[0m"
#define BLACK "\033[30m" /* Black */
#define RED "\033[31m" /* Red */
#define GREEN "\033[32m" /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m" /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m" /* Cyan */
#define WHITE "\033[37m" /* White */
#define BOLDBLACK "\033[1m\033[30m" /* Bold Black */
#define BOLDRED "\033[1m\033[31m" /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m" /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m" /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m" /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m" /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m" /* Bold White */

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class DenseMap
{
    public:
    DenseMap(std::string path);
 
    void init();
    bool loadCalibFiles();
    void getCameraTransformMatrix();
    void rgb2ir(cv::Mat rgbImg, cv::Mat &rgb2irImg);
    void depth2cloud(cv::Mat rgbImg, cv::Mat depthImg, double timestamp, PointCloudT::Ptr &localMap);


    PointCloudT globalMap;
    

    private:

    //相机参数
    cv::Mat irCameraMatrix;
    double ircamera_fx;
    double ircamera_fy ;
    double ircamera_cx;
    double ircamera_cy;
    double ircamera_factor;

    double timestamp;
    cv::Mat H;
    std::string ws_path;
    std::map<double,Eigen::Matrix4d> camera_transform_matrix;
 
};




// DenseMap::DenseMap(std::string path, cv::Mat cameraMatrix, cv::Mat homgraphMatrix):ws_path(path), irCameraMatrix(cameraMatrix), H(homgraphMatrix)
// {
//     getCameraTransformMatrix();
// }

DenseMap::DenseMap(std::string path):ws_path(path)
{

}

void DenseMap::init()
{
    loadCalibFiles();
    getCameraTransformMatrix();
}

void DenseMap::getCameraTransformMatrix()
{
    std::string camera_transform_matrix_filename = ws_path + "KeyFrameCameraTransformMatrix.txt";
    std::ifstream fTransform;
    fTransform.open(camera_transform_matrix_filename.c_str());
    Eigen::Matrix4d temp;
   
    while(!fTransform.eof())
    {
        std::string s;
        std::getline(fTransform,s);
        if(!s.empty())
        {
            std::stringstream ss;
            ss << s;
            ss >> timestamp;
            ss >> temp(0,0);ss >> temp(0,1);ss >> temp(0,2);ss >> temp(0,3);
            ss >> temp(1,0);ss >> temp(1,1);ss >> temp(1,2);ss >> temp(1,3);
            ss >> temp(2,0);ss >> temp(2,1);ss >> temp(2,2);ss >> temp(2,3);
            ss >> temp(3,0);ss >> temp(3,1);ss >> temp(3,2);ss >> temp(3,3);
        }
        camera_transform_matrix.insert( std::pair<double,Eigen::Matrix4d> (timestamp,temp) );        
    }

}

//从yaml文件中获取标定参数
bool DenseMap::loadCalibFiles()
{
    // ircamera
    cv::FileStorage fs1(ws_path+"/calib_ir.yaml", cv::FileStorage::READ);
    if(!fs1.isOpened())
    {
        std::cout<<RED<<"Failed to open calib_ir.yaml.\nPlease put it in catkin_ws/TUM/"<<RESET<<std::endl;
        return false;
    }

	fs1["cameraMatrix"] >> irCameraMatrix;  
    ircamera_factor = 1000;
    ircamera_fx = irCameraMatrix.at<double>(0,0);
    ircamera_fy = irCameraMatrix.at<double>(1,1);
    ircamera_cx = irCameraMatrix.at<double>(0,2);
    ircamera_cy = irCameraMatrix.at<double>(1,2);
 
    //calib_pose
    cv::FileStorage fs3(ws_path+"/calib_pose.yaml",cv::FileStorage::READ);
    if(!fs3.isOpened())
    {
        std::cout<< RED<<"Failed to open calib_pose.yaml.\nPlease put it in catkin_ws/TUM/"<< RESET<< std::endl;
        return false;
    }
    fs3["homography"] >> H;
    std::cout << CYAN<< "Find homography matrix of cap2ir：\n" << H << RESET<< std::endl;

    return true;
}

void DenseMap::rgb2ir(cv::Mat rgbImg, cv::Mat &rgb2irImg)
{
    int img_height = rgbImg.rows;
	int img_width  = rgbImg.cols;
    std::vector<cv::Point2f> ponits, points_trans;
	for(int i=0;i<img_height;i++){
		for(int j=0;j<img_width;j++){
			ponits.push_back(cv::Point2f(j,i));
		}
	}
    cv::perspectiveTransform( ponits, points_trans, H);//透视投影
	cv::Mat img_trans = cv::Mat::zeros(img_height,img_width,CV_8UC3);
	int count = 0;
	for(int i=0;i<img_height;i++){
		uchar* p = rgbImg.ptr<uchar>(i);//color
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
    img_trans.copyTo(rgb2irImg);
}

void DenseMap::depth2cloud(cv::Mat rgbImg, cv::Mat depthImg, double timestamp, PointCloudT::Ptr &localMap)
{
    //获取当前时间戳的相机变换矩阵
    //得到相机变换矩阵才进行点云转换
    Eigen::Matrix4d T;
    std::map<double, Eigen::Matrix4d>::iterator iter;  
    iter = camera_transform_matrix.find(timestamp);  
    if(iter != camera_transform_matrix.end())
        {
            T = iter->second;
            std::cout<< CYAN<<"Find transform matrix of camera2map:\n"<<T<<RESET<<std::endl;  
        }  
        else 
        {
            std::cout<<RED<<"Do not find transform matrix of camera2map.\nGo to next frame."<<RESET<<std::endl;
            return;
        }  
    //找到变换矩阵，执行深度图到点云的转换，并更新全局地图
    PointCloudT::Ptr cloud_t(new PointCloudT);
    cv::Mat rgb2irImg(rgbImg.size(),CV_8UC3,cv::Scalar(0,0,0));
    //将rgb投影到ir坐标系
    rgb2ir(rgbImg,rgb2irImg);

    // 遍历深度图
    for (int m = 0; m < depthImg.rows; m++)
        for (int n=0; n < depthImg.cols; n++)
        {
            // 获取深度图中(m,n)处的值
            ushort d = depthImg.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            PointT p;

            // 计算这个点的空间坐标
            p.z = (double(d))/ ircamera_factor;  
            p.x = (n - ircamera_cx) * p.z / ircamera_fx;
            p.y = (m - ircamera_cy) * p.z / ircamera_fy;
            
            p.b = rgb2irImg.ptr<uchar>(m)[n*3];
            p.g = rgb2irImg.ptr<uchar>(m)[n*3+1];
            p.r = rgb2irImg.ptr<uchar>(m)[n*3+2];
              
            // 当深度值小于5m且rgb值不全为0，把p加入到点云中
            if( p.z<5 && !(p.b==0 && p.g==0 && p.r==0))
            cloud_t->points.push_back( p );
        }

   
    //点云变换
    pcl::transformPointCloud(*cloud_t,*localMap,T);    
    globalMap += *localMap;   

}