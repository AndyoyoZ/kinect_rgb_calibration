#include <iostream>
#include <algorithm>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <opencv2/opencv.hpp>
#include "cv_eigen.hpp"

using namespace std;
using namespace cv;

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

// #define TEST

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool save_cloud = false;//保存点云
bool next_fusion= false;//融合下一帧gama数据

cv::Mat homographyMatrix;//gama相机到IR相机的单应变换矩阵

//相机参数
double ircamera_fx;
double ircamera_fy;
double ircamera_cx;
double ircamera_cy;
double ircamera_factor;

//颜色阈值
int iLowH = 0;
int iHighH = 150;
int iLowS = 80;
int iHighS = 255;
int iLowV = 200;
int iHighV = 255;

void help()
{
    std::cout<<YELLOW<<"1. Usage:\n\t\t./cloudFusion   path_to_catkin_ws/TUM/ \n"<<RESET<<std::endl;
    std::cout<<YELLOW<<"2. 运行前，请将下列文件放在catkin_ws/TUM/目录下"<<RESET<<std::endl;
    std::cout<<YELLOW<<"\t 1) CameraTransformMatrix.txt\n\t 2) calib_ir.yaml\n\t 3) calib_pose.yaml\n\t 4) cap_depth_ir.txt\n\t 5) cloudMap.ply\n"<<RESET<<std::endl;
    std::cout<<YELLOW<<"3. 在点云显示界面按下 空格键 执行下一帧数据,按下 s键 保存点云,按下 q键 退出 "<<RESET<<std::endl;
}


bool GetCameraTransformMatrix(const std::string camera_trans_file_path, std::map<double,Eigen::Matrix4d> &camera_transform_matrix)
{
    std::string trans_file_name = camera_trans_file_path + "/CameraTransformMatrix.txt";
    ifstream fTransform;
    fTransform.open(trans_file_name.c_str());
    
    double timestamp;
    Eigen::Matrix4d temp;
   
    while(!fTransform.eof())
    {
        string s;
        getline(fTransform,s);
        if(!s.empty())
        {
            stringstream ss;
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
bool LoadCalibFiles(const std::string calib_file_path)
{
    // ircamera
    cv::FileStorage fs1(calib_file_path+"/calib_ir.yaml", cv::FileStorage::READ);
    if(!fs1.isOpened())
    {
        std::cout<<RED<<"Failed to open calib_ir.yaml.\nPlease put it in catkin_ws/TUM/"<<RESET<<std::endl;
        return false;
    }
    cv::Mat ircameraMatrix;
	fs1["cameraMatrix"] >> ircameraMatrix;  
    ircamera_factor = 1000;
    ircamera_fx = ircameraMatrix.at<double>(0,0);
    ircamera_fy = ircameraMatrix.at<double>(1,1);
    ircamera_cx = ircameraMatrix.at<double>(0,2);
    ircamera_cy = ircameraMatrix.at<double>(1,2);
 
    //calib_pose
    cv::FileStorage fs3(calib_file_path+"/calib_pose.yaml",cv::FileStorage::READ);
    if(!fs3.isOpened())
    {
        std::cout<< RED<<"Failed to open calib_pose.yaml.\nPlease put it in catkin_ws/TUM/"<< RESET<< std::endl;
        return false;
    }
    fs3["homography"] >> homographyMatrix;
    std::cout << CYAN<< "Find homography matrix of cap2ir：\n" << homographyMatrix << RESET<< std::endl;

    return true;
}


void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesCap,
                vector<string> &vstrImageFilenamesDepth, vector<string> &vstrImageFilenamesIr, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sCap, sDepth, sIr;
            // UNKNOWN=> cannot understand the >> operator
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sCap;
            vstrImageFilenamesCap.push_back(sCap);
            ss >> t;
            ss >> sDepth;
            vstrImageFilenamesDepth.push_back(sDepth);
            ss >> t;
            ss >> sIr;
            vstrImageFilenamesIr.push_back(sIr);            
        }
    }
}

void ProjectCap2Ir(cv::Mat &input_cap,cv::Mat &input_ir,cv::Mat &input_homographyMatrix,cv::Mat &output_cap2ir)
{
    //计算透视投影 cap to ir
    int img_height = input_cap.rows;
	int img_width  = input_cap.cols;
    std::vector<cv::Point2f> ponits, points_trans;
	for(int i=0;i<img_height;i++){
		for(int j=0;j<img_width;j++){
			ponits.push_back(cv::Point2f(j,i));
		}
	}
    cv::perspectiveTransform( ponits, points_trans, input_homographyMatrix);//透视投影
	cv::Mat img_trans(img_height,img_width,CV_8UC3,cv::Scalar::all(0));
	int count = 0;
	for(int i=0;i<img_height;i++){
		uchar* p = input_cap.ptr<uchar>(i);//color
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
    img_trans.copyTo(output_cap2ir);
    
    #ifdef TEST
    //在IR图像中框出与RGB图像重合的部分
    cv::Mat ir_draw;
    input_ir.copyTo(ir_draw);
    //得到rgb图像的四个顶点
    std::vector<cv::Point2f> obj_corners(4);
	obj_corners[0] = cv::Point(0,0); 
    obj_corners[1] = cv::Point(input_cap.cols, 0 );
	obj_corners[2] = cv::Point( input_cap.cols, input_cap.rows ); 
    obj_corners[3] = cv::Point( 0, input_cap.rows );
	std::vector<cv::Point2f> scene_corners(4);
	cv::perspectiveTransform( obj_corners, scene_corners, input_homographyMatrix);//计算rgb图像中4个顶点在ir图像中的投影坐标
	cv::line( ir_draw, scene_corners[0], scene_corners[1], cv::Scalar( 50000), 2 );
	cv::line( ir_draw, scene_corners[1], scene_corners[2], cv::Scalar( 50000), 2 );
	cv::line( ir_draw, scene_corners[2], scene_corners[3], cv::Scalar( 50000), 2 );
	cv::line( ir_draw, scene_corners[3], scene_corners[0], cv::Scalar( 50000), 2 );
    cv::imshow("cap",input_cap);
    cv::imshow("ir",input_ir);
    cv::imshow("cap2ir",img_trans);
    cv::imshow("ir_draw",ir_draw);
    cv::waitKey(30);
    // cv::imwrite("rgb_trans.png",img_trans);
    // cv::imwrite("ir_draw.png",ir_draw);
    #endif
    
}


void GetHotSpotMask(cv::Mat &input_cap, cv::Mat& hotspot_mask)  //产出是一个mask
{
    cv::Mat hsv_image;   
    cv::Mat mask_t(input_cap.size(),CV_8UC1);  
    cv::cvtColor(input_cap, hsv_image, CV_BGR2HSV);

    vector<cv::Mat> hsvchannels;
    cv::split(hsv_image, hsvchannels);

    // cv::equalizeHist(hsvchannels[2],hsvchannels[2]);//直方图均匀化
    // cv::merge(hsvchannels,hsv_image);
    cv::inRange(hsv_image,cv::Scalar(iLowH,iLowS,iLowV),cv::Scalar(iHighH,iHighS,iHighV),mask_t);
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask_t, mask_t, cv::MORPH_OPEN, element);//开操作
    cv::morphologyEx(mask_t, mask_t, cv::MORPH_CLOSE, element);//闭操作

    mask_t.copyTo(hotspot_mask);

    #ifdef TEST
    // cv::imshow("H",hsvchannels[0]);
    // cv::imshow("S",hsvchannels[1]);
    // cv::imshow("V",hsvchannels[2]);
    cv::imshow("mask",mask_t);
    cv::waitKey(30);
    #endif
}



void GenerateHotSpotCloud(cv::Mat &input_cap,cv::Mat &input_depth, cv::Mat &input_ir, PointCloudT::Ptr &cloud_t)
{ 
    cv::Mat imgCap2Ir;
    //通过单应矩阵将cap映射到ir
    ProjectCap2Ir(input_cap, input_ir, homographyMatrix, imgCap2Ir);

    //获取hotspotMask
    cv::Mat hotspotMask;
    hotspotMask.create(input_depth.size(),CV_8UC1);
    GetHotSpotMask(imgCap2Ir,hotspotMask);
 
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
            p.z = (double(d))/ ircamera_factor;  
            p.x = (n - ircamera_cx) * p.z / ircamera_fx;
            p.y = (m - ircamera_cy) * p.z / ircamera_fy;
            
            // p.b = imgCap2Ir.ptr<uchar>(m)[n*3];
            // p.g = imgCap2Ir.ptr<uchar>(m)[n*3+1];
            // p.r = imgCap2Ir.ptr<uchar>(m)[n*3+2];
            //将热点轮廓内的点云设置为红色
            if(hotspotMask.at<uchar>(m,n)>100)
            {
                //给热点点云着色
                p.b = 0;
                p.g = 255;
                p.r = 0;
            }
            else
            {
                // 从rgb图像中获取它的颜色
                // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
                //方式1
                p.b = imgCap2Ir.ptr<uchar>(m)[n*3];
                p.g = imgCap2Ir.ptr<uchar>(m)[n*3+1];
                p.r = imgCap2Ir.ptr<uchar>(m)[n*3+2];
                ////方式2
                // p.b = imgCap2Ir.at<cv::Vec3b>(m,n)[0];
                // p.g = imgCap2Ir.at<cv::Vec3b>(m,n)[1];
                // p.r = imgCap2Ir.at<cv::Vec3b>(m,n)[2];
            }
              
            // 当深度值小于5m且rgb值不全为0，把p加入到点云中
            if( p.z<5 && !(p.b==0 && p.g==0 && p.r==0))
            cloud_t->points.push_back( p );
        }
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,void* nothing)
{
    //按下空格键处理下一帧数据
	if (event.getKeySym() == "space" && event.keyDown())
		next_fusion = true;
    //按下s键保存点云
	if (event.getKeySym() == "s" && event.keyDown())
		save_cloud = true;

}


int main(int argc,char** argv)
{
    if(argc!=2)
    {
        help();
        return -1;
    }
    //工作空间路径
    string tum_path = argv[1];

    //加载标定文件
    if(!LoadCalibFiles(tum_path))
    {
        return -2;
    }
    //加载相机变换矩阵
    std::map<double,Eigen::Matrix4d> camera_transform_matrix;
    GetCameraTransformMatrix(tum_path, camera_transform_matrix);

    // Retrieve paths to images
    vector<string> vstrImageFilenamesCap;
    vector<string> vstrImageFilenamesDepth;
    vector<string> vstrImageFilenamesIr;
    vector<double> vTimestamps;
    string strAssociationFilename = tum_path + "/cap_depth_ir.txt";
    LoadImages(strAssociationFilename, vstrImageFilenamesCap, vstrImageFilenamesDepth, vstrImageFilenamesIr, vTimestamps);
    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesCap.size();
    if(vstrImageFilenamesCap.empty())
    {
        std::cerr<<RED << std::endl << "No images found in provided path." <<RESET<< std::endl;
        return 1;
    }
    else if(vstrImageFilenamesCap.size()!=vstrImageFilenamesDepth.size())
    {
        std::cerr<<RED << std::endl << "Different number of images for cap and depth." << std::endl;
        return 1;
    }
    else if(vstrImageFilenamesCap.size()!=vstrImageFilenamesIr.size())
    {
        std::cerr<<RED << std::endl << "Different number of images for cap and ir." << std::endl;
        return 1;
    }

    //加载点云地图
    PointCloudT::Ptr cloud_map(new PointCloudT);
    PointCloudT::Ptr cloud_hotspot(new PointCloudT);
    PointCloudT::Ptr cloud_hotspot_t(new PointCloudT);
    std::string filename_map = tum_path +"/"+ "cloudMap.ply";
	if (pcl::io::loadPLYFile(filename_map, *cloud_map) < 0)
	{
		PCL_ERROR("Error loading cloud %s.\n", filename_map);
		system("pause");
		return (-1);
	}

    // 点云可视化  
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	// pcl::visualization::PCLVisualizer viewer("3D viewer");
    // 创建两个子窗口  
	int v1(0);
	int v2(1);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
 
    //加载点云地图
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_map(cloud_map);
	viewer->addPointCloud(cloud_map, rgb_map, "cloud_map_v1", v1);
	viewer->addPointCloud(cloud_map, rgb_map, "cloud_map_v2", v2);

    //加载热点点云
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_hotspot(cloud_hotspot);
    viewer->addPointCloud(cloud_hotspot, rgb_hotspot, "cloud_add_Hotspot", v2);

	// 设置背景颜色  
	float bckgr_gray_level = 0.0;  // 黑色 
	viewer->setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
	viewer->setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);
 
	// 设置相机位置  
	viewer->setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	viewer->setSize(1280, 1024);  // 设置可视化窗口大小 
 
	// 鼠标回调函数
	viewer->registerKeyboardCallback(&keyboardEventOccurred, (void*)NULL);

    cv::Mat imgCap, imgDepth, imgIr, imgCap2Ir;
    int ni = 0;

    cv::startWindowThread();

    while(!viewer->wasStopped())
    {
        // viewer.spinOnce();
        if(next_fusion)//处理下一帧数据
        {
            next_fusion = false;

            if(ni > nImages-1)
            {
                std::cerr<<BOLDRED << std::endl << "ERROR: No next frame data. Please press 'q' to exit."<< std::endl;
                continue;
            }
            //Removes all points in a cloud and sets the width and height to 0
            cloud_hotspot->clear();
            cloud_hotspot_t->clear();
            // 读取图像数据
            imgCap = cv::imread(tum_path+"/"+vstrImageFilenamesCap[ni],CV_LOAD_IMAGE_UNCHANGED);
            imgDepth = cv::imread(tum_path+"/"+vstrImageFilenamesDepth[ni],CV_LOAD_IMAGE_UNCHANGED);
            imgIr = cv::imread(tum_path+"/"+vstrImageFilenamesIr[ni],CV_LOAD_IMAGE_UNCHANGED);
            //判断图像是否读取成功
            if(imgCap.empty())
            {   
                std::cerr<<RED << std::endl << "Failed to load image at: "
                     << tum_path << "/" << vstrImageFilenamesCap[ni] << std::endl;
                continue;
            }
            else if(imgDepth.empty())
            {
                std::cerr<<RED << std::endl << "Failed to load image at: "
                     << tum_path << "/" << vstrImageFilenamesDepth[ni] << std::endl;
                continue;
            }
            else if(imgIr.empty())
            {
                std::cerr<<RED << std::endl << "Failed to load image at: "
                     << tum_path << "/" << vstrImageFilenamesIr[ni] << std::endl;
                continue;
            }

            cv::resize(imgCap,imgCap,cv::Size(512,424));//注意，此处必须设置为与深度图相同分辨率，Kinect2为512*424
            
            double tframe = vTimestamps[ni];

            //获取当前时间戳的相机变换矩阵
            Eigen::Matrix4d T;
            std::map<double, Eigen::Matrix4d>::iterator iter;  
            iter = camera_transform_matrix.find(tframe);  
            if(iter != camera_transform_matrix.end())
            {
                T = iter->second;
                std::cout<< CYAN<<"Find transform matrix of camera2map:\n"<<T<<RESET<<std::endl;  
            }  
            else 
            {
                std::cout<<RED<<"Do not find transform matrix of camera2map."<<RESET<<std::endl;
                continue;
            } 
                  
            //生成点云
            GenerateHotSpotCloud(imgCap,imgDepth, imgIr, cloud_hotspot_t);
            //点云变换
            pcl::transformPointCloud(*cloud_hotspot_t,*cloud_hotspot,T);
            //点云更新显示
            viewer->updatePointCloud(cloud_hotspot, rgb_hotspot, "cloud_add_Hotspot");
            ni++;
        }
        if(save_cloud)
		{
            std::cout<<YELLOW<<"Saving point cloud data, please wait....."<<RESET<<std::endl;
			save_cloud = false;
			PointCloudT::Ptr cloud_t(new PointCloudT);
            *cloud_t = *cloud_hotspot;
			// 设置并保存点云
        	cloud_t->height = 1;
        	cloud_t->width = cloud_t->points.size();
        	std::cout<<YELLOW<<"hotspot cloud size = "<<cloud_t->points.size()<<RESET<<std::endl;
        	cloud_t->is_dense = false;
        	pcl::io::savePLYFile( tum_path+"/"+std::to_string(ni)+"hotSpotCloud.ply", *cloud_t );

			*cloud_t += *cloud_map;
			// 设置并保存点云
        	cloud_t->height = 1;
        	cloud_t->width = cloud_t->points.size();
        	std::cout<<YELLOW<<"fusion cloud size = "<<cloud_t->points.size()<<RESET<<std::endl;
        	cloud_t->is_dense = false;
        	pcl::io::savePLYFile( tum_path+"/"+std::to_string(ni)+"fusion.ply", *cloud_t );
        	std::cout<<YELLOW<<"Point cloud saved to "<< tum_path <<RESET<<std::endl;   
		}

        viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        
    }


    return 0;
}