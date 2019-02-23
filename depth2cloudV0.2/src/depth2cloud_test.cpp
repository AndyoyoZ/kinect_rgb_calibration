/*
将深度图和RGB图像转为点云，并保存为.pcd文件
注意：该方法要求深度图与RGB图像为相同分辨率，且完全重合。
by andyoyo@swust
2019/2/13
*/

#include "depth2cloud.hpp"

// 定义点云类型
typedef pcl::PointXYZRGB PointT;
// typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud; 


// 主函数 
int main( int argc, char** argv )
{
    if(argc!=2)
    {
        std::cout<<"Usage: ./depth2cloud 0000\n";
        return -1;
    }
    cv::String img_num = argv[1]; 


    clock_t start, end;
    start = clock();
    // ircamera
    cv::FileStorage fs1("../data/022151343547/calib_ir.yaml", cv::FileStorage::READ);
    if(!fs1.isOpened())
    {
        std::cout<<"Failed to open calib_ir.yaml."<<std::endl;
        return -1;
    }
	cv::Mat ircameraMatrix, irdistCoeffs;
	fs1["cameraMatrix"] >> ircameraMatrix;
	fs1["distortionCoefficients"] >> irdistCoeffs;
	std::cout << "ir camera matrix：\n"<<ircameraMatrix<<std::endl
		      << "ir distortion coeffs：\n" << irdistCoeffs << std::endl;
    
    const double ircamera_factor = 1000;
    const double ircamera_fx = ircameraMatrix.at<double>(0,0);
    const double ircamera_fy = ircameraMatrix.at<double>(1,1);
    const double ircamera_cx = ircameraMatrix.at<double>(0,2);
    const double ircamera_cy = ircameraMatrix.at<double>(1,2);
    //color camera
    cv::FileStorage fs2("../data/022151343547/calib_color.yaml", cv::FileStorage::READ);
    if(!fs2.isOpened())
    {
        std::cout<<"Failed to open calib_color.yaml."<<std::endl;
        return -1;
    }
	cv::Mat colorcameraMatrix, colordistCoeffs;
	fs2["cameraMatrix"] >> colorcameraMatrix;
	fs2["distortionCoefficients"] >> colordistCoeffs;
 
	std::cout << "color camera matrix：\n"<<ircameraMatrix<<std::endl
		      << "color distortion coeffs：\n" << irdistCoeffs << std::endl;


    //calib_pose
    cv::FileStorage fs3("../data/022151343547/calib_pose.yaml",cv::FileStorage::READ);
    if(!fs3.isOpened())
    {
        std::cout<<"Failed to open calib_pose.yaml."<<std::endl;
        return -1;
    }
    cv::Mat rotationMatrix,translationMatrix,essentialMatrix,fundamentalMatrix,homographyMatrix;
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

    cv::Mat rVec,tVec;          
    cv::Rodrigues(rotationMatrix,rVec); //罗德里格斯公式将旋转矩阵转化为旋转向量
    tVec = translationMatrix;
    std::cout << "rotation vector:\n" << rVec << std::endl
		      << "translation vector:\n" << tVec << std::endl;
    Eigen::MatrixXd R(3,3);
    Eigen::MatrixXd T(3,1);
    Eigen::Matrix4d RT;
    cv::cv2eigen(rotationMatrix,R);
    cv::cv2eigen(translationMatrix,T); 
    structureRT(R,T,RT);

    //depth shift
    cv::FileStorage fs4("../data/022151343547/calib_depth.yaml",cv::FileStorage::READ);
    if(!fs4.isOpened())
    {
        std::cout<<"Failed to open calib_depth.yaml."<<std::endl;
        return -1;
    }
    double depthshift;
    fs4["depthShift"] >> depthshift;
    std::cout << "depthShift："<<depthshift<<std::endl;

    
    //读取图像
    cv::Mat rgb, depth, ir;
    std::stringstream ss;
    ss << img_num;
    cv::String color_img_path="/home/communicationgroup/catkin_ws/temp/"+ss.str()+"_sync_color.png";
    cv::String ir_img_path="/home/communicationgroup/catkin_ws/temp/"+ss.str()+"_sync_ir.png";
    cv::String depth_img_path="/home/communicationgroup/catkin_ws/temp/"+ss.str()+"_sync_depth.png";
    ir  = cv::imread(ir_img_path);
    rgb = cv::imread(color_img_path);
    // rgb 图像是8UC3的彩色图像
    // depth 是16UC1的单通道图像，注意flags设置-1,表示读取原始数据不做任何修改
    depth = cv::imread(depth_img_path, -1 );
    // std::cout<<"depthdata:"<<depth<<std::endl;

    if(rgb.empty()||depth.empty()||ir.empty())
    {
        std::cout << "Can not open image file..." <<std::endl;
        return -1;
    }
    cv::resize(rgb,rgb,cv::Size(512,424));
    cv::resize(ir,ir,cv::Size(512,424));
    cv::imshow("ir",ir);
    cv::imshow("rgb",rgb);

    //计算透视投影 color to ir
    int img_height = rgb.rows;
	int img_width  = rgb.cols;
    std::vector<cv::Point2f> ponits, points_trans;
	for(int i=0;i<img_height;i++){
		for(int j=0;j<img_width;j++){
			ponits.push_back(Point2f(j,i));
		}
	}
    cv::perspectiveTransform( ponits, points_trans, homographyMatrix);//透视投影
	cv::Mat img_trans = cv::Mat::zeros(img_height,img_width,CV_8UC3);
	int count = 0;
	for(int i=0;i<img_height;i++){
		uchar* p = rgb.ptr<uchar>(i);//color
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

    //在IR图像中框出与RGB图像重合的部分
    cv::Mat ir_draw;
    ir.copyTo(ir_draw);
    //得到rgb图像的四个顶点
    std::vector<cv::Point2f> obj_corners(4);
	obj_corners[0] = cv::Point(0,0); 
    obj_corners[1] = cv::Point(rgb.cols, 0 );
	obj_corners[2] = cv::Point( rgb.cols, rgb.rows ); 
    obj_corners[3] = cv::Point( 0, rgb.rows );
	std::vector<cv::Point2f> scene_corners(4);
	cv::perspectiveTransform( obj_corners, scene_corners, homographyMatrix);//计算rgb图像中4个顶点在ir图像中的投影坐标
	cv::line( ir_draw, scene_corners[0], scene_corners[1], cv::Scalar( 0, 255, 0), 4 );
	cv::line( ir_draw, scene_corners[1], scene_corners[2], cv::Scalar( 0, 255, 0), 4 );
	cv::line( ir_draw, scene_corners[2], scene_corners[3], cv::Scalar( 0, 255, 0), 4 );
	cv::line( ir_draw, scene_corners[3], scene_corners[0], cv::Scalar( 0, 255, 0), 4 );
    cv::imshow("trans1",img_trans);
    cv::imshow("color2ir",ir_draw);
    
    // 点云变量
    // 使用智能指针，创建一个空点云。这种指针用完会自动释放。
    PointCloud::Ptr cloud ( new PointCloud );
    PointCloud::Ptr cloud_t ( new PointCloud );
    // 遍历深度图
    for (int m = 0; m < depth.rows; m++)
        for (int n=0; n < depth.cols; n++)
        {
            // 获取深度图中(m,n)处的值
            ushort d = depth.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            PointT p;

            // 计算这个点的空间坐标
            p.z = (double(d)+depthshift)/ ircamera_factor;  
            p.x = (n - ircamera_cx) * p.z / ircamera_fx;
            p.y = (m - ircamera_cy) * p.z / ircamera_fy;
            
            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            //方式1
            p.b = img_trans.ptr<uchar>(m)[n*3];
            p.g = img_trans.ptr<uchar>(m)[n*3+1];
            p.r = img_trans.ptr<uchar>(m)[n*3+2];
            ////方式2
            // p.b = rgb.at<cv::Vec3b>(m,n)[0];
            // p.g = rgb.at<cv::Vec3b>(m,n)[1];
            // p.r = rgb.at<cv::Vec3b>(m,n)[2];
            
            // 把p加入到点云中
            cloud_t->points.push_back( p );
        }

    pcl::transformPointCloud(*cloud_t, *cloud, RT);
    

    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cout<<"point cloud size = "<<cloud->points.size()<<endl;
    cloud->is_dense = false;
    pcl::io::savePCDFile( "../data/irpointcloud.pcd", *cloud );
    cout<<"Point cloud saved."<<endl;
    // calc_rgb2cloud();
    // testRgb2Cloud(cloud);
    end = clock();
    double seconds  =(double)(end - start)/CLOCKS_PER_SEC;
    std::cout<<"Runtime: "<<seconds<<std::endl;

    //显示点云
    //创建3D窗口并添加点云	
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    // pcl::visualization::PointCloudColorHandlerCustom<PointT> source_cloud_color_handler (cloud_t, 255, 255, 255);
    // viewer->addPointCloud<PointT> (cloud_t, source_cloud_color_handler, "source cloud");
    // pcl::visualization::PointCloudColorHandlerCustom<PointT> transformed_cloud_color_handler (cloud, 230, 20, 20);
    // viewer->addPointCloud<PointT> (cloud, transformed_cloud_color_handler, "transformed cloud");
    // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "source cloud");
    // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "transformed cloud");
    // viewer->addPointCloud<PointT> (cloud,"sample cloud");//不显示颜色
    //显示彩色点云
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_(cloud);
    viewer->addPointCloud<PointT> (cloud, rgb_, "color cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "color cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    
    std::cout<<"Press 'q' exit..."<<std::endl;
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

    // 清除数据并退出
    cloud->points.clear();
  
    return 0;
}



