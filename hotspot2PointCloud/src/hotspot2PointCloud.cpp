#include <iostream>
#include <string>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main(int argc,char** argv)
{
    PointCloudT::Ptr cloud_e(new PointCloudT);
    PointCloudT::Ptr cloud_n(new PointCloudT);
    PointCloudT::Ptr cloud_temp(new PointCloudT);
    if (pcl::io::loadPLYFile ("../data/desk.ply", *cloud_e) < 0||pcl::io::loadPCDFile("../data/mesh.pcd", *cloud_n) < 0)
    {
        printf ("Error loading clouds.\n");
        return (-1);
    }

    for(size_t i=0;i<cloud_n->points.size();i++)
    {
        PointT temp;
        temp.x=cloud_n->points.at(i).x* 0.1;
        temp.y=cloud_n->points.at(i).y* 0.1-0.1;
        temp.z=cloud_n->points.at(i).z* 0.1+1.1;
        temp.r=0;
        temp.g=0;
        temp.b=250;
        temp.a=20;
        cloud_temp->points.push_back(temp);

        temp.x=cloud_n->points.at(i).x * 0.06;
        temp.y=cloud_n->points.at(i).y * 0.06-0.1;
        temp.z=cloud_n->points.at(i).z * 0.06+1.1;
        temp.r=255;
        temp.g=250;
        temp.b=0;
        temp.a=50;
        cloud_temp->points.push_back(temp);

        temp.x=cloud_n->points.at(i).x * 0.04;
        temp.y=cloud_n->points.at(i).y * 0.04-0.1;
        temp.z=cloud_n->points.at(i).z * 0.04+1.1;
        temp.r=200;
        temp.g=0;
        temp.b=0;
        temp.a=100;
        cloud_temp->points.push_back(temp);

    }

    *cloud_e += *cloud_temp;

    // Visualization
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    //显示彩色点云
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_(cloud_e);
    viewer->addPointCloud<PointT> (cloud_e, rgb_, "color cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "color cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    // 设置并保存点云
    cloud_e->height = 1;
    cloud_e->width = cloud_e->points.size();
    cout<<"point cloud size = "<<cloud_e->points.size()<<endl;
    cloud_e->is_dense = true;
    pcl::io::savePCDFile( "../data/irpointcloud.pcd", *cloud_e );
    cout<<"Point cloud saved."<<endl;

    while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
    cloud_e->clear();
    cloud_n->clear();
    cloud_temp->clear();


    return 0;
}
