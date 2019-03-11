
#include <iostream>
#include <fstream>
#include <cassert>
#include <string>
#include<opencv2/opencv.hpp>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>

using namespace std;
using namespace cv;

int main()
{
	ifstream fin("../data/meshData.txt",ios::in);
    if(!fin){
        printf("The file is not exist!");
        return -1;
    }

	vector<Point3d> points;
	Point3d point_tem;
	while(!fin.eof())
	{
		fin>>point_tem.x>>point_tem.y>>point_tem.z;
		points.push_back(point_tem);
		
	}

	pcl::PointCloud<pcl::PointXYZ> cloudA;
	
	cloudA.width = points.size();
	cloudA.height = 1;
	cloudA.is_dense = true;
	cloudA.points.resize (cloudA.width*cloudA.height);

	for (size_t i = 0; i < points.size(); i++) {

		std::cout<< "point_tem " <<points[i].x << ", " << points[i].y <<", "<<points[i].z << std::endl;
		cloudA.points[i].x = points[i].x;
		cloudA.points[i].y = points[i].y;
		cloudA.points[i].z = points[i].z;
	}


	pcl::io::savePCDFileASCII("../data/pointA.pcd", cloudA);
	cloudA.clear();
    return 0;
}



