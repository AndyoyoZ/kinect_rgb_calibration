
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include "denseMap.hpp"

#define COMPILEDWITHC11 

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 2)
    {
        cerr << endl << "Usage: ./denseMap path" << endl;
        return 1;
    }

    string path = argv[1];
    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = path + "associations.txt"; // path_to_association
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }
    std::cout<<"111\n";
    //初始化DenseMap
    DenseMap densemap(path);
    std::cout<<"222\n";
    densemap.init();
    std::cout<<"113331\n";
    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    PointCloudT::Ptr globalMap(new PointCloudT);
    pcl::visualization::CloudViewer viewer("viewer");

    // Main loop
    cv::Mat imRGB, imD;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image and depthmap from file
        imRGB = cv::imread(path+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        cv::resize(imRGB,imRGB,cv::Size(512,424));

        imD = cv::imread(path+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << path << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

        cv::imshow("frame",imRGB);
        cv::waitKey(10);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        PointCloudT::Ptr localMap(new PointCloudT);
        //计算局部地图
        densemap.depth2cloud(imRGB,imD,tframe,localMap);

        *globalMap = densemap.globalMap;//更新全局地图

        viewer.showCloud( globalMap );
        std::cout<<"show global map, size="<<globalMap->points.size()<<std::endl;

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save globalMap
    std::cout<<YELLOW<<"Saving global map, please wait....."<<RESET<<std::endl;
	
	// 设置并保存点云
    globalMap->height = 1;
    globalMap->width = globalMap->points.size();
    std::cout<<YELLOW<<"cloud size = "<<globalMap->points.size()<<RESET<<std::endl;
    globalMap->is_dense = false;
    pcl::io::savePLYFile( path+"/"+"globalMap.ply", *globalMap );

    return 0;
}


void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
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
            string sRGB, sD;
            // UNKNOWN=> cannot understand the >> operator
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);
            ss >> t;
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
        }
    }
    std::cout<<"Images loaded."<<std::endl;
}
