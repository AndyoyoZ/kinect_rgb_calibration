/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#include "pointcloudmapping.h"
#include <KeyFrame.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include "Converter.h"


PointCloudMapping::PointCloudMapping(double resolution_)
{
    this->resolution = resolution_;
    voxel.setLeafSize( resolution, resolution, resolution);
    globalMap = boost::make_shared< PointCloud >( );
    
    viewerThread = make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );
}

void PointCloudMapping::shutdown()
{
    {
        unique_lock<mutex> lck(shutDownMutex);
        shutDownFlag = true;
        keyFrameUpdated.notify_one();
    }
    viewerThread->join();
}

void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    cout<<"receive a keyframe, id = "<<kf->mnId<<endl;
    unique_lock<mutex> lck(keyframeMutex);
    keyframes.push_back( kf );
    colorImgs.push_back( color.clone() );
    depthImgs.push_back( depth.clone() );
    
    keyFrameUpdated.notify_one();
}

pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    PointCloud::Ptr tmp( new PointCloud() );
    // point cloud is null ptr
    for ( int m=0; m<depth.rows; m+=3 )
    {
        for ( int n=0; n<depth.cols; n+=3 )
        {
            float d = depth.ptr<float>(m)[n];
            if (d < 0.01 || d>10)
                continue;
            PointT p;
            p.z = d;
            p.x = ( n - kf->cx) * p.z / kf->fx;
            p.y = ( m - kf->cy) * p.z / kf->fy;
            
            p.b = color.ptr<uchar>(m)[n*3];
            p.g = color.ptr<uchar>(m)[n*3+1];
            p.r = color.ptr<uchar>(m)[n*3+2];

            //add by andyoyo 2019/4/2
            // 当深度值小于5m且rgb值不全为0，把p加入到点云中
            if( p.z<5 && !(p.b==0 && p.g==0 && p.r==0))    
            tmp->points.push_back(p);
        }
    }
    
    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
    PointCloud::Ptr cloud(new PointCloud);
    pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());
    // cloud->is_dense = false;//add by andyoyo
    cloud->is_dense = true;
    //add end
    
    cout<<"generate point cloud for kf "<<kf->mnId<<", size="<<cloud->points.size()<<endl;
    return cloud;
}


void PointCloudMapping::viewer()
{
    pcl::visualization::CloudViewer viewer("viewer");
    while(1)
    {
        {
            unique_lock<mutex> lck_shutdown( shutDownMutex );
            if (shutDownFlag)
            {
                break;
            }
        }
        {
            unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
            keyFrameUpdated.wait( lck_keyframeUpdated );
        }
        
        // keyframe is updated 
        size_t N=0;
        {
            unique_lock<mutex> lck( keyframeMutex );
            N = keyframes.size();
        }
        
        for ( size_t i=lastKeyframeSize; i<N ; i++ )
        {
            PointCloud::Ptr p = generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i] );
            *globalMap += *p;
        }
        //取消滤波，可以得到更稠密的点云
        // PointCloud::Ptr tmp(new PointCloud());
        // voxel.setInputCloud( globalMap );
        // voxel.filter( *tmp );
        // globalMap->swap( *tmp );
        viewer.showCloud( globalMap );
        cout<<"show global map, size="<<globalMap->points.size()<<endl;
        lastKeyframeSize = N;

        //add by andyoyo 2019/3/15
        //save cloud map
        if(this->saveCloudMapFlag == true)
        {
            this->saveCloudMapFlag = false;
            char current_absolute_path[64];
            if( NULL==getcwd(current_absolute_path,64) )
            {
               cerr<<"Can not get current_absolute_path "<<endl; 
               continue;
            }
            time_t t = time(NULL);
            char current_time[64] = {0};
	        strftime(current_time, sizeof(current_time) - 1, "%Y-%m-%d-%H-%M-%S", localtime(&t));     //年-月-日-时-分-秒
            // pcl::io::savePCDFileASCII (std::string(current_absolute_path)+"/"+std::string(current_time)+"_cloudMap.pcd", *globalMap);//保存为pcd
            pcl::PLYWriter writer;
            writer.write(std::string(current_absolute_path)+"/"+std::string(current_time)+"_cloudMap.ply",*globalMap,true,true);
            cout<<endl<<"\e[1;32;41mCloud map saved in"<<std::string(current_absolute_path)+"/"+std::string(current_time)+"_cloudMap.pcd\e[0m"<<endl<<endl;
        }
        //add end
    }
}


// //add by andyoyo
// void PointCloudMapping::keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
// {
//     if(event.getKeySym() == "space")
//     {
//         this->saveCloudMapFlag = true;
//     }
// }

// void PointCloudMapping::viewer()
// {
//     boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
    
//     viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
//     viewer->registerKeyboardCallback(&PointCloudMapping::keyboardEventOccurred,*this, (void*)&viewer);
//     viewer->spin();

//     while(!viewer->wasStopped())
//     {
//         {
//             unique_lock<mutex> lck_shutdown( shutDownMutex );
//             if (shutDownFlag)
//             {
//                 break;
//             }
//         }
//         {
//             unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
//             keyFrameUpdated.wait( lck_keyframeUpdated );
//         }
        
//         // keyframe is updated 
//         size_t N=0;
//         {
//             unique_lock<mutex> lck( keyframeMutex );
//             N = keyframes.size();
//         }
        
//         for ( size_t i=lastKeyframeSize; i<N ; i++ )
//         {
//             PointCloud::Ptr p = generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i] );
//             *globalMap += *p;
//         }
//         PointCloud::Ptr tmp(new PointCloud());
//         voxel.setInputCloud( globalMap );
//         voxel.filter( *tmp );
//         globalMap->swap( *tmp );
        

        
//         // boost::mutex::scoped_lock updateLock(updateModelMutex);

//         pcl::visualization::PointCloudColorHandlerRGBAField<PointT> rgba(globalMap);
//         viewer->addPointCloud<PointT>(globalMap,rgba, "globalMap");
//         if(saveCloudMapFlag == true)
//         {
//             saveCloudMapFlag = false;
//             pcl::io::savePCDFileASCII ("cloudMap.pcd", *globalMap);
//             cout<<"Cloud map saved."<<endl;
//         }
//         cout<<"show global map, size="<<globalMap->points.size()<<endl;
//         lastKeyframeSize = N;
//         // updateLock.unlock();

//         viewer->spinOnce(100);
//         boost::this_thread::sleep(boost::posix_time::microseconds(100000));

//         viewer->removeAllPointClouds();
//     }
// }
//add end