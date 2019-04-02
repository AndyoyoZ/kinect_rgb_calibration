1. 将该包放在catkin_ws/src目录下
2. 编译ORB_SLAM2_modified(build.sh)，请按照ORBSLAM2安装相应依赖项，其中g2o和DBoW2在Thirdparty目录下，下载ORBvoc.txt
3. 编译工作空间
4. 运行方式：

1） 使用kinect2（或其他RGBD传感器，需修改rgbd_slam_node.cpp中对应的话题）
roscore
rosrun kinect2_bridge kinect2_bridge 
rosrun rgbd_slam rgbd_slam_node ORBvoc.txt kinect2.yaml
2） TUM数据集
准备数据集，并生成associations.txt
roscore
rosrun rgbd_slam rgbd_tum ORBvoc.txt kinect2.yaml TUM/ TUM/associations.txt
