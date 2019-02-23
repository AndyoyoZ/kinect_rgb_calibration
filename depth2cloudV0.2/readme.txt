ROS下的Kinect与RGB融合，得到着色点云
by andyoyo@swust
2019/2/23
1.启动roscore
roscore
2.启动Kinect2和RGB相机
rosrun kinect2_bridge kinect2_bridge _fps_limits:=5
3.运行数据融合节点（在图像显示窗口按's'可以保存RGB图像、IR图像和深度图）
rosrun depth2cloud depth2cloud_node 

