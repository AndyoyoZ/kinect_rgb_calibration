ROS下的Kinect与RGB融合，得到着色点云
by andyoyo@swust
2019/2/23
1.启动roscore
roscore

rosrun kinect2_bridge kinect2_bridge _fps_limits:=5
rosrun depth2cloud depth2cloud_node 

