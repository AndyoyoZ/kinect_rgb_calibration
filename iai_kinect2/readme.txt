RGB相机与Kinect相机联合标定：

TODO:通过lauch文件指定摄像头驱动


使用方法：
1.启动Kinect2 和 RGB相机
roslauch kinect2_bridge kinect2_bridge.lauch
或者 rosrun kinect2_bridge kinect2_bridge _fps_limits:=5
2.记录RGB相机图像
rosrun kinect2_calibration kinect2_calibration chess8x11x0.01 record color
3.标定RGB相机
rosrun kinect2_calibration kinect2_calibration chess8x11x0.01 calibrate color
4.记录红外相机图像
rosrun kinect2_calibration kinect2_calibration chess8x11x0.01 record ir
5.标定红外相机
rosrun kinect2_calibration kinect2_calibration chess8x11x0.01 calibrate ir
6.同步记录深度相机和RGB相机图像
rosrun kinect2_calibration kinect2_calibration chess8x11x0.01 record sync
7.标定外参
rosrun kinect2_calibration kinect2_calibration chess8x11x0.01 calibrate sync
8.校准深度
rosrun kinect2_calibration kinect2_calibration chess8x11x0.01 calibrate depth
