RGB相机与Kinect相机联合标定 by andyoyo@swust
2019/1/22

TODO:通过lauch文件指定摄像头驱动

注意：该标定过程会保存大量图片和yaml文件，保存位置为运行命令行所在目录，建议新建一个temp目录，并在该目录下运行下列命令

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
9.保存标定结果
查看设备序列号：启动Kinect相机时，终端有输出。
[ INFO] [Kinect2Bridge::initDevice] device serial: 022151343547

roscd kinect2_bridge/data
mkdir 022151343547 
然后将标定结果保存到该目录
即
calib_color.yaml calib_depth.yaml calib_ir.yaml calib_pose.yaml


