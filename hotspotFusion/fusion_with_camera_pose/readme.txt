说明：
通过orbslam得到的相机位姿来融合热点分布点云和环境地图

1. 记录TUM格式数据
roscore 
rosrun kinect2_bridge kinect2_bridge 
rosrun rgbd_slam saveTUMDataset path_to_save_dataset
2. 对数据集生成 associations.txt
  associate.sh
3. 运行rgbd_tum,按s键保存地图（按系统时间命名），数据集跑完后会自动将相机位姿保存在CameraTrajectory.txt中

rosrun rgbd_slam rgbd_tum BOW_file camera_set_file  path_to_tum_dataset associations.txt

4. 融合
//TODO
