说明：
使用ICP算法进行点云融合，将热点分布点云融合到环境地图中

1.先通过orbslam得到点云地图，并保存在data目录下，如：dense451.ply
2.然后通过depth2cloudV0.2将gama图像与Kinect融合，得到gama图像对应的点云，并保存在data目录下，如：desk.pcd
3.运行cloudICP进行点云融合
