# Kinect2 Viewer


注意：
根据高博的　ORBSLAM2 with PointCloud ，对viewer.cpp文件进行了修改，启动该节点时，会调用ORBSLAM2，因此需先安装ORBSLAM2，也可使用备份的原文件（viewer.cpp.back）进行编译，使用kinect2_viewer的正常功能。

## Maintainer

- [Thiemo Wiedemeyer](https://ai.uni-bremen.de/team/thiemo_wiedemeyer) <<wiedemeyer@cs.uni-bremen.de>>, [Institute for Artificial Intelligence](http://ai.uni-bremen.de/), University of Bremen

*Note:* ***Please use the GitHub issues*** *for questions and problems regarding the iai_kinect2 package and its components.* ***Do not write emails.***

## Description

This is a simple viewer for the combined color an depth image provided by Kinect like depth sensors.

It just listens to two ROS topics and displays a the color with the overlayed colored depth image or a registered point cloud.

## Dependencies

- ROS Hydro/Indigo
- OpenCV
- PCL

*for the ROS packages look at the package.xml*

## Usage

```
kinect2_viewer [options]
  name: 'any string' equals to the kinect2_bridge topic base name
  mode: 'qhd', 'hd', 'sd' or 'ir'
  visualization: 'image', 'cloud' or 'both'
  options:
    'compressed' use compressed instead of raw topics
    'approx' use approximate time synchronization
```

Example: `rosrun kinect2_viewer kinect2_viewer sd cloud`

## Key bindings

Windows:
- `ESC`, `q`: Quit
- `SPACE`, `s`: Save the current image, cloud in the current directory

Terminal:
- `CRTL`+`c`: Quit
