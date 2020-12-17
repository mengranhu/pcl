# 本工程用于3D-Lidar点云处理，最终获得点云数据分类的目标，以及多目标跟踪，在此感谢视觉大佬们的论文

https://blog.csdn.net/czzc1990/article/details/107442050 本人CSDN工程简介，后期持续更新，望各位大牛交流学习

## pcl_filter部分，完成原始点云的滤波，分割
1、滤波使用体素下采样 eg. pcl::VoxelGrid
2、同时使用根据不同半径，设置下采样参数

## euclidean_cluster部分，完成分割后点云地上部分目标聚类
1、欧式聚类

## multi_object_tracking部分，聚类后的目标跟踪
1、基础卡尔曼滤波
2、3D-IOU
3、匈牙利算法

## 工程编译
tar -xf pcl_ws.tar.gz
cd pcl_ws
catkin_make
source devel/setup.bash
roslaunch src/start.launch
