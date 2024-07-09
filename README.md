# EASY_FASTLIO2
## 主要工作
参考了大佬 liangheming [FASTLIO2_ROS2](https://github.com/liangheming/FASTLIO2_ROS2.git)
重构工作，改写简化了运动方程的雅克比矩阵，使用高博的[简明ESKF推导](https://zhuanlan.zhihu.com/p/441182819)方式，更容易学习理解。


## 环境依赖
1. Ubuntu 20.04
2. ROS neotic

## 编译依赖
```text
pcl
Eigen
sophus
gtsam
livox_ros_driver
```

## 详细说明
```
mkdir -p fastlio_ws/src
cd fastlio_ws/src
git clone https://github.com/AFEICHINA/EASY_FASTLIO2.git
cd ..
catkin_make
source devel/setup.bash
roslaunch fastlio2 temp.launch
```

## 特别感谢
1. [FASTLIO2](https://github.com/hku-mars/FAST_LIO)
2. [FASTLIO2_ROS2](https://github.com/liangheming/FASTLIO2_ROS2.git)

