#pragma once
#include <iomanip>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
// #include <std_msgs/header.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>
// #include <builtin_interfaces/Time.h>
#include <ros/time.h>

#define RESET "\033[0m"
#define BLACK "\033[30m"  /* Black */
#define RED "\033[31m"    /* Red */
#define GREEN "\033[32m"  /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m"   /* Blue */
#define PURPLE "\033[35m" /* Purple */
#define CYAN "\033[36m"   /* Cyan */
#define WHITE "\033[37m"  /* White */

class Utils
{
public:
    // static pcl::PointCloud<pcl::PointXYZ>::Ptr convertToPCL(const sensor_msgs::msg::PointCloud2 &msg);
    // static sensor_msgs::msg::PointCloud2 convertToROS(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    static double getSec(std_msgs::Header &header);
    static pcl::PointCloud<pcl::PointXYZINormal>::Ptr livox2PCL(const livox_ros_driver::CustomMsg::Ptr msg, int filter_num, double min_range = 0.5, double max_range = 20.0);
    static ros::Time getTime(const double& sec);
};