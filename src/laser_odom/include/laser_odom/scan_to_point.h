#ifndef SCAN_TO_POINT_H
#define SCAN_TO_POINT_H

// ros
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

// pcl_ros
#include <pcl_ros/point_cloud.h>    

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <limits>
class ScanToPointCloud2Converter
{
    // 使用PCL中点的数据结构 pcl::PointXYZ
    typedef pcl::PointXYZ PointT;
    // 使用PCL中点云的数据结构 pcl::PointCloud<pcl::PointXYZ>
    typedef pcl::PointCloud<PointT> PointCloudT;

private:
    ros::NodeHandle node_handle_;           // ros中的句柄
    ros::NodeHandle private_node_;          // ros中的私有句柄
    ros::Subscriber laser_scan_subscriber_; // 声明一个Subscriber
    ros::Publisher pointcloud2_publisher_;  // 声明一个Publisher
    PointT invalid_point_;                  // 保存无效点的值,为nan
public:
    ScanToPointCloud2Converter(): private_node_("~"){
        ROS_INFO_STREAM("\033[1;32m----> Scan to PointCloud2 Converter.\033[0m");

        laser_scan_subscriber_ = node_handle_.subscribe(
            "/scan", 1, &ScanToPointCloud2Converter::ScanCallback, this);

        pointcloud2_publisher_ = node_handle_.advertise<PointCloudT>(
            "/scan_pointcloud", 1, this);

        invalid_point_.x = std::numeric_limits<float>::quiet_NaN();
        invalid_point_.y = std::numeric_limits<float>::quiet_NaN();
        invalid_point_.z = std::numeric_limits<float>::quiet_NaN();
    }
    ~ScanToPointCloud2Converter(){
        ROS_INFO("Destroying ScanToPointCloud2Converter");
    }
    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg){
        // 优化：使用emplace_back避免额外的拷贝构造
        PointCloudT::Ptr cloud_msg(new PointCloudT());
        cloud_msg->points.reserve(scan_msg->ranges.size());

        // 优化：避免多次计算sin和cos
        std::vector<float> sin_values(scan_msg->ranges.size());
        std::vector<float> cos_values(scan_msg->ranges.size());

        for (unsigned int i = 0; i < scan_msg->ranges.size(); ++i)
        {
            float range = scan_msg->ranges[i];

            if (!std::isfinite(range))
            {
                // 优化：使用invalid_point_而不是构造新的PointT对象
                cloud_msg->points.emplace_back(invalid_point_);
            }
            else if (range > scan_msg->range_min && range < scan_msg->range_max)
            {
                // 优化：避免多次计算sin和cos
                if (sin_values[i] == 0.0)
                    sin_values[i] = sin(scan_msg->angle_min + i * scan_msg->angle_increment);

                if (cos_values[i] == 0.0)
                    cos_values[i] = cos(scan_msg->angle_min + i * scan_msg->angle_increment);

                // 优化：使用emplace_back避免额外的拷贝构造
                cloud_msg->points.emplace_back(range * cos_values[i], range * sin_values[i], 0.0);
            }
            else
            {
                // 优化：使用emplace_back避免额外的拷贝构造
                cloud_msg->points.emplace_back(invalid_point_);
            }
        }

        cloud_msg->width = scan_msg->ranges.size();
        cloud_msg->height = 1;
        cloud_msg->is_dense = false;
        pcl_conversions::toPCL(scan_msg->header, cloud_msg->header);

        pointcloud2_publisher_.publish(cloud_msg);
    }
};

#endif 