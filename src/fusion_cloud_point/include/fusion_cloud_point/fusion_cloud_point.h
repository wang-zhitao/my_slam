#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h> 

class PointCloudMerger
{
private:
    ros::NodeHandle nh;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
    message_filters::Subscriber<sensor_msgs::PointCloud2> scan_pointcloud_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> camera_pointcloud_sub;
    message_filters::Synchronizer<MySyncPolicy> sync;
    ros::Publisher fusion_pointcloud_pub;
    sensor_msgs::PointCloud2 fusion_cloud;
    pcl::PointCloud<pcl::PointXYZ> scan_cloud, camera_cloud;
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    pcl::PCLPointCloud2 fused_cloud;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    Eigen::Matrix4f transform;
public:
    PointCloudMerger() : nh("~"),
                        scan_pointcloud_sub(nh, "/scan_pointcloud", 1),
                        camera_pointcloud_sub(nh, "/camera/depth/points", 1),
                        sync(MySyncPolicy(10), scan_pointcloud_sub, camera_pointcloud_sub),
                        tfListener(tfBuffer)
    {
        sync.registerCallback(boost::bind(&PointCloudMerger::Callback, this, _1, _2));
        fusion_pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/fusion_cloud", 1);
        getTransformMatrix("laser_link", "camera_link_optical");
    }

    void Callback(const sensor_msgs::PointCloud2::ConstPtr& scan_pointcloud, const sensor_msgs::PointCloud2::ConstPtr& camera_pointcloud)
    {
        pcl::fromROSMsg(*scan_pointcloud, scan_cloud);
        pcl::fromROSMsg(*camera_pointcloud, camera_cloud);

        point_filter(scan_cloud);
        point_filter(camera_cloud);

        pcl::transformPointCloud(scan_cloud, transformed_cloud, transform);

        for (const auto& point : transformed_cloud.points)
        {
            camera_cloud.points.push_back(point);
        }

        if (camera_cloud.points.size() != camera_cloud.width * camera_cloud.height)
        {
            camera_cloud.width = camera_cloud.points.size();
            camera_cloud.height = 1;
        }

        pcl::toROSMsg(camera_cloud, fusion_cloud);
        fusion_cloud.header = camera_pointcloud->header;

        fusion_pointcloud_pub.publish(fusion_cloud);
    }

    void getTransformMatrix(const std::string& source_frame, const std::string& target_frame)
    {
        try
        {
            geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(3.0));

            Eigen::Translation3f translation(transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
            Eigen::Quaternionf rotation(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);
            transform = (translation * rotation).matrix();
            
            std::cout << "using a Matrix4f: " << std::endl << transform.matrix() << std::endl;
        }
        catch (tf2::TransformException& ex)
        {
            ROS_WARN("Could not obtain transformation: %s", ex.what());
        }
    }

    void point_filter(pcl::PointCloud<pcl::PointXYZ> &cloud)
    {
        sor.setInputCloud(cloud.makeShared());
        sor.setLeafSize(0.03, 0.03, 0.03);
        sor.filter(cloud);
    }
};
