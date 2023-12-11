#include "fusion_cloud_point/fusion_cloud_point.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "fusion_cloud_point_node"); 
    PointCloudMerger pointcloud_merger;
    ros::spin(); 
    return 0;
}