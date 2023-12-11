#include "laser_odom/scan_to_point.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_to_point_node");
    ScanToPointCloud2Converter scan_to_cloud_converter;

    ros::spin();
    return 0;
}
