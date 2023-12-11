#include "fusion_odom/fusion_odom.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fusion_odom_node");

    FusionOdom fusion_odom;

    ros::spin();

    return 0;
}