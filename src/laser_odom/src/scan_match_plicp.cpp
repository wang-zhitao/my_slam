#include "laser_odom/scan_match_plicp.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_match_plicp_node"); 
    ScanMatchPLICP scan_match_plicp;
    ros::spin(); 
    return 0;
}