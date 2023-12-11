#include "laser_odom/feature_detection.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lesson1_feature_detection_node"); // 节点的名字
    LaserScan laser_scan;

    ros::spin(); // 程序执行到此处时开始进行等待，每次订阅的消息到来都会执行一次ScanCallback()
    return 0;
}