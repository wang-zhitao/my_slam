#include "vision_odom/ImageFeatureMatcher.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "image_feature_matcher_node");
    ImageFeatureMatcher matcher;
    ros::spin();
    return 0;
}
