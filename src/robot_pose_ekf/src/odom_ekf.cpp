#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"

class OdomEKF {
public:
    OdomEKF() {


        // Publisher of type nav_msgs/Odometry
        ekf_pub = nh.advertise<nav_msgs::Odometry>("output", 5);

        // Wait for the /odom_combined topic to become available
        ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("input");

        // Subscribe to the /odom_combined topic
        sub = nh.subscribe("input", 1, &OdomEKF::pub_ekf_odom, this);

        ROS_INFO("Publishing combined odometry on");
    }

    void pub_ekf_odom(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        nav_msgs::Odometry odom;
        odom.header = msg->header;
        odom.header.frame_id = "/odom";
        odom.child_frame_id = "base_link";
        odom.pose = msg->pose;

        ekf_pub.publish(odom);
    }

private:
    ros::NodeHandle nh;
    ros::Publisher ekf_pub;
    ros::Subscriber sub;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_ekf_node");

    try {
        OdomEKF odom_ekf;
        ros::spin();
    } catch (...) {
        // Handle the exception
    }

    return 0;
}
