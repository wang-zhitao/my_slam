#ifndef FUSION_ODOM_H_
#define FUSION_ODOM_H_

#include "ros/ros.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "laser_odom/count.h"
#include "laser_odom/my_pose.h"


class FusionOdom
{
    private:

        ros::NodeHandle nh;

        typedef message_filters::sync_policies::ApproximateTime<laser_odom::count, laser_odom::my_pose, laser_odom::count, laser_odom::my_pose> MySyncPolicy;
        message_filters::Subscriber<laser_odom::count> corner_count_sub;
        message_filters::Subscriber<laser_odom::my_pose> scan_pose_sub;
        message_filters::Subscriber<laser_odom::count> keypoints_count_sub;
        message_filters::Subscriber<laser_odom::my_pose> camera_pose_sub;
        message_filters::Synchronizer<MySyncPolicy> sync;

        tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;
        
        ros::Publisher fusion_odom_pub;
        nav_msgs::Odometry fusion_odom;
        tf2::Quaternion fusion_odom_quaternion;
        double scan_pose_weight, camera_pose_weight;

    public:
        FusionOdom(): nh("~"),
                            corner_count_sub(nh, "/corner_count", 1),
                            scan_pose_sub(nh, "/scan_pose", 1),
                            keypoints_count_sub(nh, "/keypoints_count", 1),
                            camera_pose_sub(nh, "/camera_pose", 1),
                            sync(MySyncPolicy(10), corner_count_sub, scan_pose_sub,keypoints_count_sub,camera_pose_sub)
        {
            fusion_odom_pub = nh.advertise<nav_msgs::Odometry>("/fusion_odom",1);
            sync.registerCallback(boost::bind(&FusionOdom::Callback, this, _1, _2, _3, _4));
        }

        void Callback(const laser_odom::count::ConstPtr& corner_count,
                      const laser_odom::my_pose::ConstPtr& scan_pose,
                      const laser_odom::count::ConstPtr& keypoints_count,
                      const laser_odom::my_pose::ConstPtr& camera_pose)
        {
            


            fusion_odom.header.stamp = corner_count->header.stamp;
            fusion_odom.pose.pose.position.x = scan_pose->x * scan_pose_weight + camera_pose->x * camera_pose_weight;
            fusion_odom.pose.pose.position.y = scan_pose->y * scan_pose_weight + camera_pose->y * camera_pose_weight;
            fusion_odom_quaternion.setRPY(0,0,scan_pose->yaw * scan_pose_weight + camera_pose->yaw * camera_pose_weight);
            fusion_odom.pose.pose.orientation = tf2::toMsg(fusion_odom_quaternion);

            transformStamped.header.stamp = corner_count->header.stamp;
            transformStamped.header.frame_id = "odom";
            transformStamped.child_frame_id = "base_footprint";
            transformStamped.transform.translation.x = fusion_odom.pose.pose.position.x;
            transformStamped.transform.translation.y = fusion_odom.pose.pose.position.y;
            transformStamped.transform.rotation = fusion_odom.pose.pose.orientation;

            br.sendTransform(transformStamped);
            fusion_odom_pub.publish(fusion_odom);
        }

};

#endif