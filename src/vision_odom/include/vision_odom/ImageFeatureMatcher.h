#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <laser_odom/count.h>
#include <laser_odom/my_pose.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

class ImageFeatureMatcher {
public:
    ImageFeatureMatcher() : nh("~"), 
                            depth_sub(nh, "/camera/rgb/image_raw", 1), 
                            color_sub(nh, "/camera/depth/image_raw", 1), 
                            sync(MySyncPolicy(10), depth_sub, color_sub),
                            matcher(cv::NORM_HAMMING)
    {
        sync.registerCallback(boost::bind(&ImageFeatureMatcher::callback, this, _1, _2));
        keypoints_count_pub = nh.advertise<laser_odom::count>("/keypoints_count",1);
        camera_pose_pub = nh.advertise<laser_odom::my_pose>("/camera_pose",1);
        orb_detector_ = cv::ORB::create();
    }


    void callback(const sensor_msgs::ImageConstPtr& color_msg, const sensor_msgs::ImageConstPtr& depth_msg) {

        current_rgb_image = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8)->image;
        current_depth_image = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
        if (!init) {
                previous_rgb_image = current_rgb_image.clone();
                previous_depth_image = current_depth_image.clone();
                init = true;
                return;
            }

        
        
        orb_extract_and_match();

        camera_pose.header.stamp = color_msg->header.stamp;
        keypoints_count.header.stamp = color_msg->header.stamp;
        keypoints_count_pub.publish(keypoints_count);

        previous_rgb_image = current_rgb_image.clone();
        previous_depth_image = current_depth_image.clone();

        clear();

    }

    void orb_extract_and_match(){

        keypoints_count.a = 0;
        // 检测关键点
        orb_detector_->detect(previous_rgb_image, keypoints_previous);
        orb_detector_->detect(current_rgb_image, keypoints_current);

        // 应用非极大值抑制（NMS）
        cv::KeyPointsFilter::retainBest(keypoints_previous, 100); // 保留前100个最佳关键点
        cv::KeyPointsFilter::retainBest(keypoints_current, 100);

        // 计算描述符
        orb_detector_->compute(previous_rgb_image, keypoints_previous, descriptors_previous);
        orb_detector_->compute(current_rgb_image, keypoints_current, descriptors_current);

        // 匹配描述符
        matcher.match(descriptors_previous, descriptors_current, matches);
        if (matches.empty()) {
            std::cout << "No matches found." << std::endl;
            keypoints_count.a = 0;
            
            init = false;
            return;
        }

        // 使用RANSAC筛选好的匹配点
        for (size_t i = 0; i < matches.size(); i++)
        {
            points1.push_back(keypoints_previous[matches[i].queryIdx].pt);
            points2.push_back(keypoints_current[matches[i].trainIdx].pt);
        }

        std::vector<uchar> inliers(points1.size(), 0);
        cv::Mat homography = cv::findHomography(points1, points2, cv::RANSAC, 3, inliers);

        for (size_t i = 0; i < inliers.size(); i++)
        {
            if (inliers[i])
            {
                good_matches.push_back(matches[i]);
            }
        }
        keypoints_count.a = good_matches.size();
        if(keypoints_count.a <= 4)
        {
            init = false;
            return;
        }
        pnp_slover();
    }

    void pnp_slover(){
        for (cv::DMatch m:good_matches) {
            ushort d = current_depth_image.ptr<unsigned short>(int(keypoints_previous[m.queryIdx].pt.y))[int(keypoints_previous[m.queryIdx].pt.x)];
            if (d == 0)   // bad depth
                continue;
            float dd = d * depthScale;
            cv::Point2d p1 = pixel2cam(keypoints_previous[m.queryIdx].pt, K);
            objectPoints.push_back(cv::Point3f(p1.x * dd, p1.y * dd, dd));
            imagePoints.push_back(keypoints_current[m.trainIdx].pt);
        }

        cv::solvePnP(objectPoints, imagePoints, K, cv::Mat(), r, t, false);
        // cv::Rodrigues(r, R);
        // std::cout << "R=" << std::endl << R << std::endl;
        // std::cout << "t=" << std::endl << t << std::endl;
        camera_pose.x = t.at<double>(0);
        camera_pose.y = t.at<double>(1);
        camera_pose.yaw = r.at<double>(2);
        camera_pose_pub.publish(camera_pose);
    }

    cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K) {
        return cv::Point2d
        (
            (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
            (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
        );
    }

    void clear()
    {
        keypoints_current.clear();
        keypoints_previous.clear();
        matches.clear();
        good_matches.clear();
        points1.clear();
        points2.clear();
        objectPoints.clear();
        imagePoints.clear();
    }

private:
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub;
    message_filters::Subscriber<sensor_msgs::Image> color_sub;
    message_filters::Synchronizer<MySyncPolicy> sync;

    cv::Mat current_rgb_image, previous_rgb_image, current_depth_image, previous_depth_image;

    cv::Ptr<cv::ORB> orb_detector_;
    std::vector<cv::KeyPoint> keypoints_current, keypoints_previous;
    cv::Mat descriptors_current, descriptors_previous;
    cv::BFMatcher matcher;
    std::vector<cv::DMatch> matches;
    std::vector<cv::DMatch> good_matches;
    std::vector<cv::Point2f> points1, points2;
    
    cv::Mat K = (cv::Mat_<double>(3, 3) << 277.2, 0, 160.5, 0, 277.2, 120.5, 0, 0, 1);
    double depthScale = 0.001f;
    std::vector<cv::Point3f> objectPoints;
    std::vector<cv::Point2f> imagePoints;
    cv::Mat r, t, R;

    laser_odom::my_pose camera_pose;
    ros::Publisher camera_pose_pub;

    laser_odom::count keypoints_count;
    ros::Publisher keypoints_count_pub;

    bool init = false;
};
