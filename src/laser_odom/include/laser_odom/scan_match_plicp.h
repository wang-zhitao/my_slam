#ifndef LESSON2_SCAN_MATCH_PLICP
#define LESSON2_SCAN_MATCH_PLICP

#include <cmath>
#include <vector>
#include <chrono>
#include <memory>

// ros
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <laser_odom/my_pose.h>

// tf2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// csm
#include <csm/csm_all.h>
#undef min
#undef max

class ScanMatchPLICP
{
private:
    ros::NodeHandle nh;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, geometry_msgs::PoseWithCovarianceStamped> MySyncPolicy;
    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub;
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> odom_ekf_sub;
    message_filters::Synchronizer<MySyncPolicy> sync;

    
    tf2::Quaternion prev_tf_quaternion;
    tf2::Quaternion curr_tf_quaternion;
    double roll, pitch, curr_yaw,prev_yaw;
    float odom_ekf_pose[3];
    laser_odom::my_pose scan_pose;
    ros::Publisher scan_pose_pub;
    
    bool initialized_;
    std::vector<double> a_cos_;
    std::vector<double> a_sin_;
    
    sm_params input_;
    sm_result output_;
    LDP prev_ldp_scan_;
    geometry_msgs::PoseWithCovarianceStamped prev_odom_ekf;


public:
    ScanMatchPLICP() : nh("~"),
                       scan_sub(nh, "/scan", 1),
                       odom_ekf_sub(nh, "/robot_pose_ekf/odom_combined", 1),
                       sync(MySyncPolicy(10), scan_sub, odom_ekf_sub),
                       initialized_(false)
    {
        sync.registerCallback(boost::bind(&ScanMatchPLICP::ScanCallback, this, _1, _2));
        scan_pose_pub = nh.advertise<laser_odom::my_pose>("/scan_pose",1);
        InitParams();
        ROS_INFO_STREAM("\033[1;32m----> Scan Match with PLICP started.\033[0m");
    }

    ~ScanMatchPLICP()
    {
        
    }

    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &odom_ekf){
        // 如果是第一帧数据，首先进行初始化
        if (!initialized_)
        {
            // 将雷达各个角度的sin与cos值保存下来，以节约计算量
            CreateCache(scan_msg);

            // 将 prev_ldp_scan_,last_icp_time_ 初始化
            LaserScanToLDP(scan_msg, prev_ldp_scan_);
            prev_odom_ekf = *odom_ekf;
            initialized_ = true;
            return;
        }

        // step1 进行数据类型转换
        

        static LDP curr_ldp_scan;
        LaserScanToLDP(scan_msg, curr_ldp_scan);

        static geometry_msgs::PoseWithCovarianceStamped curr_odom_ekf;
        
        odom_ekf_pose[0] = odom_ekf->pose.pose.position.x - prev_odom_ekf.pose.pose.position.x;
        odom_ekf_pose[1] = odom_ekf->pose.pose.position.y - prev_odom_ekf.pose.pose.position.y;
        tf2::fromMsg(odom_ekf->pose.pose.orientation, curr_tf_quaternion);
        tf2::fromMsg(prev_odom_ekf.pose.pose.orientation, prev_tf_quaternion);
        tf2::Matrix3x3(curr_tf_quaternion).getRPY(roll, pitch, curr_yaw);
        tf2::Matrix3x3(prev_tf_quaternion).getRPY(roll, pitch, prev_yaw);
        
        odom_ekf_pose[2] = curr_yaw - prev_yaw;


        // ROS_INFO("%.2f",curr_yaw);
        // step2 使用PLICP计算雷达前后两帧间的坐标变换

        ScanMatchWithPLICP(curr_ldp_scan, odom_ekf_pose);
        
        scan_pose.header.stamp = scan_msg->header.stamp;
        scan_pose.x = output_.x[0];
        scan_pose.y = output_.x[1];
        scan_pose.yaw = output_.x[2] * 180 / M_PI;
        scan_pose_pub.publish(scan_pose);
        prev_odom_ekf = *odom_ekf;
    }

    /**
     * 雷达数据间的角度是固定的，因此可以将对应角度的cos与sin值缓存下来，不用每次都计算
     */
    void CreateCache(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
    {
        a_cos_.clear();
        a_sin_.clear();
        double angle;

        for (unsigned int i = 0; i < scan_msg->ranges.size(); i++)
        {
            angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            a_cos_.push_back(cos(angle));
            a_sin_.push_back(sin(angle));
        }

        input_.min_reading = scan_msg->range_min;
        input_.max_reading = scan_msg->range_max;
    }

    /**
     * 将雷达的数据格式转成 csm 需要的格式
     */
    void LaserScanToLDP(const sensor_msgs::LaserScan::ConstPtr &scan_msg, LDP &ldp)
    {
        unsigned int n = scan_msg->ranges.size();
        // 调用csm里的函数进行申请空间
        ldp = ld_alloc_new(n);

        for (unsigned int i = 0; i < n; i++)
        {
            // calculate position in laser frame
            double r = scan_msg->ranges[i];

            if (r > scan_msg->range_min && r < scan_msg->range_max)
            {
                // 填充雷达数据
                ldp->valid[i] = 1;
                ldp->readings[i] = r;
            }
            else
            {
                ldp->valid[i] = 0;
                ldp->readings[i] = -1; // for invalid range
            }

            ldp->theta[i] = scan_msg->angle_min + i * scan_msg->angle_increment;
            ldp->cluster[i] = -1;
        }

        ldp->min_theta = ldp->theta[0];
        ldp->max_theta = ldp->theta[n - 1];

        ldp->odometry[0] = 0.0;
        ldp->odometry[1] = 0.0;
        ldp->odometry[2] = 0.0;

        ldp->estimate[0] = 0.0;
        ldp->estimate[1] = 0.0;
        ldp->estimate[2] = 0.0;

        ldp->true_pose[0] = 0.0;
        ldp->true_pose[1] = 0.0;
        ldp->true_pose[2] = 0.0;
    }

    /**
     * 使用PLICP进行帧间位姿的计算
     */
    void ScanMatchWithPLICP(LDP &curr_ldp_scan, float odom_ekf_pose[])
    {
        // CSM is used in the following way:
        // The scans are always in the laser frame
        // The reference scan (prevLDPcan_) has a pose of [0, 0, 0]
        // The new scan (currLDPScan) has a pose equal to the movement
        // of the laser in the laser frame since the last scan
        // The computed correction is then propagated using the tf machinery

        input_.laser_ref = prev_ldp_scan_;
        input_.laser_sens = curr_ldp_scan;

        // 位姿的预测值为0，就是不进行预测
        input_.first_guess[0] = odom_ekf_pose[0];
        input_.first_guess[1] = odom_ekf_pose[1];
        input_.first_guess[2] = odom_ekf_pose[2];

        // input_.first_guess[0] = 0;
        // input_.first_guess[1] = 0;
        // input_.first_guess[2] = 0;

        // 调用csm里的函数进行plicp计算帧间的匹配，输出结果保存在output里
        sm_icp(&input_, &output_);

        // if (output_.valid)
        // {
        //     std::cout << "transfrom: (" << output_.x[0] << ", " << output_.x[1] << ", " 
        //         << output_.x[2] * 180 / M_PI << ")" << std::endl;
        // }
        // else
        // {
        //     std::cout << "not Converged" << std::endl;
        // }

        // 删除prev_ldp_scan_，用curr_ldp_scan进行替代
        ld_free(prev_ldp_scan_);
        prev_ldp_scan_ = curr_ldp_scan;
    }

    /*
    * csm的参数初始化
    */
    void InitParams()
    {
        // **** CSM 的参数 - comments copied from algos.h (by Andrea Censi)
    // ...

    ROS_INFO("init");
        // Maximum angular displacement between scans
        if (!nh.getParam("max_angular_correction_deg", input_.max_angular_correction_deg))
            input_.max_angular_correction_deg = 45.0;

        // Maximum translation between scans (m)
        if (!nh.getParam("max_linear_correction", input_.max_linear_correction))
            input_.max_linear_correction = 1.0;

        // Maximum ICP cycle iterations
        if (!nh.getParam("max_iterations", input_.max_iterations))
            input_.max_iterations = 10;

        // A threshold for stopping (m)
        if (!nh.getParam("epsilon_xy", input_.epsilon_xy))
            input_.epsilon_xy = 0.000001;

        // A threshold for stopping (rad)
        if (!nh.getParam("epsilon_theta", input_.epsilon_theta))
            input_.epsilon_theta = 0.000001;

        // Maximum distance for a correspondence to be valid
        if (!nh.getParam("max_correspondence_dist", input_.max_correspondence_dist))
            input_.max_correspondence_dist = 1.0;

        // Noise in the scan (m)
        if (!nh.getParam("sigma", input_.sigma))
            input_.sigma = 0.010;

        // Use smart tricks for finding correspondences.
        if (!nh.getParam("use_corr_tricks", input_.use_corr_tricks))
            input_.use_corr_tricks = 1;

        // Restart: Restart if error is over threshold
        if (!nh.getParam("restart", input_.restart))
            input_.restart = 0;

        // Restart: Threshold for restarting
        if (!nh.getParam("restart_threshold_mean_error", input_.restart_threshold_mean_error))
            input_.restart_threshold_mean_error = 0.01;

        // Restart: displacement for restarting. (m)
        if (!nh.getParam("restart_dt", input_.restart_dt))
            input_.restart_dt = 1.0;

        // Restart: displacement for restarting. (rad)
        if (!nh.getParam("restart_dtheta", input_.restart_dtheta))
            input_.restart_dtheta = 0.1;

        // Max distance for staying in the same clustering
        if (!nh.getParam("clustering_threshold", input_.clustering_threshold))
            input_.clustering_threshold = 0.25;

        // Number of neighbour rays used to estimate the orientation
        if (!nh.getParam("orientation_neighbourhood", input_.orientation_neighbourhood))
            input_.orientation_neighbourhood = 20;

        // If 0, it's vanilla ICP
        if (!nh.getParam("use_point_to_line_distance", input_.use_point_to_line_distance))
            input_.use_point_to_line_distance = 1;

        // Discard correspondences based on the angles
        if (!nh.getParam("do_alpha_test", input_.do_alpha_test))
            input_.do_alpha_test = 0;

        // Discard correspondences based on the angles - threshold angle, in degrees
        if (!nh.getParam("do_alpha_test_thresholdDeg", input_.do_alpha_test_thresholdDeg))
            input_.do_alpha_test_thresholdDeg = 20.0;

        // Percentage of correspondences to consider: if 0.9,
        // always discard the top 10% of correspondences with more error
        if (!nh.getParam("outliers_maxPerc", input_.outliers_maxPerc))
            input_.outliers_maxPerc = 0.90;

        // Parameters describing a simple adaptive algorithm for discarding.
        //  1) Order the errors.
        //  2) Choose the percentile according to outliers_adaptive_order.
        //     (if it is 0.7, get the 70% percentile)
        //  3) Define an adaptive threshold multiplying outliers_adaptive_mult
        //     with the value of the error at the chosen percentile.
        //  4) Discard correspondences over the threshold.
        //  This is useful to be conservative; yet remove the biggest errors.
        if (!nh.getParam("outliers_adaptive_order", input_.outliers_adaptive_order))
            input_.outliers_adaptive_order = 0.7;

        if (!nh.getParam("outliers_adaptive_mult", input_.outliers_adaptive_mult))
            input_.outliers_adaptive_mult = 2.0;

        // If you already have a guess of the solution, you can compute the polar angle
        // of the points of one scan in the new position. If the polar angle is not a monotone
        // function of the readings index, it means that the surface is not visible in the
        // next position. If it is not visible, then we don't use it for matching.
        if (!nh.getParam("do_visibility_test", input_.do_visibility_test))
            input_.do_visibility_test = 0;

        // no two points in laser_sens can have the same corr.
        if (!nh.getParam("outliers_remove_doubles", input_.outliers_remove_doubles))
            input_.outliers_remove_doubles = 1;

        // If 1, computes the covariance of ICP using the method http://purl.org/censi/2006/icpcov
        if (!nh.getParam("do_compute_covariance", input_.do_compute_covariance))
            input_.do_compute_covariance = 0;

        // Checks that find_correspondences_tricks gives the right answer
        if (!nh.getParam("debug_verify_tricks", input_.debug_verify_tricks))
            input_.debug_verify_tricks = 0;

        // If 1, the field 'true_alpha' (or 'alpha') in the first scan is used to compute the
        // incidence beta, and the factor (1/cos^2(beta)) used to weight the correspondence.");
        if (!nh.getParam("use_ml_weights", input_.use_ml_weights))
            input_.use_ml_weights = 0;

        // If 1, the field 'readings_sigma' in the second scan is used to weight the
        // correspondence by 1/sigma^2
        if (!nh.getParam("use_sigma_weights", input_.use_sigma_weights))
            input_.use_sigma_weights = 0;
    }
};

#endif