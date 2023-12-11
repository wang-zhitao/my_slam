#ifndef FEATURE_DETECTION_H_
#define FEATURE_DETECTION_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <map>
#include <vector>
#include <chrono>
#include <laser_odom/count.h>

#define max_scan_count 1500 // Maximum number of laser scan data points

struct smoothness_t
{
    float value;
    size_t index;
};

// Comparator for sorting
struct by_value
{
    bool operator()(smoothness_t const &left, smoothness_t const &right)
    {
        return left.value < right.value;
    }
};

class LaserScan
{
private:
    ros::NodeHandle node_handle_;
    ros::NodeHandle private_node_;
    ros::Subscriber laser_scan_subscriber_;
    ros::Publisher corner_count_pub;
    float edge_threshold_;
    laser_odom::count extracted_corner_count;

public:
    LaserScan() : private_node_("~"), edge_threshold_(1.0)
    {
        ROS_INFO_STREAM("\033[1;32m----> Feature Extraction Started.\033[0m");

        laser_scan_subscriber_ = node_handle_.subscribe("/scan", 1, &LaserScan::ScanCallback, this);
        corner_count_pub = node_handle_.advertise<laser_odom::count>("/corner_count", 1);
    }

    ~LaserScan()
    {
    }

    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
    {
        int scan_count = scan_msg->ranges.size();
        extracted_corner_count.a = 0;
        std::vector<smoothness_t> scan_smoothness(max_scan_count);
        std::vector<float> scan_curvature(max_scan_count);

        std::map<int, int> map_index;
        int count = 0;
        std::vector<float> new_scan(max_scan_count);


        for (int i = 0; i < scan_count; i++)
        {
            if (std::isfinite(scan_msg->ranges[i]))
            {
                new_scan[count] = scan_msg->ranges[i];
                map_index[count] = i;
                count++;
            }
        }

        // Calculate curvature values
        float diff_range_squared = 0.0;
        for (int i = 5; i < count - 5; i++)
        {
            diff_range_squared = pow(new_scan[i - 5] + new_scan[i - 4] +
                                         new_scan[i - 3] + new_scan[i - 2] +
                                         new_scan[i - 1] - new_scan[i] * 10 +
                                         new_scan[i + 1] + new_scan[i + 2] +
                                         new_scan[i + 3] + new_scan[i + 4] +
                                         new_scan[i + 5], 2);

            scan_curvature[i] = diff_range_squared;
            scan_smoothness[i].value = scan_curvature[i];
            scan_smoothness[i].index = i;
        }

        // Extract corners
        for (int j = 0; j < 6; j++)
        {
            int start_index = (0 * (6 - j) + count * j) / 6;
            int end_index = (0 * (5 - j) + count * (j + 1)) / 6 - 1;

            if (start_index < end_index)
            {
                std::partial_sort(scan_smoothness.begin() + start_index,
                                  scan_smoothness.begin() + start_index + 20,
                                  scan_smoothness.begin() + end_index, by_value());

                int largestPickedNum = 0;
                for (int k = end_index; k >= start_index; k--)
                {
                    int index = start_index + k;
                    if (scan_smoothness[k].value > edge_threshold_)
                    {
                        largestPickedNum++;
                        if (largestPickedNum <= 20)
                        {
                            extracted_corner_count.a++;
                        }
                        else
                        {
                            break;
                        }
                    }
                }
            }
        }

        extracted_corner_count.header.stamp = scan_msg->header.stamp;
        corner_count_pub.publish(extracted_corner_count);
    }
};

#endif
