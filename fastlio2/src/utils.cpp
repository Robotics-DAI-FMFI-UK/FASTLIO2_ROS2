#include "utils.h"

//******************** PALO
#include <cmath>

/*
pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::livox2PCL(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg, int filter_num, double min_range, double max_range)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    int point_num = msg->point_num;
    cloud->reserve(point_num / filter_num + 1);
    for (int i = 0; i < point_num; i += filter_num)
    {
        if ((msg->points[i].line < 4) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
        {

            float x = msg->points[i].x;
            float y = msg->points[i].y;
            float z = msg->points[i].z;
            if (x * x + y * y + z * z < min_range * min_range || x * x + y * y + z * z > max_range * max_range)
                continue;
            pcl::PointXYZINormal p;
            p.x = x;
            p.y = y;
            p.z = z;
            p.intensity = msg->points[i].reflectivity;
            p.curvature = msg->points[i].offset_time / 1000000.0f;
            cloud->push_back(p);
        }
    }
    return cloud;
}
*/

pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::pointCloud2ToPCL(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    int filter_num,
    double min_range,
    double max_range)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZINormal>);

    const std::size_t point_num =
        static_cast<std::size_t>(msg->width) * msg->height;

    cloud->reserve(point_num / filter_num + 1);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(
        *msg, "intensity");
    sensor_msgs::PointCloud2ConstIterator<float> iter_time(*msg, "time");

    const double min_range_squared = min_range * min_range;
    const double max_range_squared = max_range * max_range;

    for (std::size_t i = 0;
         i < point_num;
         ++i, ++iter_x, ++iter_y, ++iter_z,
         ++iter_intensity, ++iter_time)
    {
        if (i % filter_num != 0)
            continue;

        const float x = *iter_x;
        const float y = *iter_y;
        const float z = *iter_z;

        if (!std::isfinite(x) ||
            !std::isfinite(y) ||
            !std::isfinite(z) ||
            !std::isfinite(*iter_time))
        {
            continue;
        }

        const double range_squared = x * x + y * y + z * z;

        if (range_squared < min_range_squared ||
            range_squared > max_range_squared)
        {
            continue;
        }

        pcl::PointXYZINormal point;
        point.x = x;
        point.y = y;
        point.z = z;
        point.intensity = *iter_intensity;

	// Unitree's PointCloud2 "time" field is nanoseconds.
        // FASTLIO2 stores the point offset in milliseconds in curvature.
        point.curvature = *iter_time / 1000000.0f;

        cloud->push_back(point);
    }

    return cloud;
}

//************************



double Utils::getSec(std_msgs::msg::Header &header)
{
    return static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec) * 1e-9;
}
builtin_interfaces::msg::Time Utils::getTime(const double &sec)
{
    builtin_interfaces::msg::Time time_msg;
    time_msg.sec = static_cast<int32_t>(sec);
    time_msg.nanosec = static_cast<uint32_t>((sec - time_msg.sec) * 1e9);
    return time_msg;
}
