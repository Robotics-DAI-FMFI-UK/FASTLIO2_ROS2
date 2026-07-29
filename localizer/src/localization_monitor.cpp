#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>

#include <interface/srv/relocalize.hpp>

using namespace std::chrono_literals;

class LocalizationMonitor : public rclcpp::Node
{
public:
    explicit LocalizationMonitor(const std::string &pcd_path)
        : Node("localization_monitor"),
          pcd_path_(pcd_path)
    {
        global_odom_sub_ =
            create_subscription<nav_msgs::msg::Odometry>(
                "/localizer/global_odom",
                10,
                std::bind(
                    &LocalizationMonitor::globalOdomCallback,
                    this,
                    std::placeholders::_1));

        tracking_valid_sub_ =
            create_subscription<std_msgs::msg::Bool>(
                "/localizer/tracking_valid",
                10,
                std::bind(
                    &LocalizationMonitor::trackingValidCallback,
                    this,
                    std::placeholders::_1));

        relocalize_client_ =
            create_client<interface::srv::Relocalize>(
                "/localizer/relocalize");

        print_timer_ = create_wall_timer(
            200ms,
            std::bind(
                &LocalizationMonitor::printStatus,
                this));
    }

    bool sendLocalizationRequest()
    {
        std::cout
            << "Waiting for /localizer/relocalize service..."
            << std::flush;

        while (rclcpp::ok() &&
               !relocalize_client_->wait_for_service(1s))
        {
            std::cout
                << "\rWaiting for /localizer/relocalize service..."
                << std::flush;
        }

        if (!rclcpp::ok())
            return false;

        auto request =
            std::make_shared<interface::srv::Relocalize::Request>();

        request->pcd_path = pcd_path_;

        request->x = 0.0;
        request->y = 0.0;
        request->z = 0.0;
        request->yaw = 0.0;
        request->pitch = M_PI;
        request->roll = 0.0;

        status_ = "REQUESTING";
        request_sent_ = true;

        relocalize_client_->async_send_request(
            request,
            std::bind(
                &LocalizationMonitor::localizationResponseCallback,
                this,
                std::placeholders::_1));

        return true;
    }

private:
    using RelocalizeFuture =
        rclcpp::Client<interface::srv::Relocalize>::SharedFuture;

    static double quaternionToYaw(
        const geometry_msgs::msg::Quaternion &q)
    {
        return std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    void localizationResponseCallback(
        RelocalizeFuture future)
    {
        try
        {
            const auto response = future.get();

            if (response->success)
            {
                // The map was loaded and the request accepted.
                // tracking_valid will report actual ICP convergence.
                status_ = "LOCALIZING";
            }
            else
            {
                status_ = "REQUEST REJECTED";

                RCLCPP_ERROR(
                    get_logger(),
                    "Relocalization rejected: %s",
                    response->message.c_str());
            }
        }
        catch (const std::exception &error)
        {
            status_ = "REQUEST FAILED";

            RCLCPP_ERROR(
                get_logger(),
                "Relocalization service call failed: %s",
                error.what());
        }
    }

    void globalOdomCallback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        z_ = msg->pose.pose.position.z;

        yaw_ = quaternionToYaw(
            msg->pose.pose.orientation) + M_PI;

        if (yaw_ > M_PI) yaw_ -= 2.0 * M_PI;

        have_pose_ = true;
    }

    void trackingValidCallback(
        const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data)
        {
            status_ = "VALID";
        }
        else if (request_sent_)
        {
            status_ = have_pose_
                ? "LOST"
                : "LOCALIZING";
        }
    }

    void printStatus()
    {
        std::cout << '\r'
                  << std::fixed
                  << std::setprecision(3);

        if (have_pose_)
        {
            const double yaw_degrees =
                yaw_ * 180.0 / M_PI;

            std::cout
                << "x=" << std::setw(8) << x_ << " m  "
                << "y=" << std::setw(8) << y_ << " m  "
                << "z=" << std::setw(7) << z_ << " m  "
                << std::setprecision(2)
                << "yaw=" << std::setw(7)
                << yaw_degrees << " deg  ";
        }
        else
        {
            std::cout
                << "x=   ---    "
                << "y=   ---    "
                << "z=   ---    "
                << "yaw=   ---       ";
        }

        std::cout
            << "status="
            << std::left
            << std::setw(16)
            << status_
            << std::right
            << std::flush;
    }

    std::string pcd_path_;
    std::string status_ = "WAITING";

    bool request_sent_ = false;
    bool have_pose_ = false;

    double x_ = 0.0;
    double y_ = 0.0;
    double z_ = 0.0;
    double yaw_ = 0.0;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        global_odom_sub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
        tracking_valid_sub_;

    rclcpp::Client<interface::srv::Relocalize>::SharedPtr
        relocalize_client_;

    rclcpp::TimerBase::SharedPtr print_timer_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc != 2)
    {
        std::cerr
            << "Usage: " << argv[0]
            << " MAP_FILE.pcd"
            << std::endl;

        rclcpp::shutdown();
        return 1;
    }

    std::filesystem::path pcd_path =
        std::filesystem::absolute(
            std::filesystem::path(argv[1]));

    if (!std::filesystem::is_regular_file(pcd_path))
    {
        std::cerr
            << "PCD map file not found: "
            << pcd_path
            << std::endl;

        rclcpp::shutdown();
        return 1;
    }

    auto node =
        std::make_shared<LocalizationMonitor>(
            pcd_path.string());

    if (!node->sendLocalizationRequest())
    {
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::spin(node);

    // Move to a fresh line after Ctrl-C.
    std::cout << std::endl;

    rclcpp::shutdown();
    return 0;
}
