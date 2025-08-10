#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>


class SlamDetect : public rclcpp::Node
{
    public:
    SlamDetect();
    ~SlamDetect() = default;

    private:

    struct Point
    {
        float x_val;
        float y_val;
        float z_val;
        uint32_t rgb_val;
    };

    void observationCallback(const std::shared_ptr<sensor_msgs::msg::PointCloud2> msg);

    void screenDisplay(std::vector<Point> points);

    bool log_check();

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _observations;

    union byte_to_float {
        uint8_t data[4];
        float value;
    };

    union byte_to_uint {
        uint8_t data[4];
        uint32_t value;
    };

    int _log_counter;


};
