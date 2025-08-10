#include <rclcpp/rclcpp.hpp>
#include <slam_detect/SlamDetect.hpp>

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SlamDetect>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
