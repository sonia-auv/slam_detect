#include "slam_detect/SlamDetect.hpp"
#include <sensor_msgs/msg/point_field.hpp>
using std::placeholders::_1;

SlamDetect::SlamDetect() : Node("sonia_slam_detect"), _log_counter(0)
{
    _observations = this->create_subscription<sensor_msgs::msg::PointCloud2>("/visual_slam/vis/observations_cloud", 10, std::bind(&SlamDetect::observationCallback, this, _1));
}


void SlamDetect::observationCallback(const std::shared_ptr<sensor_msgs::msg::PointCloud2> msg)
{
    // RCLCPP_INFO_STREAM(this->get_logger(), "Size of fields list: " << msg->fields.size() );
    std::vector<Point> points;
    for (size_t i = 0; i < msg->row_step; i+=msg->point_step)
    {
        size_t index = i;
        byte_to_float float_convert;
        byte_to_uint uint_convert;
        Point p;
        // X
        float_convert.data[0] = msg->data[index];
        float_convert.data[1] = msg->data[index + 1];
        float_convert.data[2] = msg->data[index + 2];
        float_convert.data[3] = msg->data[index + 3];
        
        p.x_val = float_convert.value;
        
        // y
        float_convert.data[0] = msg->data[index + 4];
        float_convert.data[1] = msg->data[index + 5];
        float_convert.data[2] = msg->data[index + 6];
        float_convert.data[3] = msg->data[index + 7];
        
        p.y_val = float_convert.value;
        
        // z
        float_convert.data[0] = msg->data[index + 8];
        float_convert.data[1] = msg->data[index + 9];
        float_convert.data[2] = msg->data[index + 10];
        float_convert.data[3] = msg->data[index + 11];
        
        p.z_val = float_convert.value;
        
        
        // rgb
        uint_convert.data[0] = msg->data[index + 12];
        uint_convert.data[1] = msg->data[index + 13];
        uint_convert.data[2] = msg->data[index + 14];
        uint_convert.data[3] = msg->data[index + 15];
        
        p.rgb_val = uint_convert.value;
        
        points.push_back(p);
    }
    
    // RCLCPP_INFO_STREAM(this->get_logger(), "Size of detected points " << points.size());
    if (points.size() > 0)
        //simpleDisplay(points);
}

void SlamDetect::sortX(std::vector<Point> &points){
    sort(points.begin(), points.end(), [](const Point &a, const Point &b) { return abs(a.x_val) < abs(b.x_val); });
}

void SlamDetect::sortY(std::vector<Point> &points){
    sort(points.begin(), points.end(), [](const Point &a, const Point &b) { return abs(a.y_val) < abs(b.y_val); });
}

void SlamDetect::sortZ(std::vector<Point> &points){
    sort(points.begin(), points.end(), [](const Point &a, const Point &b) { return abs(a.z_val) < abs(b.z_val); });
}

void SlamDetect::sort3D(std::vector<Point> &points){
    sort(points.begin(), points.end(), [](const Point &a, const Point &b) { 
        float d1 = sqrt(pow(a.x_val, 2) + pow(a.y_val, 2) + pow(a.z_val, 2));
        float d2 = sqrt(pow(b.x_val, 2) + pow(b.y_val, 2) + pow(b.z_val, 2));
        return d1 < d2;
    });
}

void SlamDetect::simpleDisplay(std::vector<Point> points){
    int numIt = 1;
    for (auto p : points){
        RCLCPP_INFO_STREAM(this->get_logger(), "x value : " << p.x_val);
        RCLCPP_INFO_STREAM(this->get_logger(), "y value : " << p.y_val);
        RCLCPP_INFO_STREAM(this->get_logger(), "z value : " << p.z_val);
        RCLCPP_INFO_STREAM(this->get_logger(), "number of iterations : " << numIt);
        RCLCPP_INFO_STREAM(this->get_logger(), "");
        numIt++;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "observations point cloud sorted");    
}

void SlamDetect::screenDisplay(std::vector<Point> points)
{
    // Find the corner points: smallest x + smallest y, smallest x + biggest y, biggest x + smallest y, biggest x + biggest y
    
    Point sxsy = points[0];
    Point sxby = points[0];
    Point bxsy = points[0];
    Point bxby = points[0];
    // RCLCPP_INFO_STREAM(this->get_logger(), "inititial points defined");

    for (size_t i = 0; i < points.size(); i++)
    {
        Point p = points.at(i);
        
        // Check against sxsy
        if (p.x_val <= sxsy.x_val && p.y_val <= sxsy.y_val)
        {
            sxsy = p;
        }

        // Check against sxby
        if (p.x_val <= sxby.x_val && p.y_val >= sxby.y_val)
        {
            sxby = p;
        }

        // Check against bxsy
        if (p.x_val >= bxsy.x_val && p.y_val <= bxsy.y_val)
        {
            bxsy = p;
        }

        // Check against bxby
        if (p.x_val >= bxby.x_val && p.y_val >= bxby.y_val)
        {
            bxby = p;
        }
    }
    
    if (_log_counter % 10 == 0)
    {
        _log_counter = 0;
        RCLCPP_INFO_STREAM(this->get_logger(), "sxsy, x: " << sxsy.x_val << " y: " << sxsy.y_val << " z: " << sxsy.z_val);
        RCLCPP_INFO_STREAM(this->get_logger(), "sxby, x: " << sxby.x_val << " y: " << sxby.y_val << " z: " << sxby.z_val);
        RCLCPP_INFO_STREAM(this->get_logger(), "bxsy, x: " << bxsy.x_val << " y: " << bxsy.y_val << " z: " << bxsy.z_val);
        RCLCPP_INFO_STREAM(this->get_logger(), "bxby, x: " << bxby.x_val << " y: " << bxby.y_val << " z: " << bxby.z_val);
        RCLCPP_INFO_STREAM(this->get_logger(), "");
    }
    
    _log_counter++;

    // Finding the points at the edges -> smallest X, largest X, smallest y, largest y (zed irrelevent as it is distance)
}

bool SlamDetect::log_check()
{
    if (_log_counter % 10 == 0)
    {
        _log_counter = 1;
        return true;
    }
    return false;
}