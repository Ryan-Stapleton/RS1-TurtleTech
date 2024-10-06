#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

class ObjDetectionNode : public rclcpp::Node {
public:
    ObjDetectionNode()
    : Node("object_detection_node") {
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                            "/scan", 10, std::bind(&ObjDetectionNode::scanCallback, this, std::placeholders::_1));
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
                            "/odom", 10, std::bind(&ObjDetectionNode::odomCallback, this, std::placeholders::_1));

        // Load the maps
        gazebo_map_ = cv::imread("/home/student/ros2_ws/src/sprint_03/src/prebuilt_maps/map.pgm");

        original_gazebo_map_ = gazebo_map_.clone();

        map_scale_ = 0.05; //Map Resolution

        cv::namedWindow(WINDOW_Map, cv::WINDOW_AUTOSIZE);
        cv::namedWindow(WINDOW_LaserScan, cv::WINDOW_AUTOSIZE);
        cv::namedWindow(WINDOW_ObjDetect, cv::WINDOW_AUTOSIZE);

        cv::imshow(WINDOW_Map, gazebo_map_);
        cv::waitKey(1);

    }

private:
    // Laser scan callback to process the laser scan
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scanMsg) {
        // Convert laser scan to image (Image C)
        scan = *scanMsg;
        cv::Mat laser_image = laserScanToMat(scanMsg);
        cv::rotate(laser_image, laser_image, cv::ROTATE_90_COUNTERCLOCKWISE);

        cv::imshow(WINDOW_LaserScan, laser_image);
        cv::waitKey(1);

        objDetection(scan);
    }

        // Odometry callback to track robot's movement
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMsg) {
        odom_ = *odomMsg;
    }

    void objDetection(const sensor_msgs::msg::LaserScan Scan) {
        // Set parameters for object detection
        float object_diameter = 0.30;  // 30 cm
        float tolerance = 0.06;        // 2 cm tolerance
        float min_radius = (object_diameter - tolerance) / 2.0;  // Min radius in meters
        float max_radius = (object_diameter + tolerance) / 2.0;  // Max radius in meters

        // Define the range limits based on 10% of the laser's max distance
        float min_distance = 1.0 * Scan.range_min;  // 90% of max range
        float max_distance = 0.2 * Scan.range_max;  // 100% of max range

        std::vector<cv::Point2f> laser_points;

        // Extract the robot's position and orientation from odometry
        float robot_x = odom_.pose.pose.position.x;
        float robot_y = odom_.pose.pose.position.y;

        // Convert orientation from quaternion to yaw (heading)
        tf2::Quaternion quat(
            odom_.pose.pose.orientation.x,
            odom_.pose.pose.orientation.y,
            odom_.pose.pose.orientation.z,
            odom_.pose.pose.orientation.w
        );
        tf2::Matrix3x3 mat(quat);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);

        // Iterate over the laser scan ranges to collect valid points
        for (size_t i = 0; i < Scan.ranges.size(); ++i) {
            float range = Scan.ranges[i];
            if (range > min_distance && range < max_distance) {
                float angle = Scan.angle_min + i * Scan.angle_increment;

                // Convert polar coordinates to Cartesian coordinates (relative to the robot)
                float local_x = range * cos(angle);
                float local_y = range * sin(angle);

                // Transform local coordinates to global map coordinates
                float global_x = robot_x + (local_x * cos(yaw) - local_y * sin(yaw));
                float global_y = -robot_y + (local_x * sin(yaw) + local_y * cos(yaw));

                laser_points.push_back(cv::Point2f(global_x, global_y));
            }
        }

        if (laser_points.size() < 3) {
            RCLCPP_WARN(this->get_logger(), "Not enough points detected.");
            return;
        }

        // Use OpenCV to find the minimum enclosing circle for the detected points
        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(laser_points, center, radius);

        // RCLCPP_INFO(this->get_logger(), "Detected cylindrical object with radius: %f", radius);

        // Check if the detected circle matches the object's size (within tolerance)
        if (radius >= min_radius && radius <= max_radius) {
            RCLCPP_INFO(this->get_logger(), "Detected cylindrical object with radius: %f", radius);
            
            // Reset the gazebo map by copying the original map (clearing previous detections)
            gazebo_map_ = original_gazebo_map_.clone();

            // Transform to map coordinates
            int map_x = static_cast<int>((center.x / map_scale_) + (gazebo_map_.cols / 2));
            int map_y = static_cast<int>((center.y / map_scale_) + (gazebo_map_.rows / 2));

            // Draw a circle on the map to represent the detected object
            int circle_radius_in_map = static_cast<int>((object_diameter / 2.0) / map_scale_);  // Convert object size to map scale
            cv::circle(gazebo_map_, cv::Point(map_x, map_y), circle_radius_in_map, cv::Scalar(0, 255, 0), 2);

            // Display the map with the object marked
            cv::imshow(WINDOW_ObjDetect, gazebo_map_);
            cv::waitKey(1);
        } else {
            RCLCPP_WARN(this->get_logger(), "No cylindrical object of expected size detected.");
        }
    }

    // Convert the laser scan to an OpenCV image (Image C)
    cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
        // Parameters
        int img_size = 500;
        float max_range = scan->range_max;
        cv::Mat image = cv::Mat::zeros(img_size, img_size, CV_8UC1);

        for (size_t i = 0; i < scan->ranges.size(); i++) {
            float range = scan->ranges[i];
            if (range > scan->range_min && range < scan->range_max) {
                float angle = scan->angle_min + i * scan->angle_increment;
                int x = static_cast<int>((range * cos(angle)) * img_size / (2 * max_range)) + img_size / 2;
                int y = static_cast<int>((range * sin(angle)) * img_size / (2 * max_range)) + img_size / 2;
                if (x >= 0 && x < img_size && y >= 0 && y < img_size) {
                    image.at<uchar>(y, x) = 255;
                }
            }
        }
        return image;
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    sensor_msgs::msg::LaserScan scan;
    nav_msgs::msg::Odometry odom_;

    cv::Mat gazebo_map_;
    cv::Mat original_gazebo_map_;

    double map_scale_;

    const std::string WINDOW_Map = "Map";
    const std::string WINDOW_LaserScan = "Laser Scan";
    const std::string WINDOW_ObjDetect = "Detected Object";

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjDetectionNode>());
    rclcpp::shutdown();
    return 0;
}