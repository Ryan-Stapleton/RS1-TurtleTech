#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

class RGBDepthDetect : public rclcpp::Node
{
public:
    RGBDepthDetect() : Node("rgbDetect")
    {
        // Subscriber for the RGB camera
        rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&RGBDepthDetect::rgb_callback, this, std::placeholders::_1));

        // Subscriber for the depth camera
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw", 10, std::bind(&RGBDepthDetect::depth_callback, this, std::placeholders::_1));

        // Subscriber for the odometry data
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&RGBDepthDetect::odom_callback, this, std::placeholders::_1));

    }

private:
    void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            std::lock_guard<std::mutex> lock(image_mutex_); // Ensure thread-safe access

            // Convert the ROS image message to an OpenCV image
            cv::Mat input_image = cv_bridge::toCvCopy(msg, "bgr8")->image;

            // Apply a Gaussian blur to the image


            //BLLUR MUST BE ODD NUMBER 3 5 7 9 ect
            int blur_kernel_size = 7; // Adjust this value to control the blur level (must be an odd number)
            //BLLUR MUST BE ODD 3 5 7 9 ect
            
            cv::Mat blurred_image;
            cv::GaussianBlur(input_image, blurred_image, cv::Size(blur_kernel_size, blur_kernel_size), 0);

            // Save the blurred image to rgb_image_
            rgb_image_ = blurred_image;
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not convert RGB image: %s", e.what());
        }
    }


    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            std::lock_guard<std::mutex> lock(image_mutex_); // Ensure thread-safe access
            depth_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not convert Depth image: %s", e.what());
        }
    }

    cv::Mat detectBlue()
    {
        std::lock_guard<std::mutex> lock(image_mutex_); // Ensure thread-safe access

        // Extract the robot's position and orientation from odometry
        float robot_x = odom_.pose.pose.position.x;
        float robot_y = odom_.pose.pose.position.y;
        double yaw = tf2::getYaw(odom_.pose.pose.orientation);

        cv::Mat result_image = rgb_image_.clone(); // Clone the original image to draw bounding boxes

        if (!rgb_image_.empty() && !depth_image_.empty())
        {
            cv::Mat hsv_image, blue_mask;
            
            // Convert the RGB image to HSV color space
            cv::cvtColor(rgb_image_, hsv_image, cv::COLOR_BGR2HSV);
            
            // Define the range for blue color in HSV
            cv::Scalar lower_blue(100, 150, 50); // Lower bound of blue (can be adjusted)
            cv::Scalar upper_blue(140, 255, 255); // Upper bound of blue (can be adjusted)

            // Create a mask for blue pixels
            cv::inRange(hsv_image, lower_blue, upper_blue, blue_mask);
            
            // Find contours in the blue mask
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(blue_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            
            // Sort the contours by area in descending order
            std::sort(contours.begin(), contours.end(), [](const std::vector<cv::Point> &c1, const std::vector<cv::Point> &c2) {
                return cv::contourArea(c1) > cv::contourArea(c2);
            });

            // Keep only the 4 largest contours (if there are that many)
            if (contours.size() > 4) {
                contours.resize(4);
            }

            if (!contours.empty()) {
                // Get the largest contour (first one after sorting)
                const auto& largest_contour = contours[0];
                
                // Get the bounding box for the largest contour
                cv::Rect bounding_box = cv::boundingRect(largest_contour);
                
                // Draw the bounding box on the result image
                cv::rectangle(result_image, bounding_box, cv::Scalar(0, 255, 0), 2); // Green box
                
                // Find the center of the top half of the bounding box
                int center_x = bounding_box.x + bounding_box.width / 2;
                int top_half_center_y = bounding_box.y + bounding_box.height / 4; // Center of the top half
                
                // Map the RGB image center coordinates to the depth image coordinates
                int depth_x = static_cast<int>((center_x / static_cast<float>(rgb_image_.cols)) * depth_image_.cols);
                int depth_y = static_cast<int>((top_half_center_y / static_cast<float>(rgb_image_.rows)) * depth_image_.rows);

                // Retrieve the raw depth value at the mapped coordinates
                uint16_t raw_depth_value = depth_image_.at<uint16_t>(depth_y, depth_x);

                // Calculate the angle based on the horizontal position
                int img_center_x = rgb_image_.cols / 2; // Center of the image
                float angle = ((center_x - img_center_x) / static_cast<float>(img_center_x)) * 30.0f;

                // Convert angle to radians
                float angle_rad = angle * (M_PI / 180.0f);
                
                // Convert to cartesian 
                float x = raw_depth_value * cos(angle_rad); // Please note raw_depth_values are already in meters and thus dont require conversion. 
                float y = raw_depth_value * sin(angle_rad);

                // Print the angle and raw depth value
                RCLCPP_INFO(this->get_logger(), "Local Coordinate: Blue person detected at angle %.2f degrees with raw depth value: %d meters. Position in robot frame: x=%.2f m, y=%.2f m.",
                angle, raw_depth_value, x, y);

                // Transform local coordinates to global map coordinates
                float global_x = robot_x + (x * cos(yaw) + y * sin(yaw));
                float global_y = robot_y + (x * sin(yaw) + y * cos(yaw));

                RCLCPP_INFO(this->get_logger(), "Global Coordinate: Blue person detected, position in global coordinate frame: x=%.2f m, y=%.2f m.",
                global_x, global_y);
            }
        }

        return result_image;
    }



    cv::Mat detectRed()
    {
        std::lock_guard<std::mutex> lock(image_mutex_); // Ensure thread-safe access

        // Extract the robot's position and orientation from odometry
        float robot_x = odom_.pose.pose.position.x;
        float robot_y = odom_.pose.pose.position.y;
        double yaw = tf2::getYaw(odom_.pose.pose.orientation);

        cv::Mat result_image = rgb_image_.clone(); // Clone the original image to draw bounding boxes

        if (!rgb_image_.empty() && !depth_image_.empty())
        {
            cv::Mat hsv_image, red_mask;
            
            // Convert the RGB image to HSV color space
            cv::cvtColor(rgb_image_, hsv_image, cv::COLOR_BGR2HSV);
            
            // Define the range for red color in HSV (tighter ranges)
            cv::Scalar lower_red1(0, 180, 100);    // Narrower lower bound of red (first range)
            cv::Scalar upper_red1(8, 255, 255);    // Narrower upper bound of red (first range)
            cv::Scalar lower_red2(170, 180, 100);  // Narrower lower bound of red (second range)
            cv::Scalar upper_red2(180, 255, 255);  // Narrower upper bound of red (second range)

            // Create two masks for red pixels (to handle red hue wrapping around 0 degrees)
            cv::Mat red_mask1, red_mask2;
            cv::inRange(hsv_image, lower_red1, upper_red1, red_mask1);
            cv::inRange(hsv_image, lower_red2, upper_red2, red_mask2);

            // Combine the two masks
            red_mask = red_mask1 | red_mask2;

            // Find contours in the red mask
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            
            // Sort the contours by area in descending order
            std::sort(contours.begin(), contours.end(), [](const std::vector<cv::Point> &c1, const std::vector<cv::Point> &c2) {
                return cv::contourArea(c1) > cv::contourArea(c2);
            });

            // Keep only the 4 largest contours (if there are that many)
            if (contours.size() > 4) {
                contours.resize(4);
            }

            if (!contours.empty()) {
                // Get the largest contour (first one after sorting)
                const auto& largest_contour = contours[0];
                
                // Get the bounding box for the largest contour
                cv::Rect bounding_box = cv::boundingRect(largest_contour);
                
                // Draw the bounding box on the result image
                cv::rectangle(result_image, bounding_box, cv::Scalar(0, 255, 0), 2); // Green box
                
                // Find the center of the top half of the bounding box
                int center_x = bounding_box.x + bounding_box.width / 2;
                int top_half_center_y = bounding_box.y + bounding_box.height / 4; // Center of the top half
                
                // Map the RGB image center coordinates to the depth image coordinates
                int depth_x = static_cast<int>((center_x / static_cast<float>(rgb_image_.cols)) * depth_image_.cols);
                int depth_y = static_cast<int>((top_half_center_y / static_cast<float>(rgb_image_.rows)) * depth_image_.rows);

                // Retrieve the raw depth value at the mapped coordinates
                uint16_t raw_depth_value = depth_image_.at<uint16_t>(depth_y, depth_x);

                // Calculate the angle based on the horizontal position
                int img_center_x = rgb_image_.cols / 2; // Center of the image
                float angle = ((center_x - img_center_x) / static_cast<float>(img_center_x)) * 30.0f;

                // Convert angle to radians
                float angle_rad = angle * (M_PI / 180.0f);
                
                // Convert to cartesian 
                float x = raw_depth_value * cos(angle_rad); // Please note raw_depth_values are already in meters and thus don't require conversion. 
                float y = raw_depth_value * sin(angle_rad);

                // Print the angle and raw depth value
                RCLCPP_INFO(this->get_logger(), "Local Coordinate: Red person detected at angle %.2f degrees with raw depth value: %d meters. Position in robot frame: x=%.2f m, y=%.2f m.",
                angle, raw_depth_value, x, y);

                // Transform local coordinates to global map coordinates
                float global_x = robot_x + (x * cos(yaw) + y * sin(yaw));
                float global_y = robot_y + (x * sin(yaw) + y * cos(yaw));

                RCLCPP_INFO(this->get_logger(), "Global Coordinate: Red person detected, position in global coordinate frame: x=%.2f m, y=%.2f m.",
                global_x, global_y);
            }
        }

        return result_image;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_ = *msg;

        // Call the detectBlue function to process and get the result image
        cv::Mat result_image_blue = detectBlue();
        cv::Mat result_image_red = detectRed();

        // Show the result image with bounding boxes
        if (!result_image_blue.empty()) {
            cv::imshow("RGB Image with Blue Detection", result_image_blue);

            // Normalize the depth image to a range from 0 to 255 (8-bit image) for viewing
            cv::Mat scaled_depth_image;
            double minVal, maxVal;
            cv::minMaxLoc(depth_image_, &minVal, &maxVal);
            depth_image_.convertTo(scaled_depth_image, CV_8UC1, 255.0 / maxVal);

            cv::imshow("Scaled Depth Image", scaled_depth_image);
        }

        // Show the result image with bounding boxes
        if (!result_image_red.empty()) {
            cv::imshow("RGB Image with Red Detection", result_image_red);

            // Normalize the depth image to a range from 0 to 255 (8-bit image) for viewing
            cv::Mat scaled_depth_image;
            double minVal, maxVal;
            cv::minMaxLoc(depth_image_, &minVal, &maxVal);
            depth_image_.convertTo(scaled_depth_image, CV_8UC1, 255.0 / maxVal);

            cv::imshow("Scaled Depth Image", scaled_depth_image);
        }

        cv::waitKey(1);
    }

    // Member variables to store images
    cv::Mat rgb_image_;
    cv::Mat depth_image_;
    std::mutex image_mutex_; // Mutex for thread-safe access to images
    nav_msgs::msg::Odometry odom_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RGBDepthDetect>());
    rclcpp::shutdown();
    return 0;
}
