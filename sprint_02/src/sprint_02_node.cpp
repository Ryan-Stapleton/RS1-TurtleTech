#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

class ScanMatchingNode : public rclcpp::Node {
public:
    ScanMatchingNode()
    : Node("scan_matching_node") {
        // Subscriptions for map, laser scan, and odometry
        map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&ScanMatchingNode::mapCallback, this, std::placeholders::_1));
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ScanMatchingNode::scanCallback, this, std::placeholders::_1));
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&ScanMatchingNode::odomCallback, this, std::placeholders::_1));

        // For robot movement control (if needed)
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Create OpenCV windows for visualization
        cv::namedWindow(WINDOW_ImageA, cv::WINDOW_AUTOSIZE);
        cv::namedWindow(WINDOW_ImageB, cv::WINDOW_AUTOSIZE);
        cv::namedWindow(WINDOW_ImageC, cv::WINDOW_AUTOSIZE);
        cv::namedWindow(WINDOW_Matches, cv::WINDOW_AUTOSIZE);
        cv::namedWindow(WINDOW_Localised, cv::WINDOW_AUTOSIZE);
    }

private:
    // Map callback to store the map and update the section around the robot
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr mapMsg) {

        RCLCPP_INFO(this->get_logger(), "Map received");
        map_ = mapMsg;  // Store map globally
        occupancyGridToImage(map_);
    }

    // Laser scan callback to process the laser scan
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scanMsg) {
        // Convert laser scan to image (Image C)
        scan = scanMsg;
        cv::Mat laser_image = laserScanToMat(scanMsg);
        cv::rotate(laser_image, laser_image, cv::ROTATE_90_COUNTERCLOCKWISE);

        cv::imshow(WINDOW_ImageC, laser_image);
        cv::waitKey(1);

        if (!map_edges.empty()) {
            // Perform feature matching between Image B (map edges) and Image C (laser scan)
            localiseTurtlebot(map_edges, laser_image);
        }
    }

    // Odometry callback to track robot's movement
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMsg) {
        odom_ = *odomMsg;
        updateMapSection();  // Update map section as robot moves
    }

    // Extract the map section around the robot, and apply edge detection with rotation based on orientation
    void updateMapSection() {
        if (!map_) return;  // Ensure map is available

        int section_size = 300;

        map_section = cv::Mat::zeros(section_size, section_size, CV_8UC1);

        // Robot's position and orientation in the map
        double robot_x = odom_.pose.pose.position.x;
        double robot_y = odom_.pose.pose.position.y;

        // Extract orientation (yaw) from quaternion
        tf2::Quaternion q(
            odom_.pose.pose.orientation.x,
            odom_.pose.pose.orientation.y,
            odom_.pose.pose.orientation.z,
            odom_.pose.pose.orientation.w
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);  // Get roll, pitch, yaw (in radians)
        double robot_yaw = yaw * 180.0 / CV_PI;  // Convert yaw to degrees

        // Convert robot's real-world coordinates to map coordinates
        int map_robot_x = static_cast<int>((robot_x - origin_x) / map_scale_);
        int map_robot_y = static_cast<int>((robot_y - origin_y) / map_scale_);

        int img_size = 130;
        int half_img_size = img_size / 2;

        // Calculate the top-left corner of the window in map coordinates
        int top_left_x = std::max(0, map_robot_x - half_img_size);
        int top_left_y = std::max(0, map_robot_y - half_img_size);

        // Ensure that the window stays within the bounds of the map
        int bottom_right_x = std::min(static_cast<int>(size_x), map_robot_x + half_img_size);
        int bottom_right_y = std::min(static_cast<int>(size_y), map_robot_y + half_img_size);

        // Extract the section of the map around the robot's position
        cv::Rect section_rect(top_left_x, top_left_y,
                            bottom_right_x - top_left_x,
                            bottom_right_y - top_left_y);

        // Extract the map section around the robot
        cv::Mat temp_seciton = m_MapColImage(section_rect).clone();

        cv::resize(temp_seciton, map_section, cv::Size(section_size,section_size), 0, 0, cv::INTER_LINEAR);

        // Rotate the map section based on the robot's yaw
        cv::Point2f center(map_section.cols / 2.0F, map_section.rows / 2.0F);
        cv::Mat rotation_matrix = cv::getRotationMatrix2D(center, robot_yaw, 1.0);
        cv::warpAffine(map_section, map_section, rotation_matrix, map_section.size());

        cv::rotate(map_section, map_section, cv::ROTATE_90_COUNTERCLOCKWISE);

        cv::imshow(WINDOW_ImageA, map_section);
        cv::waitKey(1);

        // Apply Canny edge detection
        cv::Canny(map_section, map_edges, 20, 80);  // You can adjust these thresholds for better results

        // Optionally, display the edge-detected image
        cv::imshow(WINDOW_ImageB, map_edges);
        cv::waitKey(1);
    
    }
    
    void localiseTurtlebot(cv::Mat img1, cv::Mat img2) {
        // Detect and match features between the first and second images
        std::vector<cv::Point2f> srcPoints, dstPoints;
        detectAndMatchFeatures(img1, img2, srcPoints, dstPoints);

        if (srcPoints.size() < 3 || dstPoints.size() < 3) {
            // RCLCPP_ERROR(this->get_logger(), "Not enough points for affine transformation.");
            return;
        }

        try {
            cv::Mat transform_matrix = cv::estimateAffinePartial2D(srcPoints, dstPoints);
            if (transform_matrix.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Transformation matrix estimation failed.");
            } else {
                // Extract the rotation angle from the transformation matrix
                angle_difference_ = atan2(transform_matrix.at<double>(1, 0), transform_matrix.at<double>(0, 0));
                angle_difference_ = angle_difference_ * 180.0 / CV_PI;
                RCLCPP_INFO(this->get_logger(), "Estimated yaw angle change: %f degrees", angle_difference_);
            }
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in estimateAffinePartial2D: %s", e.what());
        }

        try {
            cv::Mat transform_matrix = cv::estimateAffinePartial2D(srcPoints, dstPoints);
            if (transform_matrix.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Transformation matrix estimation failed.");
            } else {
                double dx = transform_matrix.at<double>(0, 2);  // Translation in x direction
                double dy = transform_matrix.at<double>(1, 2);  // Translation in y direction
                // RCLCPP_INFO(this->get_logger(), "Estimated translation: x = %f, y = %f", dx, dy);

                cv::Mat map_localised = map_section.clone();

                // Scale translation according to map resolution
                double map_dx = dx * map_scale_;
                double map_dy = dy * map_scale_;

                // Calculate robot's current position in the map section (relative to the section)
                int section_robot_x = map_localised.cols / 2;  // Assuming the robot is at the center of the section
                int section_robot_y = map_localised.rows / 2;

                // Calculate the new robot position in the map section
                int new_section_x = section_robot_x + static_cast<int>(map_dx);
                int new_section_y = section_robot_y + static_cast<int>(map_dy);

                // Ensure the new position is within the bounds of the map section
                if (new_section_x >= 0 && new_section_x < map_localised.cols && new_section_y >= 0 && new_section_y < map_localised.rows) {
                    // Draw a small circle representing the TurtleBot's position
                    cv::Point new_pos(new_section_x, new_section_y);
                    cv::circle(map_localised, new_pos, 10, cv::Scalar(0, 0, 255), -1);  // Red filled circle with a radius of 5

                    // Display the updated map section with the robot's estimated position
                    cv::imshow(WINDOW_Localised, map_localised);
                    cv::waitKey(1);
                } else {
                    RCLCPP_WARN(this->get_logger(), "New robot position out of bounds in the map section.");
                }
            }
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in estimateAffinePartial2D: %s", e.what());
        }
    }

    void detectAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2, std::vector<cv::Point2f>& srcPoints, std::vector<cv::Point2f>& dstPoints) {
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;

        orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
        orb->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

        // Check if descriptors are empty
        if (descriptors1.empty() || descriptors2.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty descriptors, skipping matching.");
            return;  // Skip matching if either descriptor is empty
        }

        cv::BFMatcher matcher(cv::NORM_HAMMING, true);
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors1, descriptors2, matches);

        // Sort matches based on distance (lower distance means better match)
        std::sort(matches.begin(), matches.end(), [](const cv::DMatch& a, const cv::DMatch& b) {
            return a.distance < b.distance;
        });

        // Determine the number of top matches to keep (5% of total matches)
        size_t numGoodMatches = static_cast<size_t>(matches.size() * 0.05);

        // Keep only the best matches (top 30%)
        std::vector<cv::DMatch> goodMatches(matches.begin(), matches.begin() + numGoodMatches);

        for (const auto& match : matches) {
            srcPoints.push_back(keypoints1[match.queryIdx].pt);
            dstPoints.push_back(keypoints2[match.trainIdx].pt);
        }
        
        // Draw matches for visualization
        cv::Mat matchImg;
        cv::drawMatches(img1, keypoints1, img2, keypoints2, goodMatches, matchImg);
        cv::imshow(WINDOW_Matches, matchImg);
        cv::waitKey(1);
    }

        void occupancyGridToImage(const nav_msgs::msg::OccupancyGrid::SharedPtr grid) {
            int grid_data;
            unsigned int row, col, val;

            m_temp_img = cv::Mat::zeros(grid->info.height, grid->info.width, CV_8UC1);

            std::cout << "DataParse started for map: " << grid->header.stamp.sec << " Dim: " << grid->info.height << "x" << grid->info.width << std::endl;

            for (row = 0; row < grid->info.height; row++) {
                for (col = 0; col < grid->info.width; col++) {
                    grid_data = grid->data[row * grid->info.width + col];
                    if (grid_data != -1) {
                        val = 255 - (255 * grid_data) / 100;
                        val = (val == 0) ? 255 : 0;
                        m_temp_img.at<uchar>(grid->info.height - row - 1, col) = val;
                    } else {
                        m_temp_img.at<uchar>(grid->info.height - row - 1, col) = 0;
                    }
                }
            }

            map_scale_ = grid->info.resolution;
            origin_x = grid->info.origin.position.x;
            origin_y = grid->info.origin.position.y;
            size_x = grid->info.width;
            size_y = grid->info.height;

            cv::Mat kernel = (cv::Mat_<uchar>(3, 3) << 0, 0, 0,
                                        0, 1, 0,
                                        0, 0, 0);

            cv::erode(m_temp_img, m_MapBinImage, kernel);

            m_MapColImage.create(m_MapBinImage.size(), CV_8UC3);
            cv::cvtColor(m_MapBinImage, m_MapColImage, cv::COLOR_GRAY2BGR);

            std::cout << "Occupancy grid map converted to a binary image\n";

            // // Display the image to verify
            // cv::imshow("Occupancy Grid", m_MapColImage);
            // cv::waitKey(1);
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

    // ROS subscriptions and publishers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;

    // Variables to store received map and odometry
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    sensor_msgs::msg::LaserScan::SharedPtr scan;
    nav_msgs::msg::Odometry odom_;

    // Variables for image processing and visualization

    cv::Mat map_section;
    cv::Mat map_edges;

    cv::Mat m_temp_img;
    cv::Mat m_MapBinImage;
    cv::Mat m_MapColImage;

    double map_scale_;
    double origin_x;
    double origin_y;
    unsigned int size_x;
    unsigned int size_y;

    double angle_difference_ = 0.0;
    double relative_orientaion_ = 0.0;

    // OpenCV window names
    const std::string WINDOW_ImageA = "Image A";
    const std::string WINDOW_ImageB = "Image B";
    const std::string WINDOW_ImageC = "Image C";

    const std::string WINDOW_Matches = "Matches";
    const std::string WINDOW_Localised = "Localised Map";
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanMatchingNode>());
    rclcpp::shutdown();
    return 0;
}