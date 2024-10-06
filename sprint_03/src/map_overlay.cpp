#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class MapMatchingNode : public rclcpp::Node {
public:
    MapMatchingNode()
    : Node("map_matching_node") {
        cv::namedWindow(WINDOW_Overlaid, cv::WINDOW_AUTOSIZE);
        cv::namedWindow(WINDOW_RTAB, cv::WINDOW_AUTOSIZE);
        cv::namedWindow(WINDOW_PBuilt, cv::WINDOW_AUTOSIZE);

        // Load the maps
        rtabmap_ = cv::imread("/home/student/ros2_ws/src/sprint_03/src/maps/turtle_tech_rtabmap.pgm");
        gazebo_map_ = cv::imread("/home/student/ros2_ws/src/sprint_03/src/prebuilt_maps/map.pgm");

        // Ensure the maps were loaded
        if (rtabmap_.data == NULL)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load rtab map.");
        }
        // Ensure the maps were loaded
        if (gazebo_map_.data == NULL)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load prebuilt map.");
        }

        // Overlay the maps
        overlayMap();
    }

private:
    void overlayMap()
    {
        while (gazebo_map_.data != NULL && rtabmap_.data != NULL){
            // Find the maximum dimensions (width and height)
            int max_width = std::max(rtabmap_.cols, gazebo_map_.cols);
            int max_height = std::max(rtabmap_.rows, gazebo_map_.rows);

            // Create blank canvases with the maximum dimensions
            cv::Mat rtabmap_canvas = cv::Mat::zeros(max_height, max_width, rtabmap_.type());
            cv::Mat gazebo_map_canvas = cv::Mat::zeros(max_height, max_width, gazebo_map_.type());

            // Copy the RTAB-Map to the top-left of its canvas
            int x_offset_rtab = (max_width - rtabmap_.cols) / 2;
            int y_offset_rtab = (max_height - rtabmap_.rows) / 2;
            rtabmap_.copyTo(rtabmap_canvas(cv::Rect(x_offset_rtab, y_offset_rtab, rtabmap_.cols, rtabmap_.rows)));


            // Copy the Gazebo map to the top-left of its canvas (or wherever you'd like it to be aligned)
            int x_offset_gazebo = (max_width - gazebo_map_.cols) / 2;
            int y_offset_gazebo = (max_height - gazebo_map_.rows) / 2;
            gazebo_map_.copyTo(gazebo_map_canvas(cv::Rect(x_offset_gazebo, y_offset_gazebo, gazebo_map_.cols, gazebo_map_.rows)));

            // Now, blend the two canvases with equal weights
            cv::Mat overlaid_map;
            cv::addWeighted(rtabmap_canvas, 0.5, gazebo_map_canvas, 0.5, 0.0, overlaid_map);

            // Show the original maps and the overlaid result
            cv::imshow(WINDOW_RTAB, rtabmap_);
            cv::imshow(WINDOW_PBuilt, gazebo_map_);
            cv::imshow(WINDOW_Overlaid, overlaid_map);
            cv::waitKey(1);
            RCLCPP_INFO(this->get_logger(), "Overlaid Map Image Created");
        }
        
    }

    cv::Mat rtabmap_;
    cv::Mat gazebo_map_;
    const std::string WINDOW_Overlaid = "Overlaid Maps";
    const std::string WINDOW_RTAB = "RTAB Map";
    const std::string WINDOW_PBuilt = "Prebuilt Maps";

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapMatchingNode>());
    rclcpp::shutdown();
    return 0;
}