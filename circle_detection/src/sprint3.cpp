#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <iostream>
#include <tuple>
#include <thread>
#include <utility>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <cmath>
#include <visualization_msgs/msg/marker.hpp>
#include <chrono>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class sprint3 : public rclcpp::Node
{
public:
    sprint3() : Node("cylinder_detector")
    {
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&sprint3::scanCallback, this, std::placeholders::_1));

        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&sprint3::odom_callback, this, std::placeholders::_1));

        cylinder_diameter_ = 0.3;
        odom_found_ = false;
        radius_tolerance = 0.1;
        goalReceived = false;
        complete = false;

        // for testing purposes 
        // can open rviz, select add new, marker, select topic below
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

        publisher_ = this->create_publisher<geometry_msgs::msg::Point>("point_topic", 10);

        vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    }

private:
    void publishPoint(double x, double y) {
        // Create a Point message
        geometry_msgs::msg::Point point_msg;
        point_msg.x = x;  // Example x coordinate
        point_msg.y = y;  // Example y coordinate
        point_msg.z = 0.0;  // Optional: for 2D points, z can be 0

        // RCLCPP_INFO(this->get_logger(), "Publishing point: x=%.2f, y=%.2f", point_msg.x, point_msg.y);

        // Publish the point
        if(!complete){
            publisher_->publish(point_msg);
        }
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
    {
        laserScan_ = *msg;

        if(odom_found_){
            std::vector<std::pair<double, double>> circle_centers = getCircle(odom_);
    
            publishCircleCenters(circle_centers);
            if (!circle_centers.empty() && !goalReceived) {
                publishPoint(circle_centers[0].first, circle_centers[0].second);

                // std::cout << "x: " << circle_centers[0].first << ", y: " << circle_centers[0].second << std::endl;

                goal = circle_centers[0];
                goalReceived = true;
            }
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extract position data
        odom_ = *msg;

        odom_found_ = true;
    }

    std::vector<std::pair<double, double>> getCircle(nav_msgs::msg::Odometry odo_){
        
        std::vector<std::pair<double, double>> circle_centers;
        std::vector<std::pair<double, double>> circle_centers_world;
        std::vector<std::vector<std::pair<double, double>>> segments = getSegments();         // updates segments

        for(auto cluster : segments){
            if(cluster.size() > 5){
                detectArc(cluster, circle_centers);
            }
        }

        // List of (x, y) points to discard within a certain distance
        std::vector<std::pair<double, double>> discard_points = {
            {-1.2, -0.7},
            {-1.2, 1.0},
            {-1.2, 2.5},
            {-1.2, 4.3},
            {1.2, 2.5},
            {1.2, 2.5},
            {1.2, 1.0},
            {1.2, -0.7},
            {-1.2, -2.3},
            {-1.2, -4.2},
            {3.2, -4.5},
            {-2, -1.6},
            {-2.7, 3},
            {-2.7, 4.5},
            {-0.7, -2.4},
            {0.13, -1.9},
            {0.7, -0.3},
            {0.5, 0.7},
            {1.8, 4}
        };
        // Distance threshold
        const double threshold = 0.5;

    // transform into world coords
        for(auto circle : circle_centers){
            Eigen::Vector3d local_point(circle.first, circle.second, 0);      
            // Pose of the car
            Eigen::Vector3d translation(odo_.pose.pose.position.x, odo_.pose.pose.position.y, 0);
            Eigen::Quaterniond rotation(odo_.pose.pose.orientation.w, odo_.pose.pose.orientation.x, 
                                        odo_.pose.pose.orientation.y, odo_.pose.pose.orientation.z);
            rotation.normalize();
            //Rotation Matrix
            Eigen::Matrix3d rotation_matrix = rotation.toRotationMatrix();
            Eigen::Vector3d world_point = rotation_matrix * local_point + translation;
            double x = world_point.x();
            double y = world_point.y();

            bool discard = false;

            for (const auto& point : discard_points) {
                double dx = x - point.first;
                double dy = y - point.second;
                double distance = std::sqrt(dx * dx + dy * dy);
                
                if (distance <= threshold) {
                    discard = true;
                    break;
                }
            }

            if(!discard){
                circle_centers_world.push_back({x, y});
            }
        }
        
        return circle_centers_world;
    }

    // convert polar coordinates (distance and angle) to cartesian coords in the frame of the robot
    std::pair<double, double> polarToCart(unsigned int index)
    {
        float angle = laserScan_.angle_min + laserScan_.angle_increment*index; // + 1.5708;
        float range = laserScan_.ranges.at(index);
        geometry_msgs::msg::Point cart;
        double x = static_cast<double>(range*cos(angle));
        double y = static_cast<double>(range*sin(angle));
        std::pair<double, double> pair(x,y);
        return pair;
    }

    int calculateDistance(std::pair<double, double> p1, std::pair<double, double>p2){
        int dist = std::sqrt(std::pow(p1.first - p2.first, 2) + std::pow(p1.second - p2.second, 2));
        return dist;
    }

    std::vector<std::vector<std::pair<double, double>>> getSegments()
    {
        std::vector<std::vector<std::pair<double, double>>> segments; // vector of clusters
        std::vector<std::pair<double, double>> cluster;               // vector of indexes for laserScan range values within a single cluster/group
        std::vector<float> ranges = laserScan_.ranges;

        sensor_msgs::msg::LaserScan laserScan = laserScan_;   

        // obtains only valid readings
        std::vector<std::pair<double, double>> points;
        for (unsigned int i = 0; i < laserScan.ranges.size(); ++i){

            if ((laserScan.ranges.at(i) > laserScan.range_min) &&
                (laserScan.ranges.at(i) < laserScan.range_max) &&
                !isnan(laserScan.ranges.at(i)) && isfinite(laserScan.ranges.at(i))){

                std::pair<double, double> pair = polarToCart(i);   
                points.push_back(pair);
            }
        }

        for (unsigned int i = 1; i < points.size(); ++i){

            if((calculateDistance(points[i], points[i-1])) < 0.3){
                cluster.push_back(points[i]);
            }
            else{
                segments.push_back(cluster);
                cluster.clear();
            }
        }
        return segments;
    }

    /**
     * @brief determines wheter or not a cluster of points from a laser scan is in an arc and if so where the centre of the arc is located.
     * @param cluster the input cluster of points.
     * @param circle_centers the output estemation for the circles centre.
     */
    void detectArc(const std::vector<std::pair<double, double>> &cluster, std::vector<std::pair<double, double>> &circle_centers)
    {
        if (cluster.size() < 5)
        {
            return; // We need at least 5 points to verify the arc condition
        }

        const double target_radius = cylinder_diameter_ / 2.0;
        std::vector<std::pair<double, double>> potential_centers;
        std::vector<double> radii;

        // Iterate through combinations of points in the cluster
        for (size_t i = 0; i < cluster.size() - 2; ++i)
        {
            for (size_t j = i + 1; j < cluster.size() - 1; ++j)
            {
                for (size_t k = j + 1; k < cluster.size(); ++k)
                {
                    std::pair<double, double> center;
                    double radius;

                    if (calculateCircleFromThreePoints(cluster.at(i), cluster.at(j), cluster.at(k), radius, center))
                    {
                        potential_centers.push_back(center);
                        radii.push_back(radius);
                    }
                }
            }
        }

        // Calculate the mean center and radius from all potential circles
        if (!potential_centers.empty())
        {
            double avg_x = 0.0, avg_y = 0.0, avg_radius = 0.0;
            for (size_t i = 0; i < potential_centers.size(); ++i)
            {
                avg_x += potential_centers[i].first;
                avg_y += potential_centers[i].second;
                avg_radius += radii[i];
            }
            avg_x /= potential_centers.size();
            avg_y /= potential_centers.size();
            avg_radius /= potential_centers.size();

            // Check if the average radius is within the tolerance range
            if (std::abs(avg_radius - target_radius) <= radius_tolerance)
            {
                circle_centers.push_back({avg_x, avg_y});
            }
        }
    }

    /**
     * @brief Calculates the center and radius of a circle from three given points.
     * This function determines the circle that passes through three non-collinear points
     * by calculating the circle's center and radius. If the points are collinear,
     * the function returns `false`, as no circle can be formed. https://math.stackexchange.com/questions/213658/get-the-equation-of-a-circle-when-given-3-points
     *
     * @param p1 A pair representing the coordinates (x, y) of the first point.
     * @param p2 A pair representing the coordinates (x, y) of the second point.
     * @param p3 A pair representing the coordinates (x, y) of the third point.
     * @param[out] radius A reference to a double that will store the calculated radius of the circle.
     * @param[out] center A reference to a pair that will store the calculated center (x, y) of the circle.
     *
     * @return `true` if the circle was successfully calculated, `false` if the points are collinear.
     */
    bool calculateCircleFromThreePoints(const std::pair<double, double> &p1,
                                        const std::pair<double, double> &p2,
                                        const std::pair<double, double> &p3,
                                        double &radius,
                                        std::pair<double, double> &center)
    {
        double tol = 0.05;
        double x1 = p1.first, y1 = p1.second;
        double x2 = p2.first, y2 = p2.second;
        double x3 = p3.first, y3 = p3.second;

        double ma = (y2 - y1) / (x2 - x1);
        double mb = (y3 - y2) / (x3 - x2);

        // Check for collinearity (parallel slopes)
        if (std::abs(ma - mb) < tol)
        {
            return false; // The points are collinear, can't form a circle
        }

        // Calculate center of the circle
        double cx = (ma * mb * (y1 - y3) + mb * (x1 + x2) - ma * (x2 + x3)) / (2 * (mb - ma));
        double cy = -1 / ma * (cx - (x1 + x2) / 2) + (y1 + y2) / 2;

        center = {cx, cy};
        radius = std::sqrt(std::pow(cx - x1, 2) + std::pow(cy - y1, 2));

        return true;
    }

    void publishCircleCenters(const std::vector<std::pair<double, double>>& circle_centers) {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";  // Ensure the frame matches your system
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "circle_centers";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the scale (size) of the points
        marker.scale.x = 0.2;  // Set this to the desired size of the points in RViz
        marker.scale.y = 0.2;

        // Set the color of the points (RGBA)
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;  // Alpha (transparency)

        // Fill the points with the circle centers
        for (const auto& center : circle_centers) {
            geometry_msgs::msg::Point p;
            p.x = center.first;  // x-coordinate
            p.y = center.second; // y-coordinate
            p.z = 0.0;  // Assuming a 2D plane, set z to 0
            marker.points.push_back(p);
        }

        // Publish the marker
        marker_pub_->publish(marker);
    }

public:
    void run(){

        std::pair<double, double> pointC;
        bool turning = true;
        bool driving = true;
        bool circleMotion = true;
        bool circleDriving = true;
        bool circleturning = true;
        double v = 0.2;
        double minTurnError = 0.01;
        std::chrono::steady_clock::time_point start_time;
    
        publishVel(0.0, 0.0);

        while(true){
            
            while(goalReceived){

                double goalYaw = calculateYaw(odom_);
                pointC = calculatePointC(goal);

                RCLCPP_ERROR(this->get_logger(), "[CIRCLE DETECTION] TURNING TOWARDS THE CIRCLE");

                while(turning){
                    calcError(pointC, goalYaw, odom_);
                    publishVel(0.0, errorYaw_);

                    if(errorYaw_ < minTurnError){
                        publishVel(0.0, 0.0);
                        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(10)));
                        turning = false;
                    }
                }

                RCLCPP_ERROR(this->get_logger(), "[CIRCLE DETECTION] DRIVING TOWARDS THE CIRCLE");

                while(driving){
                    calcError(pointC, goalYaw, odom_);
                    publishVel(v, errorYaw_);
                    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(10)));
                    if(errorDist_ < 0.1){
                        publishVel(0.0, 0.0);
                        driving = false;
                    }
                }

                // circling (hard coded)
                double circleYaw = goalYaw + 1.75; // 100 degress ISH
                circleYaw = normalizeAngle(circleYaw);

                RCLCPP_ERROR(this->get_logger(), "[CIRCLE DETECTION] CIRCLING THE CIRCLE");

                while(circleMotion){
                    while(circleturning){
                        calcError(pointC, circleYaw, odom_);
                        publishVel(0.0, errorYaw_); // spins to quick, error to large
                        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(10)));

                        // std::cout << "Yaw Error = " << errorYaw_ << std::endl;

                        if(errorYaw_ < 0.01 && errorYaw_ > -0.01){
                            publishVel(0.0, 0.0);                            
                            circleturning = false;
                            start_time = std::chrono::steady_clock::now();
                        }
                    }
                    
                    while(circleDriving){
                        goalYaw = calculateYaw(odom_) + 1.4; // add 90 degress to make perpendicular
                        goalYaw = normalizeAngle(goalYaw);
                        calcError(goal, goalYaw, odom_);
                        publishVel(v, errorYaw_*2);

                        // std::cout << "goalYaw = " << goalYaw << ",  errorYaw = " << errorYaw_ << std::endl;

                        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(10)));

                        auto current_time = std::chrono::steady_clock::now();
                        std::chrono::duration<double> elapsed_seconds = current_time - start_time;
                        
                        if(elapsed_seconds.count() >= 40){
                            circleDriving = false; // Toggle circleDriving
                            RCLCPP_INFO(this->get_logger(), "[CIRCLE DETECTION] ERROR: CIRCLE PATH INCOMPLETE, TIMED OUT");
                        }
                        if(elapsed_seconds.count() > 10 && (goalYaw > circleYaw - 0.05 && goalYaw < circleYaw + 0.05)){
                            circleDriving = false; // Toggle circleDriving
                            RCLCPP_INFO(this->get_logger(), "[CIRCLE DETECTION] CIRCLE PATH COMPLETED");
                        }
                    }
                    
                    publishVel(0, 0);
                    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(10)));
                    circleMotion = false;
                }

                //publish goal to indicate finished
                publishPoint(99.0, 99.0);
                goalReceived = false;
                complete = true;
            }

            if(complete){
                // rclcpp::shutdown();
                return;
            }
        }
    }

    std::pair<double, double> calculatePointC(std::pair<double, double> circle_center)
    {
        std::pair<double, double> point_c;
        double desired_dist = drivingCircleRad;
        double theta = atan2(odom_.pose.pose.position.y - circle_center.second, odom_.pose.pose.position.x - circle_center.first);
        point_c.first = circle_center.first + desired_dist * cos(theta);
        point_c.second = circle_center.second + desired_dist * sin(theta);
        return point_c;
    }

private:
    double calculateYaw(nav_msgs::msg::Odometry odom){
        double x1 = odom.pose.pose.position.x;
        double y1 = odom.pose.pose.position.y;
        double x2 = goal.first;
        double y2 = goal.second;
        double rad_180deg_ = 3.14159;

        // calculate where to look 
        double goalYaw = atan((y2-y1)/(x2-x1));

        if((x2 < x1) && (y2 > y1)){ // bottom left
            goalYaw = goalYaw + rad_180deg_;
        }
        else if((x2 < x1) && (y2 < y1)){ // bottom right
            goalYaw = goalYaw - rad_180deg_;
        }
        return goalYaw;
    }

    void publishVel(double linear, double angular){
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = linear;
        twist_msg.angular.z = angular;
        vel_publisher_->publish(twist_msg);
    }

    void calcError(std::pair<double, double> goal, double goalYaw, nav_msgs::msg::Odometry odom_){
        // Extract position data
        double currentX = odom_.pose.pose.position.x;
        double currentY = odom_.pose.pose.position.y;

        // Extract and convert orientation (quaternion) to yaw
        tf2::Quaternion q(
            odom_.pose.pose.orientation.x,
            odom_.pose.pose.orientation.y,
            odom_.pose.pose.orientation.z,
            odom_.pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // Set the current yaw angle
        double currentYaw = yaw;
        // error in yaw
        errorYaw_ = goalYaw - currentYaw;
        errorYaw_ = normalizeAngle(errorYaw_);

        // error in distance
        errorDist_ = std::sqrt(std::pow(goal.first - currentX, 2) + std::pow(goal.second - currentY, 2));
    }

    double normalizeAngle(double angle)
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

    nav_msgs::msg::Odometry odom_;
    sensor_msgs::msg::LaserScan laserScan_;

    bool odom_found_;
    double cylinder_diameter_;
    double radius_tolerance; // Allowable deviation from the target radius

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;

    std::pair<double, double> goal;
    bool goalReceived;
    double errorYaw_;
    double errorDist_;
    const double drivingCircleRad = 0.5;
    bool complete;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<sprint3>());
    // rclcpp::shutdown();

    auto node = std::make_shared<sprint3>();  // Create a shared pointer to sprint3 node
    std::thread spin_thread([node]() { rclcpp::spin(node); }); // Spin in a separate thread
    node->run();  // Run the main logic
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}