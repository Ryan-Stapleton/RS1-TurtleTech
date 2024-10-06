#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class LaserScanProcessor : public rclcpp::Node
{
public:
    LaserScanProcessor()
        : Node("laser_scan_processor")
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LaserScanProcessor::scanCallback, this, std::placeholders::_1));

        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_filter", 10);
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        int n = 10;

        auto filtered_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*scan);

        filtered_scan->ranges.clear();
        filtered_scan->ranges.resize(scan->ranges.size());

        int increments = scan->ranges.size() / n;
        for (int i = 0; i < increments; i++){
            filtered_scan->ranges.at(i * n) = scan->ranges.at(i * n);
        }

        scan_pub_->publish(*filtered_scan);
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanProcessor>());
    rclcpp::shutdown();
    return 0;
}