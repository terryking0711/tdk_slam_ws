#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>
#include <limits>

class LaserAngleFilter : public rclcpp::Node {
public:
    LaserAngleFilter() : Node("laser_angle_filter") {
        this->declare_parameter("lower_angle", -0.17);
        this->declare_parameter("upper_angle", 0.17);
        this->declare_parameter<std::string>("input_topic", "/scan");
        this->declare_parameter<std::string>("output_topic", "/scan_filtered");

        this->get_parameter("input_topic", input_topic);
        this->get_parameter("output_topic", output_topic);
        pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(output_topic, rclcpp::QoS(10).reliable().durability_volatile());
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            input_topic, 10, std::bind(&LaserAngleFilter::scan_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "C++ 雷達角度濾波器已啟動");
    }

private:
    std::string input_topic;
    std::string output_topic;
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        auto filtered_scan = *msg; // 複製原始資料
        
        double lower = this->get_parameter("lower_angle").as_double();
        double upper = this->get_parameter("upper_angle").as_double();

        for (size_t i = 0; i < filtered_scan.ranges.size(); ++i) {
            // 計算當前點的角度
            double angle = msg->angle_min + i * msg->angle_increment;

            // 邏輯：如果在指定的角度區間內，將距離設為無窮大 (inf)
            if (angle >= lower && angle <= upper) {
                filtered_scan.ranges[i] = std::numeric_limits<float>::infinity();
            }
        }

        pub_->publish(filtered_scan);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserAngleFilter>());
    rclcpp::shutdown();
    return 0;
}