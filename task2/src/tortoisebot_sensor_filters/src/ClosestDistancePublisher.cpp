#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32.hpp"

class ClosestDistancePublisher : public rclcpp::Node
{
public:
    ClosestDistancePublisher() : Node("closest_distance_publisher")
    {
        _publisher = this->create_publisher<std_msgs::msg::Float32>("closest_distance", 10);
        _scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            std::bind(&ClosestDistancePublisher::scan_callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "ClosestDistancePublisher node has been started.");

    }
private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (msg->ranges.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty LaserScan message.");
            return;
        }

        float min_angle = -15.0f * (M_PI / 180.0f); // -15 degrees in radians
        float max_angle = 15.0f * (M_PI / 180.0f);  // 15 degrees in radians

        
        int min_range_index = (int)((min_angle - msg->angle_min) / msg->angle_increment);
        int max_range_index = (int)((max_angle - msg->angle_min) / msg->angle_increment);
        // RCLCPP_INFO(this->get_logger(), "Processing LaserScan data between %d and %d indexes. Center index is %ld", min_range_index, max_range_index, msg->ranges.size() / 2);

        int size = max_range_index - min_range_index + 1;
        float* relevant_ranges = new float[size];
        std::copy(msg->ranges.begin() + min_range_index, msg->ranges.begin() + max_range_index + 1, relevant_ranges);

        std::vector<float> filtered_ranges;
        for(int i = 0; i <= max_range_index - min_range_index; i++) {
            if(!(std::isnan(relevant_ranges[i]) || std::isinf(relevant_ranges[i]) || relevant_ranges[i] < msg->range_min || relevant_ranges[i] > msg->range_max)) {
                filtered_ranges.push_back(relevant_ranges[i]);
            }
        }

        if (filtered_ranges.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "No Objects detected in the specified angle sector.");
            return;
        }

        filtered_ranges = median_filter(filtered_ranges, 5);

        float closest_distance = *std::min_element(filtered_ranges.begin(), filtered_ranges.end());
        std_msgs::msg::Float32 pub_msg;
        pub_msg.data = closest_distance;
        _publisher->publish(pub_msg);

        RCLCPP_DEBUG(this->get_logger(), "Published closest distance: %.2f meters", closest_distance);

        delete[] relevant_ranges;
    }
    std::vector<float> median_filter(const std::vector<float>& data, int window_size=3) {
        std::vector<float> filtered_data;
        int half_window = window_size / 2;
        int data_size = data.size();

        if (data_size < window_size) {
            RCLCPP_DEBUG(this->get_logger(), "Data size less than window size for median filter.");
            return data; // Not enough data to apply filter
        }

        for (int i = 0; i < data_size; ++i) {
            std::vector<float> window;

            for (int j = -half_window; j <= half_window; ++j) {
                int index = i + j;
                if (index >= 0 && index < data_size) {
                    window.push_back(data[index]);
                }
            }

            std::sort(window.begin(), window.end());
            float median;
            int window_size_actual = window.size();
            if (window_size_actual % 2 == 0) {
                median = (window[window_size_actual / 2 - 1] + window[window_size_actual / 2]) / 2.0f;
            } else {
                median = window[window_size_actual / 2];
            }
            filtered_data.push_back(median);
        }
        return filtered_data;
    }
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _publisher;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _scan_subscriber;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ClosestDistancePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}