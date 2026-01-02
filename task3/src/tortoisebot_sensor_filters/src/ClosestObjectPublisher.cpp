#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_2d_msgs/msg/twist2_d.hpp"

struct indexed_value {
    int index;
    float value;
};

class ClosestObjectPublisher : public rclcpp::Node
{
public:
    ClosestObjectPublisher() : Node("closest_object_publisher")
    {
        _publisher = this->create_publisher<nav_2d_msgs::msg::Twist2D>("closest_object", 10);
        _scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            std::bind(&ClosestObjectPublisher::scan_callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "ClosestObjectPublisher node has been started.");

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

        // RCLCPP_INFO(this->get_logger(), "Min angle: %.2f radians, Max angle: %.2f radians", min_angle, max_angle);

        int min_range_index = (int)((min_angle - msg->angle_min) / msg->angle_increment);
        int max_range_index = (int)((max_angle - msg->angle_min) / msg->angle_increment);

        // RCLCPP_INFO(this->get_logger(), "Processing LaserScan from index %d to %d", min_range_index, max_range_index);   

        std::vector<indexed_value> indexed_ranges;

        for (int i = min_range_index; i <= max_range_index; i++) {
            indexed_value iv;
            iv.index = i;
            iv.value = msg->ranges[i];
            indexed_ranges.push_back(iv);
        }

        // RCLCPP_INFO(this->get_logger(), "Values start with: <%d, %.2f>,<%d, %.2f>, and end with: <%d, %.2f>,<%d, %.2f>", 
        //     indexed_ranges[0].index, indexed_ranges[0].value,
        //     indexed_ranges[1].index, indexed_ranges[1].value,
        //     indexed_ranges[indexed_ranges.size() - 2].index, indexed_ranges[indexed_ranges.size() - 2].value,
        //     indexed_ranges[indexed_ranges.size() - 1].index, indexed_ranges[indexed_ranges.size() - 1].value
        // );

        
        std::vector<indexed_value> filtered_ranges;
        for(int i = 0; i <= max_range_index - min_range_index; i++) {
            if(!(std::isnan(indexed_ranges[i].value) || std::isinf(indexed_ranges[i].value) || indexed_ranges[i].value < msg->range_min || indexed_ranges[i].value > msg->range_max)) {
                filtered_ranges.push_back(indexed_ranges[i]);
            }
        }
        
        nav_2d_msgs::msg::Twist2D pub_msg;
        
        if (filtered_ranges.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "No Objects detected in the specified angle sector.");
            pub_msg.x = -1.0f; // Indicate no object found
            pub_msg.theta = 0.0f;
            _publisher->publish(pub_msg);
            return;
        }

        filtered_ranges = median_filter(filtered_ranges, 5);

        indexed_value closest_object = filtered_ranges[0];
        for (const auto& iv : filtered_ranges) {
            if(iv.value < closest_object.value) {
                closest_object = iv;
            }
        }

        pub_msg.x = closest_object.value;
        float angle = msg->angle_min + (closest_object.index * msg->angle_increment);
        // RCLCPP_INFO(this->get_logger(), "Closest object at index %d with distance %.2f meters and angle %.2f radians", closest_object.index, closest_object.value, angle);
        pub_msg.theta = angle;
        _publisher->publish(pub_msg);

        // RCLCPP_INFO(this->get_logger(), "Published closest distance: %.2f meters and angle: %.2f radians", closest_object.value, pub_msg.theta);
    }
    std::vector<indexed_value> median_filter(const std::vector<indexed_value>& data, int window_size=3) {
        std::vector<indexed_value> filtered_data;
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
                    window.push_back(data[index].value);
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
            filtered_data.push_back({data[i].index, median});
        }
        return filtered_data;
    }
    rclcpp::Publisher<nav_2d_msgs::msg::Twist2D>::SharedPtr _publisher;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _scan_subscriber;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ClosestObjectPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}