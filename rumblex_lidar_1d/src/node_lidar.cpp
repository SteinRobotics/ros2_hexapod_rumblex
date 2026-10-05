/*******************************************************************************
 * Copyright (c) 2025 Christian Stein
 ******************************************************************************/

#include <lidarlite_v3.h>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

using namespace std::chrono_literals;

class NodeLidar : public rclcpp::Node {
   public:
    NodeLidar() : Node("node_lidar") {
        frame_id_ = declare_parameter("frame_id", "lidar_link");
        field_of_view_ = declare_parameter("field_of_view", 0.00436332313);
        min_range_ = declare_parameter("min_range", 0.05);
        max_range_ = declare_parameter("max_range", 40.0);
        const auto period = declare_parameter("measurement_period_ms", 100);
        if (period <= 0) throw std::invalid_argument("measurement_period_ms must be positive");
        lidarLite_.i2c_init();

        // Optionally configure LIDAR-Lite
        lidarLite_.configure(0);

        publisher_ = this->create_publisher<sensor_msgs::msg::Range>("scan_1d", 10);
        auto timer_callback = [this]() -> void {
            auto busyFlag = lidarLite_.getBusyFlag();

            if (busyFlag == 0x00) {
                // When no longer busy, immediately initialize another measurement
                // and then read the distance data from the last measurement.
                // This method will result in faster I2C rep rates.
                lidarLite_.takeRange();
                auto distance = lidarLite_.readDistance();
                const auto distance_m = static_cast<float>(distance) / 100.0F;  // convert cm to m

                auto message = sensor_msgs::msg::Range();
                message.header.stamp = this->get_clock()->now();
                message.header.frame_id = frame_id_;
                message.radiation_type = sensor_msgs::msg::Range::INFRARED;
                message.field_of_view = static_cast<float>(field_of_view_);
                message.min_range = static_cast<float>(min_range_);
                message.max_range = static_cast<float>(max_range_);
                message.range = distance_m;
                this->publisher_->publish(message);
            }
        };
        timer_ = this->create_wall_timer(std::chrono::milliseconds(period), timer_callback);
    }

   private:
    std::string frame_id_;
    double field_of_view_, min_range_, max_range_;
    LIDARLite_v3 lidarLite_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NodeLidar>());
    rclcpp::shutdown();
    return 0;
}