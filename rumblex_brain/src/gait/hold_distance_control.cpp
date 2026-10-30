#include "gait/hold_distance_control.hpp"

#include <algorithm>

#include "feedback_validation.hpp"
#include "tf2/LinearMath/Transform.hpp"

namespace brain {
using namespace feedback_detail;

HoldDistanceSettings loadHoldDistanceSettings(const std::shared_ptr<rclcpp::Node>& node) {
    return {positive(node, "hold_distance.distance_gain"),
            positive(node, "hold_distance.distance_deadband_m"),
            std::min(positive(node, "hold_distance.max_speed_m_s"),
                     node->get_parameter("max_velocity_linear").as_double()),
            positive(node, "hold_distance.max_acceleration_m_s2")};
}
double HoldDistanceLaw::update(double range, double target, double dt) {
    validateInterval(dt);
    if (!std::isfinite(range) || !std::isfinite(target))
        throw std::invalid_argument("Invalid distance input");
    const double error = range - target;
    const double desired =
        std::abs(error) <= settings_.deadband
            ? 0.0
            : std::clamp(settings_.gain * error, -settings_.max_speed, settings_.max_speed);
    speed_ += std::clamp(desired - speed_, -settings_.max_acceleration * dt, settings_.max_acceleration * dt);
    return speed_;
}

RangeFeedback::RangeFeedback(const std::shared_ptr<rclcpp::Node>& node)
    : freshness_s(positive(node, "hold_distance.freshness_s")),
      startup_timeout_s(positive(node, "hold_distance.startup_timeout_s")),
      body_frame_(textParameter(node, "hold_distance.body_frame")),
      node_(node),
      buffer_(node->get_clock()),
      listener_(buffer_, tf2_ros::TransformListener::RequiredInterfaces(*node), false) {
    subscription_ = node->create_subscription<sensor_msgs::msg::Range>(
        textParameter(node, "hold_distance.range_topic"), rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Range::ConstSharedPtr message) {
            message_ = std::move(message);
            received_ = std::chrono::steady_clock::now();
        });
}
std::optional<double> RangeFeedback::range() const {
    if (!message_ || !fresh(*node_, message_->header, received_, freshness_s) ||
        !std::isfinite(message_->range) || !std::isfinite(message_->min_range) ||
        !std::isfinite(message_->max_range) || message_->min_range < 0.0 ||
        message_->max_range <= message_->min_range || message_->range < message_->min_range ||
        message_->range > message_->max_range)
        return std::nullopt;
    try {
        const auto beam = buffer_.lookupTransform(body_frame_, message_->header.frame_id,
                                                  rclcpp::Time(message_->header.stamp));
        const auto q = quaternion(beam.transform.rotation);
        // Range beams point along sensor +X; accept alignment within 15 degrees of body +X.
        if (!validQuaternion(q) ||
            tf2::quatRotate(q, tf2::Vector3(1, 0, 0)).x() < std::cos(std::acos(-1.0) / 12.0))
            return std::nullopt;
        return message_->range;
    } catch (const tf2::TransformException&) {
        return std::nullopt;
    }
}
}  // namespace brain
