#include "gait/level_torso_control.hpp"

#include <algorithm>

#include "feedback_validation.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"

namespace brain {
using namespace feedback_detail;

LevelTorsoSettings loadLevelTorsoSettings(const std::shared_ptr<rclcpp::Node>& node) {
    const double degrees_to_rad = std::acos(-1.0) / 180.0;
    return {positive(node, "level_torso.tilt_gain"), positive(node, "level_torso.tilt_deadband_rad"),
            std::min({positive(node, "level_torso.max_tilt_rad"),
                      node->get_parameter("gait.generic.torso_max_roll_deg").as_double() * degrees_to_rad,
                      node->get_parameter("gait.generic.torso_max_pitch_deg").as_double() * degrees_to_rad}),
            positive(node, "level_torso.max_tilt_rate_rad_s")};
}
void LevelTorsoLaw::reset(TorsoTilt initial) {
    if (!std::isfinite(initial.roll) || !std::isfinite(initial.pitch) ||
        std::abs(initial.roll) > settings_.max_tilt || std::abs(initial.pitch) > settings_.max_tilt)
        throw std::invalid_argument("Initial torso pose exceeds leveling limits");
    tilt_ = initial;
}
TorsoTilt LevelTorsoLaw::update(TorsoTilt measured, double dt) {
    validateInterval(dt);
    if (!std::isfinite(measured.roll) || !std::isfinite(measured.pitch))
        throw std::invalid_argument("Invalid tilt input");
    const auto correct = [&](double command, double tilt) {
        const double rate = std::abs(tilt) <= settings_.deadband
                                ? 0.0
                                : std::clamp(-settings_.gain * tilt, -settings_.max_rate, settings_.max_rate);
        return std::clamp(command + rate * dt, -settings_.max_tilt, settings_.max_tilt);
    };
    tilt_ = {correct(tilt_.roll, measured.roll), correct(tilt_.pitch, measured.pitch)};
    return tilt_;
}

ImuFeedback::ImuFeedback(const std::shared_ptr<rclcpp::Node>& node)
    : freshness_s(positive(node, "level_torso.freshness_s")),
      startup_timeout_s(positive(node, "level_torso.startup_timeout_s")),
      body_frame_(textParameter(node, "level_torso.body_frame")),
      node_(node),
      buffer_(node->get_clock()),
      listener_(buffer_, tf2_ros::TransformListener::RequiredInterfaces(*node), false) {
    subscription_ = node->create_subscription<sensor_msgs::msg::Imu>(
        textParameter(node, "level_torso.imu_topic"), rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr message) {
            message_ = std::move(message);
            received_ = std::chrono::steady_clock::now();
        });
}
std::optional<TorsoTilt> ImuFeedback::tilt() const {
    if (!message_ || !fresh(*node_, message_->header, received_, freshness_s) ||
        message_->orientation_covariance[0] == -1.0)
        return std::nullopt;
    auto orientation = quaternion(message_->orientation);
    if (!validQuaternion(orientation)) return std::nullopt;
    try {
        const auto mounting = buffer_.lookupTransform(body_frame_, message_->header.frame_id,
                                                      rclcpp::Time(message_->header.stamp));
        auto mount_q = quaternion(mounting.transform.rotation);
        if (!validQuaternion(mount_q)) return std::nullopt;
        orientation.normalize();
        mount_q.normalize();
        TorsoTilt result;
        double yaw;
        tf2::Matrix3x3(orientation * mount_q.inverse()).getRPY(result.roll, result.pitch, yaw);
        return result;
    } catch (const tf2::TransformException&) {
        return std::nullopt;
    }
}
}  // namespace brain
