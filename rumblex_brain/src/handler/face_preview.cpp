#include "handler/face_preview.hpp"

#include <algorithm>
#include <cmath>

namespace brain {
using namespace mp_units;
using namespace mp_units::si::unit_symbols;
using mp_units::angular::unit_symbols::deg;
using mp_units::angular::unit_symbols::rad;
using namespace rumblex_geometry;

FacePreview::FacePreview(const std::shared_ptr<rclcpp::Node>& node) : node_(node) {
    preview_ = node_->declare_parameter("face_tracking.desktop_preview", false);
    if (!preview_) return;
    const auto settings = [this](const std::string& name) {
        const auto key = "face_tracking." + name;
        const auto value = node_->declare_parameter<double>(key);
        if (!std::isfinite(value) || value <= 0)
            throw std::invalid_argument(key + " must be finite and positive");
        return value;
    };
    max_yaw_ = settings("max_yaw_deg");
    max_pitch_ = settings("max_pitch_deg");
    max_speed_ = settings("max_speed_deg_s");
    acceleration_ = settings("acceleration_deg_s2");
    gain_ = settings("gain");
    deadband_ = settings("deadband_deg");
    if (max_yaw_ > node_->get_parameter("gait.generic.head_max_yaw_deg").as_double() ||
        max_pitch_ > node_->get_parameter("gait.generic.head_max_pitch_deg").as_double())
        throw std::invalid_argument("Face preview limits exceed robot head limits");
    enabled_ = true;
    status_pub_ = node_->create_publisher<std_msgs::msg::String>("brain/face_tracking_status",
                                                                 rclcpp::QoS(1).transient_local());
    subscription_ = node_->create_subscription<rumblex_interfaces::msg::FaceTarget>(
        "perception/face_target", rclcpp::QoS(1),
        [this](rumblex_interfaces::msg::FaceTarget::ConstSharedPtr target) {
            if (!enabled_) return;
            const auto stamp = rclcpp::Time(target->header.stamp).nanoseconds();
            const double age = (node_->now() - rclcpp::Time(target->header.stamp)).seconds();
            if (!target->valid || target->header.frame_id != "desktop_camera_optical_frame" ||
                !std::isfinite(target->x_error) || !std::isfinite(target->y_error) ||
                !std::isfinite(target->confidence) || target->confidence < 0 || target->confidence > 1 ||
                age < -0.05 || age > 0.3 || stamp <= last_stamp_) {
                target_.reset();
                if (stamp < last_stamp_) last_stamp_ = 0;
                return;
            }
            last_stamp_ = stamp;
            target_ = target;
            received_ = std::chrono::steady_clock::now();
        });
    service_ = node_->create_service<std_srvs::srv::SetBool>(
        "brain/set_face_tracking", [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
                                          std_srvs::srv::SetBool::Response::SharedPtr response) {
            enabled_ = request->data;
            target_.reset();
            last_stamp_ = 0;
            yaw_velocity_ = pitch_velocity_ = 0.0 * deg / s;
            response->success = true;
            response->message =
                enabled_ ? "Offline preview enabled; waiting for idle pose and fresh face" : "Disabled";
            status(response->message);
        });
    status("ACQUIRING: offline head preview");
}

void FacePreview::status(const std::string& value) {
    if (!status_pub_ || state_ == value) return;
    state_ = value;
    std_msgs::msg::String message;
    message.data = value;
    status_pub_->publish(message);
}

void FacePreview::suspend() {
    if (!preview_) return;
    enabled_ = false;
    target_.reset();
    yaw_velocity_ = pitch_velocity_ = 0.0 * deg / s;
    status("DISABLED: manual/sequence/safety override; re-enable explicitly when ready");
}

std::optional<COrientation> FacePreview::update(const COrientation& current, units::Duration dt, bool idle) {
    const bool fresh =
        target_ && (node_->now() - rclcpp::Time(target_->header.stamp)).seconds() >= -0.05 &&
        (node_->now() - rclcpp::Time(target_->header.stamp)).seconds() <= 0.3 &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - received_).count() <= 0.3;
    if (!active() || !idle || !fresh) {
        yaw_velocity_ = pitch_velocity_ = 0.0 * deg / s;
        if (active()) status(idle ? "LOST: waiting for fresh face" : "INHIBITED: pose is not idle");
        return std::nullopt;
    }
    if (!isfinite(dt) || dt <= 0.0 * s) return std::nullopt;
    dt = std::min(dt, 0.1 * s);
    // Optical ray (x right, y down, z forward) -> base direction (forward, left, up).
    const units::Angle yaw = -std::atan(target_->x_error) * rad;
    const units::Angle pitch = std::atan2(target_->y_error, std::hypot(1.0, target_->x_error)) * rad;
    const auto step = [this, dt](units::Angle actual, units::Angle desired, double limit,
                                 units::AngularVelocity& velocity) -> units::Angle {
        desired = std::clamp(desired, -limit * deg, limit * deg);
        const auto error = desired - actual;
        if (abs(error) <= deadband_ * deg) {
            velocity = 0.0 * deg / s;
            return actual;
        }
        const units::AngularVelocity requested =
            std::clamp(gain_ * error / s, -max_speed_ * deg / s, max_speed_ * deg / s);
        const units::AngularVelocity change = acceleration_ * deg / (s * s) * dt;
        velocity += std::clamp(requested - velocity, -change, change);
        units::Angle result = actual + velocity * dt;
        if ((desired - actual) * (desired - result) < 0.0 * (deg * deg)) {
            result = desired;
            velocity = 0.0 * deg / s;
        }
        return std::clamp(result, -limit * deg, limit * deg);
    };
    COrientation result(0.0 * deg, step(current.pitch, pitch, max_pitch_, pitch_velocity_),
                        step(current.yaw, yaw, max_yaw_, yaw_velocity_));
    status(target_->calibrated ? "TRACKING: offline calibrated preview"
                               : "TRACKING: offline approximate FOV");
    return result;
}
}  // namespace brain
