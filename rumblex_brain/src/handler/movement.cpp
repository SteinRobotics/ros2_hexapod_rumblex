#include "handler/movement.hpp"

namespace brain {

using namespace mp_units;
using namespace mp_units::si::unit_symbols;
using mp_units::angular::unit_symbols::rad;
CMovement::CMovement(std::shared_ptr<rclcpp::Node> node) : node_(std::move(node)) {
    pose_model_ = std::make_shared<CPoseModel>(node_);
    gait_controller_ = std::make_unique<CGaitController>(node_, pose_model_);
    pub_body_pose_ =
        node_->create_publisher<rumblex_interfaces::msg::BodyPose>("cmd_movement", rclcpp::QoS(1));
    pub_movement_name_ =
        node_->create_publisher<std_msgs::msg::String>("movement_name", rclcpp::QoS(1).transient_local());
    pub_movement_velocity_ = node_->create_publisher<geometry_msgs::msg::Twist>("movement_velocity", 10);
    sub_body_pose_ = node_->create_subscription<rumblex_interfaces::msg::BodyPose>(
        "body_pose_actual", rclcpp::QoS(1).transient_local(),
        [this](const rumblex_interfaces::msg::BodyPose& pose) { onInitialPose(pose); });
    gait_controller_->on_gait_changed = [this](const MovementRequest& request) {
        const auto target = resolveBodyPose(bodyPose(*pose_model_), request.body_pose);
        torso_ = CPose(target.torso_pose);
        head_ = COrientation(target.head_orientation);
        std_msgs::msg::String name;
        name.data = request.name;
        pub_movement_name_->publish(name);
        if (on_gait_changed) on_gait_changed(request);
    };
    std_msgs::msg::String name;
    name.data = "SEQUENCE_LAYDOWN";
    pub_movement_name_->publish(name);
    setRequestFinished(true);
}

void CMovement::onInitialPose(const rumblex_interfaces::msg::BodyPose& pose) {
    if (initialized_ || !completeBodyPose(pose) || !validBodyPose(pose)) return;
    pose_model_->moveTorso(toeTargets(pose), CPose(pose.torso_pose));
    pose_model_->setHeadOrientation(COrientation(pose.head_orientation));
    last_pose_ = bodyPose(*pose_model_);
    torso_ = CPose(pose.torso_pose);
    head_ = COrientation(pose.head_orientation);
    initialized_ = true;
    if (pending_request_) {
        const auto request = *pending_request_;
        pending_request_.reset();
        startRequest(request);
    } else {
        // Start from measured feedback so startup follows the normal pose trajectory.
        MovementRequest request;
        request.type = MovementRequest::SEQUENCE_LAYDOWN;
        request.name = "SEQUENCE_LAYDOWN";
        setRequestFinished(false);
        startRequest(request);
    }
}

void CMovement::startRequest(const MovementRequest& request) {
    dwell_deadline_.reset();
    tracked_request_.reset();
    if (!gait_controller_->setGait(request)) {
        setRequestFinished(true);
        return;
    }
    // Continuous inputs can change while the same gait remains active.
    if (!gait_controller_->hasPendingGait() && request.type != MovementRequest::SINGLE_POSE) {
        if (request.body_pose.has_torso_pose) torso_ = CPose(request.body_pose.torso_pose);
        if (request.body_pose.has_head_orientation) head_ = COrientation(request.body_pose.head_orientation);
    }
    tracked_request_ = request;
    updateRequestCompletion();
}

void CMovement::run(std::shared_ptr<RequestMovementType> request) {
    const auto& movement = request->movementRequest;
    if (!validBodyPose(movement.body_pose) || !isfinite(movement.duration_s) ||
        (movement.type == MovementRequest::SINGLE_POSE && !hasBodyPose(movement.body_pose)) ||
        (movement.type != MovementRequest::SINGLE_POSE && movement.body_pose.has_toe_positions)) {
        RCLCPP_WARN(node_->get_logger(), "Ignoring invalid movement pose or duration");
        return;
    }
    setRequestFinished(false);
    if (initialized_)
        startRequest(request->movementRequest);
    else
        pending_request_ = request->movementRequest;
}
void CMovement::run(std::shared_ptr<RequestVelocity> request) {
    Velocity limited;
    if (!limitVelocity(request->velocity, velocityLimit<m / s>(node_, "max_velocity_linear"),
                       velocityLimit<rad / s>(node_, "max_velocity_rotation"), limited)) {
        RCLCPP_WARN(node_->get_logger(), "Ignoring non-finite velocity request");
        return;
    }
    velocity_ = limited;
}

void CMovement::cancel() {
    pending_request_.reset();
    dwell_deadline_.reset();
    tracked_request_.reset();
    gait_controller_->requestStopSelectedGait();
    // Release the scheduler now; the controller still owns graceful stopping
    // and the rest-endpoint handoff to a subsequent movement request.
    setRequestFinished(true);
}

void CMovement::updateRequestCompletion() {
    if (!tracked_request_ || gait_controller_->hasPendingGait() ||
        gait_controller_->currentGait() != tracked_request_->type) {
        return;
    }
    if (gait_controller_->finishesAutomatically()) {
        if (!gait_controller_->motionStopped()) return;
    } else {
        // Repeating gaits retain their requested dwell time, measured from activation.
        if (!dwell_deadline_) {
            dwell_deadline_ =
                node_->now() + rclcpp::Duration::from_seconds(
                                   std::max(0.0 * s, tracked_request_->duration_s).numerical_value_in(s));
        }
        if (node_->now() < *dwell_deadline_) return;
    }
    tracked_request_.reset();
    dwell_deadline_.reset();
    setRequestFinished(true);
}

void CMovement::update() {
    updateRequestCompletion();
    const auto now = std::chrono::steady_clock::now();
    if (now < next_update_) return;
    const auto gait = gait_controller_->currentGait();
    const bool frequent =
        gait == MovementRequest::CONTINUOUS_MOVE || gait == MovementRequest::CONTINUOUS_RUNNING ||
        gait == MovementRequest::CONTINUOUS_POSE || gait == MovementRequest::SINGLE_POSE ||
        gait == MovementRequest::SEQUENCE_STAND_UP || gait == MovementRequest::SEQUENCE_LAYDOWN ||
        gait == MovementRequest::SEQUENCE_LOOK || gait == MovementRequest::SEQUENCE_WATCH ||
        gait == MovementRequest::SEQUENCE_BODY_ROLL;
    const auto period = std::chrono::milliseconds(frequent ? 20 : 100);
    const units::Duration elapsed_s =
        (last_update_ ? std::chrono::duration<double>(now - *last_update_).count()
                      : std::chrono::duration<double>(period).count()) *
        s;
    last_update_ = now;
    // Keep a fixed deadline: scheduling from now can turn a 50 Hz loop into 25 Hz
    // when the next iteration arrives fractionally before the previous deadline.
    if (next_update_.time_since_epoch().count() == 0) next_update_ = now;
    next_update_ += period;
    if (next_update_ <= now) next_update_ = now + period;
    geometry_msgs::msg::Twist effective_velocity;
    if (initialized_) {
        const bool progressed = gait_controller_->updateSelectedGait(velocity_, torso_, head_, elapsed_s);
        updateRequestCompletion();
        auto pose = bodyPose(*pose_model_);
        // Some sequences restore their final pose while returning false on completion.
        if ((progressed || pose != last_pose_) && validBodyPose(pose)) {
            pub_body_pose_->publish(pose);
            last_pose_ = pose;
        }
        if (gait_controller_->moving()) effective_velocity = velocity_.toMsg();
    }
    pub_movement_velocity_->publish(effective_velocity);
}
}  // namespace brain
