#include "handler/movement.hpp"

namespace brain {
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
        std_msgs::msg::String name;
        name.data = request.name;
        pub_movement_name_->publish(name);
        if (on_gait_changed) on_gait_changed(request);
    };
    std_msgs::msg::String name;
    name.data = "SEQUENCE_LAYDOWN";
    pub_movement_name_->publish(name);
    setDone(true);
}

void CMovement::onInitialPose(const rumblex_interfaces::msg::BodyPose& pose) {
    if (initialized_ || !validBodyPose(pose)) return;
    pose_model_->moveTorso(toeTargets(pose), CPose(pose.torso_pose));
    pose_model_->setHeadOrientation(COrientation(pose.head_pose));
    last_pose_ = bodyPose(*pose_model_);
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
        setDone(false);
        startRequest(request);
    }
}

void CMovement::startRequest(const MovementRequest& request) {
    completion_time_.reset();
    completion_request_.reset();
    if (!gait_controller_->setGait(request)) {
        setDone(true);
        return;
    }
    completion_request_ = request;
    updateCompletion();
}

void CMovement::run(std::shared_ptr<RequestMovementType> request) {
    setDone(false);
    if (initialized_)
        startRequest(request->movementRequest);
    else
        pending_request_ = request->movementRequest;
}
void CMovement::run(std::shared_ptr<RequestSinglePose> request) {
    torso_ = CPose(request->pose);
}
void CMovement::run(std::shared_ptr<RequestHeadOrientation> request) {
    head_ = COrientation(request->orientation);
}
void CMovement::run(std::shared_ptr<RequestVelocity> request) {
    geometry_msgs::msg::Twist limited;
    if (!limitVelocity(request->velocity, velocityLimit(node_, "max_velocity_linear"),
                       velocityLimit(node_, "max_velocity_rotation"), limited)) {
        RCLCPP_WARN(node_->get_logger(), "Ignoring non-finite velocity request");
        return;
    }
    velocity_ = limited;
}

void CMovement::cancel() {
    pending_request_.reset();
    completion_time_.reset();
    completion_request_.reset();
    gait_controller_->requestStopSelectedGait();
    setDone(true);
}

void CMovement::updateCompletion() {
    if (!completion_request_ || gait_controller_->hasPendingGait() ||
        gait_controller_->currentGait() != completion_request_->type) {
        return;
    }
    if (gait_controller_->finishesAutomatically()) {
        if (!gait_controller_->stopped()) return;
    } else {
        // Repeating gaits retain their requested dwell time, measured from activation.
        if (!completion_time_) {
            completion_time_ =
                node_->now() + rclcpp::Duration::from_seconds(std::max(0.0, completion_request_->duration_s));
        }
        if (node_->now() < *completion_time_) return;
    }
    completion_request_.reset();
    completion_time_.reset();
    setDone(true);
}

void CMovement::update() {
    updateCompletion();
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
    const double elapsed_s = last_update_ ? std::chrono::duration<double>(now - *last_update_).count()
                                          : std::chrono::duration<double>(period).count();
    last_update_ = now;
    // Keep a fixed deadline: scheduling from now can turn a 50 Hz loop into 25 Hz
    // when the next iteration arrives fractionally before the previous deadline.
    if (next_update_.time_since_epoch().count() == 0) next_update_ = now;
    next_update_ += period;
    if (next_update_ <= now) next_update_ = now + period;
    geometry_msgs::msg::Twist effective_velocity;
    if (initialized_) {
        const bool progressed = gait_controller_->updateSelectedGait(velocity_, torso_, head_, elapsed_s);
        updateCompletion();
        auto pose = bodyPose(*pose_model_);
        // Some sequences restore their final pose while returning false on completion.
        if ((progressed || pose != last_pose_) && validBodyPose(pose)) {
            pub_body_pose_->publish(pose);
            last_pose_ = pose;
        }
        if (gait_controller_->moving()) effective_velocity = velocity_;
    }
    pub_movement_velocity_->publish(effective_velocity);
}
}  // namespace brain
