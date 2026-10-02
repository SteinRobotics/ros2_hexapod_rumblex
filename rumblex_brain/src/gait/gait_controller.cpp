/*******************************************************************************
 * Copyright (c) 2024 Christian Stein
 ******************************************************************************/

#include "gait/gait_controller.hpp"

using brain::MovementRequest;

namespace brain {

brain::MovementRequest CGaitController::createMsg(std::string name, MovementRequestType type) {
    brain::MovementRequest msg;
    msg.name = name;
    msg.type = type;
    return msg;
}

CGaitController::CGaitController(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CPoseModel> kinematics)
    : node_(node), kinematics_(kinematics) {
    // read parameters
    params_ = Parameters::declare(node_);

    // Create all gait instances
    gaits_[MovementRequest::SEQUENCE_BODY_ROLL] =
        std::make_shared<CTorsoRollGait>(node_, kinematics_, params_.torso_roll);
    gaits_[MovementRequest::SEQUENCE_CLAP] = std::make_shared<CClapGait>(node_, kinematics_, params_.clap);
    gaits_[MovementRequest::SEQUENCE_HIGH_FIVE] =
        std::make_shared<CHighFiveGait>(node_, kinematics_, params_.high_five);
    gaits_[MovementRequest::SEQUENCE_LAYDOWN] =
        std::make_shared<CLayDownGait>(node_, kinematics_, params_.lay_down);
    gaits_[MovementRequest::SEQUENCE_LEGS_WAVE] =
        std::make_shared<CLegWaveGait>(node_, kinematics_, params_.leg_wave);
    gaits_[MovementRequest::SEQUENCE_LOOK] = std::make_shared<CLookGait>(node_, kinematics_, params_.look);
    gaits_[MovementRequest::CONTINUOUS_MOVE] = std::make_shared<CMoveCombinedGait>(
        node_, kinematics_, params_.wave, params_.ripple, params_.tripod, params_.move_combined);
    gaits_[MovementRequest::CONTINUOUS_RUNNING] =
        std::make_shared<CRunningGait>(node_, kinematics_, params_.running);
    gaits_[MovementRequest::SEQUENCE_STAND_UP] =
        std::make_shared<CStandUpGait>(node_, kinematics_, params_.stand_up);
    gaits_[MovementRequest::SEQUENCE_TESTLEGS] =
        std::make_shared<CTestLegsGait>(node_, kinematics_, params_.test_legs);
    gaits_[MovementRequest::SINGLE_POSE] =
        std::make_shared<CSinglePoseGait>(node_, kinematics_, params_.single_pose);
    gaits_[MovementRequest::CONTINUOUS_POSE] =
        std::make_shared<CContinuousPoseGait>(node_, kinematics_, params_.continuous_pose);
    gaits_[MovementRequest::SEQUENCE_WAITING] =
        std::make_shared<CWaitingGait>(node_, kinematics_, params_.waiting);
    gaits_[MovementRequest::SEQUENCE_WATCH] = std::make_shared<CWatchGait>(node_, kinematics_, params_.watch);

    // Default active gait (robot starts laying down)
    active_gait_ = gaits_[MovementRequest::SEQUENCE_LAYDOWN];
    active_request_ = createMsg("SEQUENCE_LAYDOWN", MovementRequest::SEQUENCE_LAYDOWN);
    pending_request_ = createMsg("NO_REQUEST", MovementRequest::NO_REQUEST);
}

CGaitController::~CGaitController() = default;

void CGaitController::setGait(brain::MovementRequest request) {
    if (request.type != MovementRequest::NO_REQUEST && !gaits_.contains(request.type)) {
        RCLCPP_WARN(node_->get_logger(), "Unsupported gait: %u", request.type);
        return;
    }
    if (request.type == MovementRequest::NO_REQUEST) {
        RCLCPP_DEBUG(node_->get_logger(), "CGaitController::setGait ignoring NO_REQUEST message");
        return;
    }

    pending_request_ = createMsg("NO_REQUEST", MovementRequest::NO_REQUEST);

    // If there's an active gait and the same gait is requested again, handle
    // transient states (Stopped -> start, StopPending/Stopping -> cancelStop) and return.
    if (request.type == active_request_.type) {
        if (active_gait_->state() == EGaitState::Stopped) {
            RCLCPP_INFO(node_->get_logger(), "CGaitController::setGait starting %s", request.name.c_str());
            active_gait_->start(request.duration_s, request.direction);
        }
        if (active_gait_->state() == EGaitState::StopPending ||
            active_gait_->state() == EGaitState::Stopping) {
            active_gait_->cancelStop();
        }
        return;
    }

    // If the active gait is not yet stopped, request stop and set pending type
    if (active_gait_->state() != EGaitState::Stopped) {
        pending_request_ = request;
        active_gait_->requestStop();
        return;
    }

    // Otherwise, switch immediately
    switchGait(request);
}

void CGaitController::switchGait(brain::MovementRequest request) {
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       "CGaitController: switching to " << request.name << " (type=" << +request.type << ")");
    pending_request_ = createMsg("NO_REQUEST", MovementRequest::NO_REQUEST);
    active_request_ = request;

    active_gait_ = gaits_[request.type];
    active_gait_->start(request.duration_s, request.direction);

    // Publish the new movement type
    if (on_gait_changed) on_gait_changed(request);
}

bool CGaitController::updateSelectedGait(const geometry_msgs::msg::Twist& velocity, CPose torso,
                                         COrientation head) {
    if (pending_request_.type != MovementRequest::NO_REQUEST &&
        active_gait_->state() == EGaitState::Stopped) {
        switchGait(pending_request_);
    }

    if (auto continuous = std::dynamic_pointer_cast<IContinuousGait>(active_gait_)) {
        return continuous->update(velocity, torso, head);
    }
    if (auto sequence = std::dynamic_pointer_cast<ISequenceGait>(active_gait_)) {
        return sequence->update();
    }
    return false;
}

void CGaitController::requestStopSelectedGait() {
    pending_request_ = createMsg("NO_REQUEST", MovementRequest::NO_REQUEST);
    active_gait_->requestStop();
}

}  // namespace brain
