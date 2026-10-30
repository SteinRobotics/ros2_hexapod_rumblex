/*******************************************************************************
 * Copyright (c) 2024 Christian Stein
 ******************************************************************************/

#include "gait/gait_controller.hpp"

#include "gait/gait_hold_distance.hpp"
#include "gait/gait_level_torso.hpp"

using brain::MovementRequest;

namespace brain {

using namespace mp_units;
using namespace mp_units::si::unit_symbols;
using mp_units::angular::unit_symbols::deg;

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
    gaits_[MovementRequest::SEQUENCE_LAYDOWN] = std::make_shared<CSinglePoseGait>(
        node_, kinematics_, params_.single_pose,
        bodyPose(kinematics_->getLaydownToePositions(), CPose(),
                 COrientation(0.0 * deg, -params_.single_pose.head_max_pitch, 0.0 * deg)));
    gaits_[MovementRequest::SEQUENCE_LEGS_WAVE] =
        std::make_shared<CLegWaveGait>(node_, kinematics_, params_.leg_wave);
    gaits_[MovementRequest::SEQUENCE_LOOK] =
        std::make_shared<CYawSequenceGait>(node_, kinematics_, CYawSequenceGait::Sweep::OneSide,
                                           params_.look.head_max_yaw, params_.look.torso_max_yaw);
    gaits_[MovementRequest::CONTINUOUS_MOVE] = std::make_shared<CMoveCombinedGait>(
        node_, kinematics_, params_.wave, params_.ripple, params_.tripod, params_.move_combined);
    gaits_[MovementRequest::CONTINUOUS_RUNNING] =
        std::make_shared<CRunningGait>(node_, kinematics_, params_.running);
    gaits_[MovementRequest::SEQUENCE_STAND_UP] = std::make_shared<CSinglePoseGait>(
        node_, kinematics_, params_.single_pose,
        bodyPose(kinematics_->getStandingToePositions(), CPose(), COrientation()));
    gaits_[MovementRequest::SEQUENCE_TESTLEGS] =
        std::make_shared<CTestLegsGait>(node_, kinematics_, params_.test_legs);
    gaits_[MovementRequest::SINGLE_POSE] =
        std::make_shared<CSinglePoseGait>(node_, kinematics_, params_.single_pose);
    gaits_[MovementRequest::CONTINUOUS_POSE] =
        std::make_shared<CContinuousPoseGait>(node_, kinematics_, params_.continuous_pose);
    gaits_[MovementRequest::SEQUENCE_WAITING] =
        std::make_shared<CWaitingGait>(node_, kinematics_, params_.waiting);
    gaits_[MovementRequest::SEQUENCE_WATCH] = std::make_shared<CYawSequenceGait>(
        node_, kinematics_, CYawSequenceGait::Sweep::BothSides, params_.watch.head_max_yaw, 0.0 * deg);

    gaits_[MovementRequest::HOLD_DISTANCE] = std::make_shared<CHoldDistanceGait>(node_, kinematics_, params_);
    gaits_[MovementRequest::LEVEL_TORSO] = std::make_shared<CLevelTorsoGait>(node_, kinematics_);

    // Default active gait (robot starts laying down)
    active_gait_ = gaits_[MovementRequest::SEQUENCE_LAYDOWN];
    active_request_ = createMsg("SEQUENCE_LAYDOWN", MovementRequest::SEQUENCE_LAYDOWN);
    pending_request_ = createMsg("NO_REQUEST", MovementRequest::NO_REQUEST);
}

CGaitController::~CGaitController() = default;

bool CGaitController::setGait(brain::MovementRequest request) {
    if (request.type != MovementRequest::NO_REQUEST && !gaits_.contains(request.type)) {
        RCLCPP_WARN(node_->get_logger(), "Unsupported gait: %u", request.type);
        return false;
    }
    if (request.type == MovementRequest::NO_REQUEST) {
        RCLCPP_DEBUG(node_->get_logger(), "CGaitController::setGait ignoring NO_REQUEST message");
        return false;
    }

    const bool sensor_gait =
        request.type == MovementRequest::HOLD_DISTANCE || request.type == MovementRequest::LEVEL_TORSO;
    if ((sensor_gait && (!isfinite(request.duration_s) || request.duration_s <= 0.0 * s ||
                         hasBodyPose(request.body_pose))) ||
        (request.type == MovementRequest::HOLD_DISTANCE &&
         (!request.distance_m || !isfinite(*request.distance_m) || *request.distance_m <= 0.0 * m)) ||
        (request.type != MovementRequest::HOLD_DISTANCE && request.distance_m)) {
        RCLCPP_WARN(node_->get_logger(), "Invalid sensor gait parameters");
        return false;
    }
    pending_request_ = createMsg("NO_REQUEST", MovementRequest::NO_REQUEST);

    // If there's an active gait and the same gait is requested again, handle
    // transient states (Stopped -> start, StopPending/Stopping -> cancelStop) and return.
    if (request.type == active_request_.type) {
        stop_requested_ = false;
        if ((request.type == MovementRequest::SINGLE_POSE || sensor_gait) &&
            active_gait_->state() != EGaitState::Stopped) {
            pending_request_ = request;
            active_gait_->requestStop();
            stop_requested_ = true;
            return true;
        }
        if (active_gait_->state() == EGaitState::Stopped) {
            configureGait(request);
            if (sensor_gait) active_request_ = request;
            RCLCPP_INFO(node_->get_logger(), "CGaitController::setGait starting %s", request.name.c_str());
            active_gait_->start(request.duration_s, request.direction);
            if (sensor_gait && on_gait_changed) on_gait_changed(request);
        }
        if (active_gait_->state() == EGaitState::StopPending ||
            active_gait_->state() == EGaitState::Stopping) {
            active_gait_->cancelStop();
        }
        return true;
    }

    // If the active gait is not yet stopped, request stop and set pending type
    if (active_gait_->state() != EGaitState::Stopped) {
        pending_request_ = request;
        active_gait_->requestStop();
        stop_requested_ = !motionStopped();
        return true;
    }

    // Otherwise, switch immediately
    switchGait(request);
    return true;
}

void CGaitController::switchGait(brain::MovementRequest request) {
    stop_requested_ = false;
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       "CGaitController: switching to " << request.name << " (type=" << +request.type << ")");
    pending_request_ = createMsg("NO_REQUEST", MovementRequest::NO_REQUEST);
    active_request_ = request;

    active_gait_ = gaits_[request.type];
    configureGait(request);
    active_gait_->start(request.duration_s, request.direction);

    // Publish the new movement type
    if (on_gait_changed) on_gait_changed(request);
}

void CGaitController::configureGait(const MovementRequest& request) {
    if (request.type == MovementRequest::SINGLE_POSE)
        std::static_pointer_cast<CSinglePoseGait>(active_gait_)->setTarget(request.body_pose);
    else if (request.type == MovementRequest::HOLD_DISTANCE)
        std::static_pointer_cast<CHoldDistanceGait>(active_gait_)->setDistance(*request.distance_m);
}

bool CGaitController::updateSelectedGait(const Velocity& velocity, CPose torso, COrientation head,
                                         units::Duration elapsed_s) {
    if (pending_request_.type != MovementRequest::NO_REQUEST &&
        active_gait_->state() == EGaitState::Stopped) {
        switchGait(pending_request_);
    }

    bool progressed = false;
    if (auto continuous = std::dynamic_pointer_cast<IContinuousGait>(active_gait_)) {
        progressed = continuous->updateTimed(velocity, torso, head, elapsed_s);
    } else if (auto sequence = std::dynamic_pointer_cast<ISequenceGait>(active_gait_)) {
        progressed = sequence->updateTimed(elapsed_s);
    }
    if (motionStopped()) stop_requested_ = false;
    return progressed;
}

bool CGaitController::finishesAutomatically() const {
    switch (currentGait()) {
        case MovementRequest::HOLD_DISTANCE:
        case MovementRequest::LEVEL_TORSO:
        case MovementRequest::SINGLE_POSE:
        case MovementRequest::SEQUENCE_LAYDOWN:
        case MovementRequest::SEQUENCE_STAND_UP:
        case MovementRequest::SEQUENCE_LOOK:
        case MovementRequest::SEQUENCE_WATCH:
        case MovementRequest::SEQUENCE_CLAP:
        case MovementRequest::SEQUENCE_HIGH_FIVE:
        case MovementRequest::SEQUENCE_TESTLEGS:
            return true;
        default:
            return false;
    }
}

void CGaitController::requestStopSelectedGait() {
    pending_request_ = createMsg("NO_REQUEST", MovementRequest::NO_REQUEST);
    active_gait_->requestStop();
    stop_requested_ = !motionStopped();
}

}  // namespace brain
