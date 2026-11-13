#include "action_decoder.hpp"

#include "rumblex_utils/body_pose.hpp"
#include "yaml_fields.hpp"
namespace brain::sequence_detail {
using namespace mp_units;
using namespace mp_units::si::unit_symbols;
namespace {
// Named YAML legs map to the fixed ROS leg order; list input remains compatible.
template <typename Parse>
void parseLegs(const YAML::Node& node, Parse parse, bool requireAll = false) {
    static const std::array<std::string, 6> names = {"right_front", "right_mid", "right_back",
                                                     "left_front",  "left_mid",  "left_back"};
    if (node.IsSequence()) {
        if (node.size() != names.size()) throw std::runtime_error("Leg lists require six entries");
        for (size_t i = 0; i < names.size(); ++i) parse(i, node[i]);
        return;
    }
    keys(node, {"right_front", "right_mid", "right_back", "left_front", "left_mid", "left_back"});
    if (node.size() == 0 || (requireAll && node.size() != names.size()))
        throw std::runtime_error(requireAll ? "All six toe positions are required" : "Empty leg mapping");
    for (size_t i = 0; i < names.size(); ++i)
        if (node[names[i]]) parse(i, node[names[i]]);
}
InterpolationType parseInterpolation(const YAML::Node& value) {
    if (!value["interpolation"]) return InterpolationType::Linear;
    const auto name = value["interpolation"].as<std::string>();
    if (name == "linear") return InterpolationType::Linear;
    if (name == "sin_squared") return InterpolationType::SinSquared;
    if (name == "sin") return InterpolationType::Sin;
    if (name == "cos") return InterpolationType::Cos;
    throw std::runtime_error("Unknown interpolation: " + name);
}
void parseVector(const YAML::Node& node, geometry_msgs::msg::Vector3& vector) {
    keys(node, {"x", "y", "z"});
    vector.x = valueOr(node, "x", 0.0);
    vector.y = valueOr(node, "y", 0.0);
    vector.z = valueOr(node, "z", 0.0);
}
void parseOrientation(const YAML::Node& node, rumblex_interfaces::msg::Orientation& orientation) {
    keys(node, {"roll", "pitch", "yaw"});
    orientation.roll = valueOr(node, "roll", 0.0);
    orientation.pitch = valueOr(node, "pitch", 0.0);
    orientation.yaw = valueOr(node, "yaw", 0.0);
}
rumblex_interfaces::msg::BodyPose parseBodyPose(const YAML::Node& value) {
    keys(value, {"duration_s", "interpolation", "torso_pose", "head_orientation", "toe_positions"});
    rumblex_interfaces::msg::BodyPose pose;
    if (value["torso_pose"]) {
        pose.has_torso_pose = true;
        const auto torso = value["torso_pose"];
        keys(torso, {"position", "orientation"});
        if (torso["position"]) parseVector(torso["position"], pose.torso_pose.position);
        if (torso["orientation"]) parseOrientation(torso["orientation"], pose.torso_pose.orientation);
    }
    if (value["head_orientation"]) {
        pose.has_head_orientation = true;
        parseOrientation(value["head_orientation"], pose.head_orientation);
    }
    if (value["toe_positions"]) {
        const auto toes = value["toe_positions"];
        pose.has_toe_positions = true;
        parseLegs(
            toes,
            [&](size_t i, const YAML::Node& toe) {
                if (!toe.IsMap() || toe.size() != 3)
                    throw std::runtime_error("Every toe requires x, y and z");
                parseVector(toe, pose.toe_positions[i]);
            },
            true);
    }
    if (!rumblex_geometry::hasBodyPose(pose) || !rumblex_geometry::validBodyPose(pose))
        throw std::runtime_error("body_pose requires at least one finite component");
    return pose;
}
}  // namespace
std::vector<std::shared_ptr<RequestBase>> ActionDecoder::parseActionGroup(const YAML::Node& action) {
    static const std::map<std::string, std::string> actionTypes = {
        {"say", "RequestTalking"},         {"chat", "RequestChat"},     {"music", "RequestMusic"},
        {"listening", "RequestListening"}, {"system", "RequestSystem"}, {"movement", "RequestMovementType"},
        {"velocity", "RequestVelocity"}};
    std::map<std::string, YAML::Node> ordered;
    for (const auto& entry : action) {
        auto key = entry.first.as<std::string>();
        if (actionTypes.contains(key))
            key = actionTypes.at(key);
        else if (key != "body_pose")
            throw std::runtime_error("Unknown action: " + key);
        if (!ordered.emplace(key, entry.second).second)
            throw std::runtime_error("Duplicate action or alias: " + key);
    }
    auto movement = std::shared_ptr<RequestMovementType>();
    if (ordered.contains("body_pose")) {
        if (ordered.contains("RequestMovementType"))
            throw std::runtime_error("body_pose conflicts with another movement target");
        const auto value = ordered.at("body_pose");
        movement = std::make_shared<RequestMovementType>();
        movement->movementRequest.body_pose = parseBodyPose(value);
        movement->movementRequest.interpolation = parseInterpolation(value);
        movement->movementRequest.type = MovementRequest::SINGLE_POSE;
        movement->movementRequest.name = "SINGLE_POSE";
        const double duration = valueOr(value, "duration_s", kDefaultPoseDuration.numerical_value_in(s));
        if (!std::isfinite(duration) || duration < kMinimumAbsolutePoseDuration.numerical_value_in(s))
            throw std::runtime_error("body_pose duration_s must be finite and at least 0.1 seconds");
        movement->movementRequest.duration_s = duration * s;
        ordered.erase("body_pose");
        ordered.emplace("RequestMovementType", YAML::Node());
    } else if (ordered.contains("RequestMovementType")) {
        movement = createRequestMovementType(ordered.at("RequestMovementType"));
        if (!movement) throw std::runtime_error("Invalid movement action");
    }
    if (movement && !movement->movementRequest.validPoseOptions())
        throw std::runtime_error("Invalid pose options");
    if (movement && movement->movementRequest.type == MovementRequest::SINGLE_POSE &&
        !movement->movementRequest.hasPoseTarget())
        throw std::runtime_error("SINGLE_POSE requires a body_pose target");
    if (movement && movement->movementRequest.isSensorGait() &&
        (ordered.contains("RequestVelocity") ||
         rumblex_geometry::hasBodyPose(movement->movementRequest.body_pose)))
        throw std::runtime_error("Sensor gaits own their velocity and pose targets");
    std::vector<std::shared_ptr<RequestBase>> requests;
    // Canonical names preserve dispatch order even when YAML actionTypes change.
    for (const auto& [type, value] : ordered) {
        std::shared_ptr<RequestBase> request;
        if (type != "RequestMovementType") {
            if (value.IsMap()) {
                if (type == "RequestTalking" || type == "RequestChat")
                    keys(value, {"text", "language", "minDuration"});
                else if (type == "RequestMusic")
                    keys(value, {"song", "volume", "minDuration", "background", "fade_out_s"});
                else if (type == "RequestListening")
                    keys(value, {"active", "minDuration"});
                else if (type == "RequestSystem")
                    keys(value, {"turnOffServoRelay", "systemShutdown", "minDuration"});
                else if (type == "RequestVelocity")
                    keys(value, {"linear", "angular", "minDuration"});
            } else if (!value.IsScalar()) {
                throw std::runtime_error("Expected scalar or mapping for " + type);
            }
        }
        if (type == "RequestTalking")
            request = createRequestTalking(value);
        else if (type == "RequestChat")
            request = createRequestChat(value);
        else if (type == "RequestMusic")
            request = createRequestMusic(value);
        else if (type == "RequestListening")
            request = createRequestListening(value);
        else if (type == "RequestSystem")
            request = createRequestSystem(value);
        else if (type == "RequestMovementType")
            request = movement;
        else if (type == "RequestVelocity")
            request = createRequestMoveVelocity(value);
        else
            throw std::runtime_error("Unknown action: " + type);
        if (!request || !isfinite(request->minDuration) || request->minDuration < 0.0 * s)
            throw std::runtime_error("Invalid action: " + type);
        requests.push_back(request);
    }
    return requests;
}

std::shared_ptr<RequestTalking> ActionDecoder::createRequestTalking(const YAML::Node& value) {
    try {
        auto request = std::make_shared<RequestTalking>();

        if (value.IsScalar()) {
            request->text = value.as<std::string>();
        } else if (value.IsMap()) {
            request->text = valueOr<std::string>(value, "text", "");
            request->language = valueOr<std::string>(value, "language", "de");
            request->minDuration = valueOr(value, "minDuration", 0.0) * s;
        }

        return request;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestTalking: %s", e.what());
    }

    return nullptr;
}

std::shared_ptr<RequestChat> ActionDecoder::createRequestChat(const YAML::Node& value) {
    try {
        auto request = std::make_shared<RequestChat>();

        if (value.IsScalar()) {
            request->text = value.as<std::string>();
        } else if (value.IsMap()) {
            request->text = valueOr<std::string>(value, "text", "");
            request->language = valueOr<std::string>(value, "language", "de");
            request->minDuration = valueOr(value, "minDuration", 0.0) * s;
        }

        return request;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestChat: %s", e.what());
    }

    return nullptr;
}

std::shared_ptr<RequestMusic> ActionDecoder::createRequestMusic(const YAML::Node& value) {
    try {
        auto request = std::make_shared<RequestMusic>();

        if (value.IsScalar()) {
            request->song = value.as<std::string>();
        } else if (value.IsMap()) {
            request->song = valueOr<std::string>(value, "song", "");
            request->fade_out_s = valueOr(value, "fade_out_s", 0.0f);
            if (!std::isfinite(request->fade_out_s) || request->fade_out_s < 0.0f ||
                request->fade_out_s > 60.0f || (request->fade_out_s > 0.0f && request->song != "STOP")) {
                throw std::runtime_error("Music fade_out_s must be 0-60 seconds and requires song: STOP");
            }
            request->volume = valueOr(value, "volume", 0.8f);
            if (!std::isfinite(request->volume)) throw std::runtime_error("Non-finite music volume");
            request->minDuration = valueOr(value, "minDuration", 0.0) * s;
        }

        request->background = request->song != "STOP";

        return request;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestMusic: %s", e.what());
    }

    return nullptr;
}

std::shared_ptr<RequestListening> ActionDecoder::createRequestListening(const YAML::Node& value) {
    try {
        auto request = std::make_shared<RequestListening>();

        if (value.IsScalar()) {
            request->active = value.as<bool>();
        } else if (value.IsMap()) {
            request->active = valueOr(value, "active", false);
            request->minDuration = valueOr(value, "minDuration", 0.0) * s;
        }

        return request;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestListening: %s", e.what());
    }

    return nullptr;
}

std::shared_ptr<RequestSystem> ActionDecoder::createRequestSystem(const YAML::Node& value) {
    try {
        if (value.IsMap()) {
            auto request = std::make_shared<RequestSystem>();
            request->turnOffServoRelay = valueOr(value, "turnOffServoRelay", false);
            request->systemShutdown = valueOr(value, "systemShutdown", false);
            request->minDuration = valueOr(value, "minDuration", 0.0) * s;
            return request;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestSystem: %s", e.what());
    }

    return nullptr;
}

std::shared_ptr<RequestMovementType> ActionDecoder::createRequestMovementType(const YAML::Node& value) {
    try {
        if (value.IsMap()) {
            keys(value, {"name", "direction", "duration_s", "minDuration", "body_pose", "distance_m",
                         "pose_delta", "joint_targets", "save_pose", "restore_pose", "interpolation"});
            brain::MovementRequest movementRequest;
            movementRequest.interpolation = parseInterpolation(value);
            if (value["body_pose"]) {
                if (value["body_pose"]["duration_s"]) throw std::runtime_error("Put duration_s on movement");
                if (value["body_pose"]["interpolation"])
                    throw std::runtime_error("Put interpolation on movement");
                movementRequest.body_pose = parseBodyPose(value["body_pose"]);
            }

            if (value["pose_delta"]) {
                const auto delta = value["pose_delta"];
                keys(delta, {"torso_translation", "toe_translation", "coxa_rotation_deg", "head_rotation",
                             "torso_rotation"});
                if (delta.size() == 0) throw std::runtime_error("Empty pose_delta");
                PoseDelta parsed;
                if (delta["head_rotation"]) parseOrientation(delta["head_rotation"], parsed.head_rotation);
                if (delta["torso_rotation"]) parseOrientation(delta["torso_rotation"], parsed.torso_rotation);
                if (delta["torso_translation"])
                    parseVector(delta["torso_translation"], parsed.torso_translation);
                for (const auto* field : {"toe_translation", "coxa_rotation_deg"}) {
                    if (!delta[field]) continue;
                    const auto values = delta[field];
                    parseLegs(values, [&](size_t i, const YAML::Node& leg) {
                        if (std::string(field) == "toe_translation")
                            parseVector(leg, parsed.toe_translation[i]);
                        else
                            parsed.coxa_rotation_deg[i] = leg.as<double>();
                    });
                }
                if (!parsed.valid()) throw std::runtime_error("Non-finite pose_delta");
                movementRequest.pose_delta = parsed;
            }

            movementRequest.save_pose = valueOr(value, "save_pose", false);
            movementRequest.restore_pose = valueOr(value, "restore_pose", false);
            if (value["joint_targets"]) {
                const auto targets = value["joint_targets"];
                parseLegs(targets, [&](size_t i, const YAML::Node& leg) {
                    if (targets.IsSequence() && leg.IsNull()) return;
                    keys(leg, {"torso_coxa", "coxa_femur", "femur_tibia"});
                    movementRequest.joint_targets[i] =
                        std::array<double, 3>{leg["torso_coxa"].as<double>(), leg["coxa_femur"].as<double>(),
                                              leg["femur_tibia"].as<double>()};
                });
                if (!movementRequest.hasJointTargets()) throw std::runtime_error("Empty joint_targets");
            }

            const auto typeStr = value["name"].as<std::string>();
            const auto type = nameToMovementType.find(typeStr);
            if (type == nameToMovementType.end())
                throw std::runtime_error("Unknown movement type: " + typeStr);
            movementRequest.type = type->second;
            movementRequest.name = typeStr;
            if (value["interpolation"] && movementRequest.type != MovementRequest::SINGLE_POSE)
                throw std::runtime_error("interpolation requires SINGLE_POSE");
            // Parse direction if present
            if (value["direction"]) {
                std::string directionStr = value["direction"].as<std::string>();
                if (directionStr == "CLOCKWISE") {
                    movementRequest.direction = brain::MovementRequest::CLOCKWISE;
                } else if (directionStr == "ANTICLOCKWISE") {
                    movementRequest.direction = brain::MovementRequest::ANTICLOCKWISE;
                } else {
                    throw std::runtime_error("Unknown direction: " + directionStr);
                }
            }
            if (value["duration_s"]) {
                movementRequest.duration_s = value["duration_s"].as<double>() * s;
            }

            if (value["distance_m"]) movementRequest.distance_m = value["distance_m"].as<double>() * m;
            if (!movementRequest.validDistance())
                throw std::runtime_error(movementRequest.type == MovementRequest::HOLD_DISTANCE
                                             ? "HOLD_DISTANCE requires positive finite distance_m"
                                             : "distance_m requires HOLD_DISTANCE");
            if (!movementRequest.hasFiniteDuration() || movementRequest.duration_s < 0.0 * s)
                throw std::runtime_error("Invalid movement duration");
            if (!movementRequest.validSensorPose())
                throw std::runtime_error("Sensor gaits require positive duration and own their pose targets");
            if (!movementRequest.validToeTargetType())
                throw std::runtime_error("Toe targets require SINGLE_POSE");
            if (!movementRequest.validRelativePose())
                throw std::runtime_error(
                    "pose_delta requires SINGLE_POSE, positive duration and no body_pose");
            auto request = std::make_shared<RequestMovementType>();
            request->movementRequest = movementRequest;
            request->minDuration = valueOr(value, "minDuration", 0.0) * s;
            return request;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestMovementType: %s", e.what());
    }

    return nullptr;
}

std::shared_ptr<RequestVelocity> ActionDecoder::createRequestMoveVelocity(const YAML::Node& value) {
    try {
        if (value.IsMap()) {
            geometry_msgs::msg::Twist velocity;

            if (value["linear"]) parseVector(value["linear"], velocity.linear);
            if (value["angular"]) parseVector(value["angular"], velocity.angular);
            for (double v : {velocity.linear.x, velocity.linear.y, velocity.linear.z, velocity.angular.x,
                             velocity.angular.y, velocity.angular.z})
                if (!std::isfinite(v)) throw std::runtime_error("Non-finite velocity");

            auto request = std::make_shared<RequestVelocity>();
            request->velocity = velocity;
            request->minDuration = valueOr(value, "minDuration", 0.0) * s;
            return request;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create RequestVelocity: %s", e.what());
    }

    return nullptr;
}

}  // namespace brain::sequence_detail
