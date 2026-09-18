#pragma once

#include <array>
#include <magic_enum.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "rumblex_utils/filters.hpp"
#include "rumblex_utils/geometry.hpp"
#include "rumblex_utils/msg_twist.hpp"
#include "rumblex_utils/simpletimer.hpp"
#include "requester/gait_parameters.hpp"
#include "requester/igaits.hpp"
#include "requester/kinematics.hpp"
#include "requester/types.hpp"

namespace rumblex_movement {

enum class EMoveCombinedGaitType { Wave, Ripple, Tripod };

class CMoveCombinedGait : public IContinuousGait {
   public:
    CMoveCombinedGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics,
                      Parameters::Wave& wave_params, Parameters::Ripple& ripple_params,
                      Parameters::Tripod& tripod_params, Parameters::MoveCombined& combined_params);
    ~CMoveCombinedGait() override = default;

    void start(double duration_s, uint8_t direction) override;
    bool update(const geometry_msgs::msg::Twist& velocity, const CPose& body,
                const COrientation& head) override;
    void requestStop() override;
    void cancelStop() override;
    EGaitState state() const override {
        return state_;
    }

   private:
    struct LegMotion {
        double step{0.0};
        double lift{0.0};
    };

    struct LegPhaseInfo {
        ELegIndex index;
        double phase_offset;
    };

    EMoveCombinedGaitType selectGait(double combined_mag) const;

    LegMotion computeWaveLegMotion(ELegIndex index, double phase) const;
    LegMotion computeRippleLegMotion(ELegIndex index, double phase) const;
    LegMotion computeTripodLegMotion(ELegIndex index, double phase) const;
    LegMotion computeLegMotion(EMoveCombinedGaitType gait, ELegIndex index, double phase) const;

    double getFactorVelocityCycleTime(EMoveCombinedGaitType gait) const;
    double getHeadAmplitudeYawDeg(EMoveCombinedGaitType gait) const;

    // Wave: 6 individual legs, each offset by π/3
    static constexpr double kWaveTransferPhase = 2.0 * M_PI / 6.0;
    static constexpr double kWaveSupportPhase = 10.0 * M_PI / 6.0;
    static constexpr std::array<LegPhaseInfo, 6> kWaveLegPhases = {{
        {ELegIndex::RightBack, 0.0},
        {ELegIndex::RightMid, 1.0 * M_PI / 3.0},
        {ELegIndex::RightFront, 2.0 * M_PI / 3.0},
        {ELegIndex::LeftBack, 3.0 * M_PI / 3.0},
        {ELegIndex::LeftMid, 4.0 * M_PI / 3.0},
        {ELegIndex::LeftFront, 5.0 * M_PI / 3.0},
    }};

    // Ripple: 3 groups of 2 diagonal legs, each offset by 2π/3
    static constexpr double kRippleTransferPhase = 2.0 * M_PI / 3.0;
    static constexpr double kRippleSupportPhase = 4.0 * M_PI / 3.0;
    static constexpr std::array<LegPhaseInfo, 6> kRippleLegPhases = {{
        {ELegIndex::RightBack, 0.0},
        {ELegIndex::LeftFront, 0.0},
        {ELegIndex::RightMid, 2.0 * M_PI / 3.0},
        {ELegIndex::LeftMid, 2.0 * M_PI / 3.0},
        {ELegIndex::RightFront, 4.0 * M_PI / 3.0},
        {ELegIndex::LeftBack, 4.0 * M_PI / 3.0},
    }};

    // Tripod: 2 groups of 3 legs
    static constexpr std::array<ELegIndex, 3> kTripodFirstGroup = {ELegIndex::RightFront, ELegIndex::LeftMid,
                                                                   ELegIndex::RightBack};

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CKinematics> kinematics_;
    Parameters::Wave wave_params_;
    Parameters::Ripple ripple_params_;
    Parameters::Tripod tripod_params_;
    Parameters::MoveCombined combined_params_;

    EGaitState state_ = EGaitState::Stopped;
    double phase_ = 0.0;
    EMoveCombinedGaitType active_gait_type_ = EMoveCombinedGaitType::Wave;

    // Blending state for jerk-free transitions
    bool is_blending_ = false;
    double blend_alpha_ = 1.0;
    std::map<ELegIndex, CPosition> blend_start_positions_;
    double blend_start_head_yaw_deg_ = 0.0;

    CSimpleTimer no_velocity_timer_;
    std::map<ELegIndex, CPosition> target_positions_;
    CPose body_old_;

    geometry_msgs::msg::Twist velocity_{geometry_msgs::msg::Twist()};
};

}  // namespace rumblex_movement
