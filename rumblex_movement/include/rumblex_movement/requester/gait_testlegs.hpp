#pragma once

#include <array>
#include <map>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "requester/gait_parameters.hpp"
#include "requester/igaits.hpp"
#include "requester/kinematics.hpp"
#include "requester/types.hpp"

namespace rumblex_movement {

class CTestLegsGait : public ISequenceGait {
   public:
    CTestLegsGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics,
                  Parameters::TestLegs& params);
    ~CTestLegsGait() override = default;

    void start(double duration_s, uint8_t direction) override;
    bool update() override;
    void requestStop() override;
    void cancelStop() override;
    [[nodiscard]] EGaitState state() const override {
        return state_;
    }

   private:
    enum class Stage { Raise, Hold, Lower };

    void captureBaseAngles();
    void applyStageForCurrentLeg();
    void advanceStage();
    bool hasCurrentLeg() const;
    ELegIndex currentLeg() const;
    void restoreLeg(ELegIndex index);
    void restoreAllLegs();

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CKinematics> kinematics_;
    Parameters::TestLegs params_;

    EGaitState state_ = EGaitState::Stopped;
    Stage stage_ = Stage::Raise;

    double coxa_delta_deg_ = 10.0;
    double femur_delta_deg_ = 10.0;
    double tibia_delta_deg_ = 10.0;
    double default_stage_duration_ = 0.5;
    double min_stage_duration_ = 0.05;
    double stage_duration_ = 0.5;

    rclcpp::Time stage_start_time_;
    bool stage_action_applied_ = false;

    inline static const std::array<ELegIndex, 6> kDefaultLegOrder = {
        ELegIndex::RightFront, ELegIndex::RightMid, ELegIndex::RightBack,
        ELegIndex::LeftBack,   ELegIndex::LeftMid,  ELegIndex::LeftFront};
    std::vector<ELegIndex> leg_order_ =
        std::vector<ELegIndex>(kDefaultLegOrder.begin(), kDefaultLegOrder.end());
    size_t current_leg_index_ = 0;

    std::map<ELegIndex, CLegAngles> base_leg_angles_;
};

}  // namespace rumblex_movement
