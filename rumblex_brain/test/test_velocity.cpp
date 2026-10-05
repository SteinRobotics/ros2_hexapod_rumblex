#include <gtest/gtest.h>

#include <array>
#include <limits>

#include "gait/gait_move_combined.hpp"
#include "gait/gait_running.hpp"
#include "test_helpers.hpp"
#include "velocity.hpp"

using namespace brain;

class VelocityTest : public ::testing::Test {
   protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        auto parameters = test_helpers::defaultRobotParameters();
        for (auto& parameter : parameters)
            if (parameter.get_name() == "max_velocity_linear")
                parameter = rclcpp::Parameter("max_velocity_linear", 0.1);
            else if (parameter.get_name() == "max_velocity_rotation")
                parameter = rclcpp::Parameter("max_velocity_rotation", 0.4);
        rclcpp::NodeOptions options;
        options.parameter_overrides(parameters);
        node = std::make_shared<rclcpp::Node>("velocity_test", options);
        model = std::make_shared<CPoseModel>(node);
        model->moveTorso(model->getStandingToePositions());
        params = Parameters::declare(node);
    }
    void TearDown() override {
        rclcpp::shutdown();
    }
    CMoveCombinedGait gait() {
        return CMoveCombinedGait(node, model, params.wave, params.ripple, params.tripod,
                                 params.move_combined);
    }
    std::shared_ptr<rclcpp::Node> node;
    std::shared_ptr<CPoseModel> model;
    Parameters params;
};

TEST_F(VelocityTest, FullSingleAxisDemandSelectsTripodImmediately) {
    for (int axis : {0, 1, 2}) {
        for (double sign : {-1.0, 1.0}) {
            auto walking = gait();
            walking.start(0.0, 0);
            geometry_msgs::msg::Twist command;
            if (axis == 0) command.linear.x = sign * 0.1;
            if (axis == 1) command.linear.y = sign * 0.1;
            if (axis == 2) command.angular.z = sign * 0.4;
            walking.updateTimed(command, CPose(), COrientation(), 0.02);
            EXPECT_EQ(walking.activeGaitType(), EMoveCombinedGaitType::Tripod);
        }
    }
}

TEST_F(VelocityTest, LateralCommandMovesSwingWithBodyAndStanceAgainstBody) {
    for (double sign : {-1.0, 1.0}) {
        model->moveTorso(model->getStandingToePositions());
        const auto before = model->getToePositions();
        auto walking = gait();
        walking.start(0.0, 0);
        geometry_msgs::msg::Twist command;
        command.linear.y = sign * 0.01;
        ASSERT_TRUE(walking.updateTimed(command, CPose(), COrientation(), 0.02));
        ASSERT_EQ(walking.activeGaitType(), EMoveCombinedGaitType::Wave);
        const auto after = model->getToePositions();
        EXPECT_GT(sign * (after.at(ELegIndex::RightBack).y - before.at(ELegIndex::RightBack).y),
                  0.0 * units::m);
        EXPECT_LT(sign * (after.at(ELegIndex::RightFront).y - before.at(ELegIndex::RightFront).y),
                  0.0 * units::m);
    }
}

TEST_F(VelocityTest, RotationCommandKeepsStanceFootFixedInWorld) {
    for (double sign : {-1.0, 1.0}) {
        model->moveTorso(model->getStandingToePositions());
        const auto before = model->getToePositions().at(ELegIndex::RightFront);
        auto walking = gait();
        walking.start(0.0, 0);
        geometry_msgs::msg::Twist command;
        command.angular.z = sign * 0.04;
        ASSERT_TRUE(walking.updateTimed(command, CPose(), COrientation(), 0.02));
        ASSERT_EQ(walking.activeGaitType(), EMoveCombinedGaitType::Wave);
        const auto after = model->getToePositions().at(ELegIndex::RightFront);
        const double cross =
            (before.x * after.y - before.y * after.x).numerical_value_in(units::m * units::m);
        // Clockwise body rotation (negative Z) requires anticlockwise stance motion.
        EXPECT_LT(sign * cross, 0.0);
        EXPECT_NEAR(std::hypot(after.x.numerical_value_in(units::m), after.y.numerical_value_in(units::m)),
                    std::hypot(before.x.numerical_value_in(units::m), before.y.numerical_value_in(units::m)),
                    1e-12);
    }
}

TEST_F(VelocityTest, ThresholdsHysteresisAndLargeChangesUseContactBoundaries) {
    auto walking = gait();
    walking.start(0.0, 0);
    auto demand = [&](double value) {
        geometry_msgs::msg::Twist command;
        command.linear.y = value * 0.1;
        // One second is the upper bound on a segment; two seconds crosses it.
        for (int i = 0; i < 100; ++i) walking.updateTimed(command, CPose(), COrientation(), 0.02);
        return walking.activeGaitType();
    };
    EXPECT_EQ(demand(0.1), EMoveCombinedGaitType::Wave);
    EXPECT_EQ(demand(0.32), EMoveCombinedGaitType::Wave);
    EXPECT_EQ(demand(0.4), EMoveCombinedGaitType::Ripple);
    EXPECT_EQ(demand(0.29), EMoveCombinedGaitType::Ripple);
    EXPECT_EQ(demand(0.7), EMoveCombinedGaitType::Tripod);
    EXPECT_EQ(demand(0.59), EMoveCombinedGaitType::Tripod);
    EXPECT_EQ(demand(0.1), EMoveCombinedGaitType::Wave);
    EXPECT_EQ(demand(1.0), EMoveCombinedGaitType::Tripod);
}

TEST_F(VelocityTest, MaximumRequestCompletesSlowSwingBeforeSelectingTripodWithinOneSecond) {
    auto walking = gait();
    walking.start(0.0, 0);
    geometry_msgs::msg::Twist command;
    command.linear.y = 0.001;
    for (int i = 0; i < 4; ++i) walking.updateTimed(command, CPose(), COrientation(), 0.02);
    command.linear.y = 0.1;
    walking.updateTimed(command, CPose(), COrientation(), 0.02);
    EXPECT_EQ(walking.activeGaitType(), EMoveCombinedGaitType::Wave);
    int ticks = 1;
    while (walking.activeGaitType() != EMoveCombinedGaitType::Tripod && ticks < 50) {
        walking.updateTimed(command, CPose(), COrientation(), 0.02);
        ++ticks;
    }
    EXPECT_EQ(walking.activeGaitType(), EMoveCombinedGaitType::Tripod);
    EXPECT_LE(ticks * 0.02, 1.0);
}

TEST_F(VelocityTest, LegacyLimitsMustMatchAuthoritativeLimits) {
    auto parameters = test_helpers::defaultRobotParameters();
    parameters.emplace_back("gait.move_combined.max_linear_velocity_m_s", 0.1);
    rclcpp::NodeOptions options;
    options.parameter_overrides(parameters);
    auto mismatched = std::make_shared<rclcpp::Node>("mismatched_velocity_limits", options);
    EXPECT_THROW(Parameters::declare(mismatched), std::invalid_argument);
}

TEST(VelocityLimits, PlanarMagnitudeDirectionUnitsAndInvalidCommands) {
    geometry_msgs::msg::Twist input, output;
    input.linear.x = 0.3;
    input.linear.y = -0.4;
    input.angular.z = 0.1;
    ASSERT_TRUE(limitVelocity(input, 0.1, 0.02, output));
    EXPECT_NEAR(output.linear.x, 0.06, 1e-12);
    EXPECT_NEAR(output.linear.y, -0.08, 1e-12);
    EXPECT_DOUBLE_EQ(output.angular.z, 0.02);
    input.linear.x = 0.01;
    input.linear.y = 0.0;
    input.angular.z = -0.01;
    ASSERT_TRUE(limitVelocity(input, 0.1, 0.02, output));
    EXPECT_DOUBLE_EQ(output.linear.x, 0.01);
    EXPECT_DOUBLE_EQ(output.angular.z, -0.01);
    input.linear.y = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(limitVelocity(input, 0.1, 0.02, output));
}

// Recover body displacement solely from common supporting feet, independent of commands.
std::array<double, 3> displacement(const std::map<ELegIndex, CPosition>& previous,
                                   const std::map<ELegIndex, CPosition>& current,
                                   const std::map<ELegIndex, CPosition>& standing) {
    std::vector<std::array<double, 4>> pairs;
    double ax = 0.0, ay = 0.0, bx = 0.0, by = 0.0;
    for (const auto& [index, a] : previous) {
        const auto& b = current.at(index);
        if (std::abs((a.z - standing.at(index).z).numerical_value_in(units::m)) > 1e-6 ||
            std::abs((b.z - standing.at(index).z).numerical_value_in(units::m)) > 1e-6)
            continue;
        const std::array pair = {a.x.numerical_value_in(units::m), a.y.numerical_value_in(units::m),
                                 b.x.numerical_value_in(units::m), b.y.numerical_value_in(units::m)};
        pairs.push_back(pair);
        ax += pair[0];
        ay += pair[1];
        bx += pair[2];
        by += pair[3];
    }
    EXPECT_GE(pairs.size(), 2u);
    if (pairs.size() < 2) return {};
    ax /= pairs.size();
    ay /= pairs.size();
    bx /= pairs.size();
    by /= pairs.size();
    double dot = 0.0, cross = 0.0;
    for (const auto& p : pairs) {
        dot += (p[2] - bx) * (p[0] - ax) + (p[3] - by) * (p[1] - ay);
        cross += (p[2] - bx) * (p[1] - ay) - (p[3] - by) * (p[0] - ax);
    }
    const double yaw = std::atan2(cross, dot);
    return {ax - std::cos(yaw) * bx + std::sin(yaw) * by, ay - std::sin(yaw) * bx - std::cos(yaw) * by, yaw};
}

TEST_F(VelocityTest, SupportingFootMotionMatchesPhysicalCommandsAndElapsedTime) {
    for (bool running : {false, true}) {
        for (const auto& values : std::vector<std::array<double, 3>>{{0.001, 0.0, 0.0},
                                                                     {0.02, 0.0, 0.0},
                                                                     {0.0, -0.05, 0.0},
                                                                     {-0.1, 0.0, 0.0},
                                                                     {0.0, 0.1, 0.0},
                                                                     {0.0, 0.0, 0.01},
                                                                     {0.0, 0.0, -0.02},
                                                                     {0.0, 0.0, 0.2},
                                                                     {0.0, 0.0, -0.4},
                                                                     {0.04, -0.03, 0.015},
                                                                     {0.07, 0.07, -0.02},
                                                                     {0.07, 0.07, -0.2}}) {
            for (bool jitter : {false, true}) {
                SCOPED_TRACE(std::to_string(running) + " " + std::to_string(values[0]) + " " +
                             std::to_string(values[1]) + " " + std::to_string(values[2]) + " " +
                             std::to_string(jitter));
                model->moveTorso(model->getStandingToePositions());
                std::unique_ptr<IContinuousGait> walking;
                if (running)
                    walking = std::make_unique<CRunningGait>(node, model, params.running);
                else
                    walking = std::make_unique<CMoveCombinedGait>(gait());
                walking->start(0.0, 0);
                geometry_msgs::msg::Twist command;
                command.linear.x = values[0];
                command.linear.y = values[1];
                command.angular.z = values[2];
                // Warm up enough for every foot to complete its first swing.
                for (int i = 0; i < 600; ++i) walking->updateTimed(command, CPose(), COrientation(), 0.02);
                std::array<double, 3> sum{};
                double time = 0.0;
                for (int i = 0; i < 3000; ++i) {
                    const double dt = jitter ? (i % 2 ? 0.025 : 0.015) : 0.02;
                    const auto previous = model->getToePositions();
                    walking->updateTimed(command, CPose(), COrientation(), dt);
                    const auto delta =
                        displacement(previous, model->getToePositions(), model->getStandingToePositions());
                    // Convert each finite body-frame displacement back to twist integrals.
                    const double angle = delta[2];
                    const double a = std::abs(angle) < 1e-9 ? 1.0 : std::sin(angle) / angle;
                    const double b = std::abs(angle) < 1e-9 ? angle / 2.0 : (1.0 - std::cos(angle)) / angle;
                    sum[0] += (a * delta[0] + b * delta[1]) / (a * a + b * b);
                    sum[1] += (-b * delta[0] + a * delta[1]) / (a * a + b * b);
                    sum[2] += angle;
                    time += dt;
                }
                for (size_t axis = 0; axis < 3; ++axis)
                    EXPECT_NEAR(sum[axis] / time, values[axis],
                                std::max(1e-7, std::abs(values[axis]) * 0.02));
            }
        }
    }
}

TEST_F(VelocityTest, InvalidElapsedTimeHoldsPoseAndDoesNotCatchUpAcrossGap) {
    auto walking = gait();
    walking.start(0.0, 0);
    geometry_msgs::msg::Twist command;
    command.linear.y = 0.1;
    walking.updateTimed(command, CPose(), COrientation(), 0.02);
    const auto before = model->getToePositions();
    for (double dt : {0.0, -0.01, 1.0, std::numeric_limits<double>::quiet_NaN()}) {
        EXPECT_FALSE(walking.updateTimed(command, CPose(), COrientation(), dt));
        EXPECT_EQ(model->getToePositions(), before);
    }
    EXPECT_TRUE(walking.updateTimed(command, CPose(), COrientation(), 0.02));
}
