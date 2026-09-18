#include <gtest/gtest.h>

#include <map>

#include "handler/leg_servo_conversion.hpp"
#include "rumblex_interfaces/msg/servo_angles.hpp"
#include "rumblex_interfaces/msg/servo_index.hpp"

using namespace rumblex_movement;
using leg_servo_conversion::appendHeadServoTargets;
using leg_servo_conversion::appendLegServoTargets;
using leg_servo_conversion::buildServoTargets;
using leg_servo_conversion::servoAnglesMsgToLegAngles;

using rumblex_interfaces::msg::ServoIndex;

TEST(LegServoConversionTest, LegAnglesMapToServoTargets) {
    std::map<ELegIndex, CLegAngles> legAngles;
    legAngles[ELegIndex::RightFront] = CLegAngles(10.0, 20.0, 30.0);
    legAngles[ELegIndex::LeftBack] = CLegAngles(-5.0, -10.0, -15.0);

    std::map<uint32_t, double> targetAngles;
    appendLegServoTargets(legAngles, targetAngles);

    ASSERT_NE(targetAngles.find(ServoIndex::LEG_RIGHT_FRONT_COXA), targetAngles.end());
    EXPECT_DOUBLE_EQ(targetAngles.at(ServoIndex::LEG_RIGHT_FRONT_COXA), 10.0);
    EXPECT_DOUBLE_EQ(targetAngles.at(ServoIndex::LEG_RIGHT_FRONT_FEMUR), 20.0);
    EXPECT_DOUBLE_EQ(targetAngles.at(ServoIndex::LEG_RIGHT_FRONT_TIBIA), 30.0);

    ASSERT_NE(targetAngles.find(ServoIndex::LEG_LEFT_BACK_COXA), targetAngles.end());
    EXPECT_DOUBLE_EQ(targetAngles.at(ServoIndex::LEG_LEFT_BACK_COXA), -5.0);
    EXPECT_DOUBLE_EQ(targetAngles.at(ServoIndex::LEG_LEFT_BACK_FEMUR), -10.0);
    EXPECT_DOUBLE_EQ(targetAngles.at(ServoIndex::LEG_LEFT_BACK_TIBIA), -15.0);

    COrientation head(0.0, -7.5, 5.0);
    auto combined = buildServoTargets(head, legAngles);
    EXPECT_DOUBLE_EQ(combined.at(ServoIndex::HEAD_YAW), 5.0);
    EXPECT_DOUBLE_EQ(combined.at(ServoIndex::HEAD_PITCH), -7.5);
}

TEST(LegServoConversionTest, ServoAnglesMessageMapsToLegAngles) {
    rumblex_interfaces::msg::ServoAngles msg;
    msg.current_angles[0].name = "right_front_coxa";
    msg.current_angles[0].angle_deg = 12.5F;
    msg.current_angles[1].name = "RIGHT_FRONT_FEMUR";
    msg.current_angles[1].angle_deg = -4.0F;
    msg.current_angles[2].name = "Left_Back_Tibia";
    msg.current_angles[2].angle_deg = 3.25F;

    auto result = servoAnglesMsgToLegAngles(msg);

    ASSERT_NE(result.find(ELegIndex::RightFront), result.end());
    EXPECT_DOUBLE_EQ(result.at(ELegIndex::RightFront).coxa_deg, 12.5);
    EXPECT_DOUBLE_EQ(result.at(ELegIndex::RightFront).femur_deg, -4.0);

    ASSERT_NE(result.find(ELegIndex::LeftBack), result.end());
    EXPECT_DOUBLE_EQ(result.at(ELegIndex::LeftBack).tibia_deg, 3.25);
}
