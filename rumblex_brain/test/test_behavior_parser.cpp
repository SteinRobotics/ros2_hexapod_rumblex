#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "rumblex_brain/parser/behavior_parser.hpp"

namespace brain {
class BehaviorParserTest : public ::testing::Test {
   protected:
    static void SetUpTestSuite() {
        rclcpp::init(0, nullptr);
    }
    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }

    rclcpp::Node::SharedPtr node_ = std::make_shared<rclcpp::Node>("test_behavior_parser");
    CBehaviorParser parser_{node_};

    template <typename T>
    std::shared_ptr<T> request(const std::string& name, size_t group, size_t index) {
        const auto behavior = parser_.getBehavior(name);
        if (!behavior || group >= behavior->get().actionGroups.size() ||
            index >= behavior->get().actionGroups[group].size()) {
            ADD_FAILURE() << "Missing request in " << name;
            return nullptr;
        }
        return std::dynamic_pointer_cast<T>(behavior->get().actionGroups[group][index]);
    }
};

TEST_F(BehaviorParserTest, LoadsCatalogAndResolvesTriggers) {
    ASSERT_TRUE(parser_.parseFile(RUMBLEX_BEHAVIORS_FILE));
    EXPECT_EQ(parser_.getBehaviors().size(), 15u);
    EXPECT_FALSE(parser_.getBehavior("missing"));
    EXPECT_FALSE(parser_.getBehaviorForVoiceRequest("missing"));
    rumblex_interfaces::msg::JoystickRequest joystick;
    EXPECT_FALSE(parser_.getBehaviorForJoystickRequest(joystick));
    joystick.button_a = true;
    const auto watch = parser_.getBehaviorForJoystickRequest(joystick);
    ASSERT_TRUE(watch);
    EXPECT_EQ(watch->get().name, "normal_watch");
    EXPECT_EQ(watch->get().actionGroups.size(), 2u);
    const auto standup = parser_.getBehaviorForVoiceRequest("commandStandup");
    ASSERT_TRUE(standup);
    EXPECT_EQ(standup->get().name, "standup");
    auto movement = request<RequestMovementType>("standup", 0, 0);
    ASSERT_TRUE(movement);
    EXPECT_EQ(movement->movementRequest.type, MovementRequest::SEQUENCE_STAND_UP);
    EXPECT_DOUBLE_EQ(movement->movementRequest.duration_s, 1.5);
    auto music = request<RequestMusic>("standup", 0, 1);
    ASSERT_TRUE(music);
    EXPECT_EQ(music->song, "STOP");
}

TEST_F(BehaviorParserTest, PreservesDanceGroupsAndPoseUnits) {
    ASSERT_TRUE(parser_.parseFile(RUMBLEX_BEHAVIORS_FILE));
    const auto dance = parser_.getBehavior("dance_hot_dogs_for_breakfast");
    ASSERT_TRUE(dance);
    EXPECT_EQ(dance->get().actionGroups.size(), 18u);
    auto head = request<RequestHeadOrientation>(dance->get().name, 1, 0);
    auto movement = request<RequestMovementType>(dance->get().name, 1, 1);
    auto music = request<RequestMusic>(dance->get().name, 1, 2);
    auto pose = request<RequestSinglePose>(dance->get().name, 1, 3);
    ASSERT_TRUE(head);
    ASSERT_TRUE(movement);
    ASSERT_TRUE(music);
    ASSERT_TRUE(pose);
    EXPECT_DOUBLE_EQ(head->orientation.yaw, -10.0);
    EXPECT_DOUBLE_EQ(movement->movementRequest.duration_s, 1.752);
    EXPECT_EQ(music->song, "musicfox_hot_dogs_for_breakfast.mp3");
    EXPECT_DOUBLE_EQ(pose->pose.position.x, 0.025);
    EXPECT_DOUBLE_EQ(pose->pose.orientation.roll, -6.0);
    auto stop = request<RequestMusic>(dance->get().name, 17, 0);
    ASSERT_TRUE(stop);
    EXPECT_EQ(stop->song, "STOP");
}

TEST_F(BehaviorParserTest, ParsesScalarFormsAndDefaultsInLexicographicOrder) {
    ASSERT_TRUE(parser_.parseString(R"(
behaviors:
  - name: scalars
    actions:
      - RequestTalking: "Hallo: Welt # gesprochen"
        RequestMusic: STOP
        RequestListening: true
        RequestChat: Frage
      - RequestListening: false
)"));
    auto chat = request<RequestChat>("scalars", 0, 0);
    auto listening = request<RequestListening>("scalars", 0, 1);
    auto music = request<RequestMusic>("scalars", 0, 2);
    auto talking = request<RequestTalking>("scalars", 0, 3);
    ASSERT_TRUE(chat);
    ASSERT_TRUE(listening);
    ASSERT_TRUE(music);
    ASSERT_TRUE(talking);
    EXPECT_EQ(chat->text, "Frage");
    EXPECT_EQ(chat->language, "de");
    EXPECT_TRUE(listening->active);
    EXPECT_EQ(music->song, "STOP");
    EXPECT_FLOAT_EQ(music->volume, 0.8f);
    EXPECT_EQ(talking->text, "Hallo: Welt # gesprochen");
    EXPECT_EQ(talking->language, "de");
    EXPECT_DOUBLE_EQ(talking->minDuration, 0.0);
    listening = request<RequestListening>("scalars", 1, 0);
    ASSERT_TRUE(listening);
    EXPECT_FALSE(listening->active);
}

TEST_F(BehaviorParserTest, ParsesAllMappingForms) {
    ASSERT_TRUE(parser_.parseString(R"(
behaviors:
  - name: mappings
    actions:
      - RequestVelocity: {linear: {x: 0.1}, angular: {z: 0.2}, minDuration: 0.9}
        RequestTalking: {text: Hello, language: en, minDuration: 0.8}
        RequestSystem: {turnOffServoRelay: true, systemShutdown: true, minDuration: 0.7}
        RequestSinglePose: {position: {z: -0.05}, orientation: {pitch: 15}, minDuration: 0.6}
        RequestMusic: {song: dance.mp3, volume: 0.3, minDuration: 0.5}
        RequestMovementType: {type: SEQUENCE_LOOK, direction: ANTICLOCKWISE, duration_s: 3, minDuration: 0.4}
        RequestListening: {active: true, minDuration: 0.3}
        RequestHeadOrientation: {yaw: -12, minDuration: 0.2}
        RequestChat: {text: Question, language: en, minDuration: 0.1}
)"));
    auto chat = request<RequestChat>("mappings", 0, 0);
    auto head = request<RequestHeadOrientation>("mappings", 0, 1);
    auto listening = request<RequestListening>("mappings", 0, 2);
    auto movement = request<RequestMovementType>("mappings", 0, 3);
    auto music = request<RequestMusic>("mappings", 0, 4);
    auto pose = request<RequestSinglePose>("mappings", 0, 5);
    auto system = request<RequestSystem>("mappings", 0, 6);
    auto talking = request<RequestTalking>("mappings", 0, 7);
    auto velocity = request<RequestVelocity>("mappings", 0, 8);
    ASSERT_TRUE(chat);
    ASSERT_TRUE(head);
    ASSERT_TRUE(listening);
    ASSERT_TRUE(movement);
    ASSERT_TRUE(music);
    ASSERT_TRUE(pose);
    ASSERT_TRUE(system);
    ASSERT_TRUE(talking);
    ASSERT_TRUE(velocity);
    EXPECT_EQ(chat->text, "Question");
    EXPECT_EQ(chat->language, "en");
    EXPECT_DOUBLE_EQ(head->orientation.yaw, -12);
    EXPECT_DOUBLE_EQ(head->orientation.pitch, 0);
    EXPECT_TRUE(listening->active);
    EXPECT_EQ(movement->movementRequest.type, MovementRequest::SEQUENCE_LOOK);
    EXPECT_EQ(movement->movementRequest.direction, MovementRequest::ANTICLOCKWISE);
    EXPECT_DOUBLE_EQ(movement->movementRequest.duration_s, 3);
    EXPECT_EQ(music->song, "dance.mp3");
    EXPECT_FLOAT_EQ(music->volume, 0.3f);
    EXPECT_DOUBLE_EQ(pose->pose.position.z, -0.05);
    EXPECT_DOUBLE_EQ(pose->pose.position.x, 0);
    EXPECT_DOUBLE_EQ(pose->pose.orientation.pitch, 15);
    EXPECT_TRUE(system->turnOffServoRelay);
    EXPECT_TRUE(system->systemShutdown);
    EXPECT_EQ(talking->text, "Hello");
    EXPECT_EQ(talking->language, "en");
    EXPECT_DOUBLE_EQ(velocity->velocity.linear.x, 0.1);
    EXPECT_DOUBLE_EQ(velocity->velocity.angular.z, 0.2);
    EXPECT_DOUBLE_EQ(velocity->velocity.linear.y, 0);
    const auto& group = parser_.getBehavior("mappings")->get().actionGroups[0];
    for (size_t i = 0; i < group.size(); ++i) {
        EXPECT_NEAR(group[i]->minDuration, (i + 1) / 10.0, 1e-12);
    }
}

TEST_F(BehaviorParserTest, MappingDefaultsRemainUnchanged) {
    ASSERT_TRUE(parser_.parseString(R"(
behaviors:
  - name: defaults
    actions:
      - RequestChat: {}
        RequestListening: {}
        RequestMusic: {}
        RequestSystem: {}
        RequestTalking: {}
)"));
    auto chat = request<RequestChat>("defaults", 0, 0);
    auto listening = request<RequestListening>("defaults", 0, 1);
    auto music = request<RequestMusic>("defaults", 0, 2);
    auto system = request<RequestSystem>("defaults", 0, 3);
    auto talking = request<RequestTalking>("defaults", 0, 4);
    ASSERT_TRUE(chat);
    ASSERT_TRUE(listening);
    ASSERT_TRUE(music);
    ASSERT_TRUE(system);
    ASSERT_TRUE(talking);
    EXPECT_EQ(chat->language, "de");
    EXPECT_TRUE(chat->text.empty());
    EXPECT_FALSE(listening->active);
    EXPECT_FLOAT_EQ(music->volume, 0.8f);
    EXPECT_TRUE(music->song.empty());
    EXPECT_FALSE(system->turnOffServoRelay);
    EXPECT_FALSE(system->systemShutdown);
    EXPECT_EQ(talking->language, "de");
    EXPECT_TRUE(talking->text.empty());
}

TEST_F(BehaviorParserTest, RejectsMalformedYamlAndInvalidRoot) {
    for (const auto* text : {"behaviors: [", "", "[]", "behaviors: {}", "behaviors: []"}) {
        EXPECT_FALSE(parser_.parseString(text)) << text;
    }
    EXPECT_FALSE(parser_.parseFile("/nonexistent/rumblex/behaviors.yaml"));
}

TEST_F(BehaviorParserTest, InvalidExplicitNumberIsNotReplacedWithDefault) {
    EXPECT_FALSE(parser_.parseString(R"(
behaviors:
  - name: invalid
    actions:
      - RequestMusic: {song: dance.mp3, volume: loud}
)"));
}

TEST_F(BehaviorParserTest, RetainsPartialParsingForInvalidRequestsAndBehaviors) {
    ASSERT_TRUE(parser_.parseString(R"(
behaviors:
  - name: invalid
    actions: []
  - name: valid
    actions:
      - UnknownRequest: ignored
        RequestMovementType: {name: SINGLE_POSE, duration_s: invalid}
        RequestTalking: hello
)"));
    EXPECT_EQ(parser_.getBehaviors().size(), 1u);
    auto talking = request<RequestTalking>("valid", 0, 0);
    ASSERT_TRUE(talking);
    EXPECT_EQ(talking->text, "hello");
    EXPECT_EQ(parser_.getBehavior("valid")->get().actionGroups[0].size(), 1u);
}
}  // namespace brain
