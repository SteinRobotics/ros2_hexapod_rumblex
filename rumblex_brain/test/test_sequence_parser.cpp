#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "rumblex_brain/parser/sequence_parser.hpp"

namespace brain {

namespace {

using namespace mp_units;
using namespace mp_units::si::unit_symbols;
class SequenceParserTest : public ::testing::Test {
   protected:
    static void SetUpTestSuite() {
        rclcpp::init(0, nullptr);
    }
    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }

    rclcpp::Node::SharedPtr node_ = std::make_shared<rclcpp::Node>("test_sequence_parser");
    CSequenceParser parser_{node_};

    template <typename T>
    std::shared_ptr<T> request(const std::string& name, size_t group, size_t index) {
        const auto sequence = parser_.getSequence(name);
        if (!sequence || group >= sequence->get().actionGroups.size() ||
            index >= sequence->get().actionGroups[group].size()) {
            ADD_FAILURE() << "Missing request in " << name;
            return nullptr;
        }
        return std::dynamic_pointer_cast<T>(sequence->get().actionGroups[group][index]);
    }
};

class SequenceFilesTest : public SequenceParserTest {
   protected:
    void SetUp() override {
        char pattern[] = "/tmp/rumblex-sequences-XXXXXX";
        const auto directory = mkdtemp(pattern);
        ASSERT_NE(directory, nullptr);
        directory_ = directory;
    }
    void TearDown() override {
        if (!directory_.empty()) std::filesystem::remove_all(directory_);
    }
    void write(const std::string& name, const std::string& yaml) {
        const auto path = directory_ / name;
        std::filesystem::create_directories(path.parent_path());
        std::ofstream(path) << yaml;
    }
    std::filesystem::path directory_;
};

TEST_F(SequenceFilesTest, LoadsAllFilesAndNestedSnippetsInOrder) {
    write("sequences/b.yml", "name: second\nactions: [{RequestTalking: second}]\n");
    write("sequences/a.yaml", R"(
name: first
trigger: {voice: firstCommand}
actions:
  - RequestTalking: before
  - include: ../snippets/outer.yaml
  - include: ../snippets/inner.yaml
  - RequestTalking: after
)");
    write("snippets/outer.yaml", "actions:\n  - include: inner.yaml\n");
    write("snippets/inner.yaml", "actions: [{RequestTalking: inside, RequestMusic: STOP}]\n");
    write("sequences/ignored.txt", "not yaml");
    ASSERT_TRUE(parser_.parseDirectory((directory_ / "sequences").string()));
    ASSERT_EQ(parser_.getSequences().size(), 2u);
    EXPECT_EQ(parser_.getSequences()[0].name, "first");
    EXPECT_EQ(parser_.getSequences()[1].name, "second");
    ASSERT_TRUE(parser_.getSequenceForVoiceRequest("firstCommand"));
    ASSERT_EQ(parser_.getSequences()[0].actionGroups.size(), 4u);
    auto inside = request<RequestTalking>("first", 1, 1);
    auto repeated = request<RequestTalking>("first", 2, 1);
    ASSERT_TRUE(inside);
    ASSERT_TRUE(repeated);
    EXPECT_EQ(inside->text, "inside");
    EXPECT_NE(inside, repeated);
    ASSERT_TRUE(parser_.parseFile((directory_ / "sequences/b.yml").string()));
    EXPECT_FALSE(parser_.getSequenceForVoiceRequest("firstCommand"));
}

TEST_F(SequenceFilesTest, RejectsBrokenIncludesWithoutReplacingCatalog) {
    ASSERT_TRUE(parser_.parseDirectory(RUMBLEX_SEQUENCES_DIRECTORY));
    write("sequence.yaml", "name: broken\nactions: [{include: snippet.yaml}]\n");
    const auto file = (directory_ / "sequence.yaml").string();
    EXPECT_FALSE(parser_.parseFile(file));
    for (const auto* snippet :
         {"actions: [{include: snippet.yaml}]", "actions: [{include: sequence.yaml}]", "actions: {}",
          "actions: [", "name: invalid", "actions: [{include: [], RequestTalking: mixed}]"}) {
        write("snippet.yaml", snippet);
        EXPECT_FALSE(parser_.parseFile(file)) << snippet;
        EXPECT_EQ(parser_.getSequences().size(), 17u);
        EXPECT_TRUE(parser_.getSequenceForVoiceRequest("commandStandup"));
    }
}

TEST_F(SequenceFilesTest, RejectsDuplicateNamesAndInvalidDirectoryEntries) {
    write("a.yaml", "name: duplicate\nactions: [{RequestTalking: hello}]\n");
    write("b.yaml", "name: duplicate\nactions: [{RequestTalking: other}]\n");
    EXPECT_FALSE(parser_.parseDirectory(directory_.string()));
    EXPECT_TRUE(parser_.getSequences().empty());
    write("b.yaml", "name: invalid\nactions: []\n");
    EXPECT_FALSE(parser_.parseDirectory(directory_.string()));
    EXPECT_TRUE(parser_.getSequences().empty());
    EXPECT_FALSE(parser_.parseDirectory((directory_ / "missing").string()));
    std::filesystem::create_directory(directory_ / "empty");
    EXPECT_FALSE(parser_.parseDirectory((directory_ / "empty").string()));
}

TEST_F(SequenceParserTest, RejectsIncludesWithoutSourceFile) {
    EXPECT_FALSE(parser_.parseString("sequences: [{name: invalid, actions: [{include: snippet.yaml}]}]"));
}

TEST_F(SequenceParserTest, LoadsCatalogAndResolvesTriggers) {
    ASSERT_TRUE(parser_.parseDirectory(RUMBLEX_SEQUENCES_DIRECTORY));
    EXPECT_EQ(parser_.getSequences().size(), 17u);
    EXPECT_FALSE(parser_.getSequence("missing"));
    EXPECT_FALSE(parser_.getSequenceForVoiceRequest("missing"));
    rumblex_interfaces::msg::JoystickRequest joystick;
    EXPECT_FALSE(parser_.getSequenceForJoystickRequest(joystick));
    joystick.button_a = true;
    const auto watch = parser_.getSequenceForJoystickRequest(joystick);
    ASSERT_TRUE(watch);
    EXPECT_EQ(watch->get().name, "normal_watch");
    EXPECT_EQ(watch->get().actionGroups.size(), 2u);
    const auto standup = parser_.getSequenceForVoiceRequest("commandStandup");
    ASSERT_TRUE(standup);
    EXPECT_EQ(standup->get().name, "standup");
    auto movement = request<RequestMovementType>("standup", 0, 0);
    ASSERT_TRUE(movement);
    EXPECT_EQ(movement->movementRequest.type, MovementRequest::SEQUENCE_STAND_UP);
    EXPECT_DOUBLE_EQ(movement->movementRequest.duration_s.numerical_value_in(s), 1.5);
    auto music = request<RequestMusic>("standup", 0, 1);
    ASSERT_TRUE(music);
    EXPECT_EQ(music->song, "STOP");
}

TEST_F(SequenceParserTest, PreservesDanceGroupsAndPoseUnits) {
    ASSERT_TRUE(parser_.parseDirectory(RUMBLEX_SEQUENCES_DIRECTORY));
    const auto dance = parser_.getSequence("dance_hot_dogs_for_breakfast");
    ASSERT_TRUE(dance);
    EXPECT_EQ(dance->get().actionGroups.size(), 18u);
    auto movement = request<RequestMovementType>(dance->get().name, 1, 0);
    auto music = request<RequestMusic>(dance->get().name, 1, 1);
    ASSERT_TRUE(movement);
    ASSERT_TRUE(music);
    const auto& pose = movement->movementRequest.body_pose;
    EXPECT_TRUE(pose.has_head_orientation);
    EXPECT_TRUE(pose.has_torso_pose);
    EXPECT_FALSE(pose.has_toe_positions);
    EXPECT_DOUBLE_EQ(pose.head_orientation.yaw, -10.0);
    EXPECT_DOUBLE_EQ(movement->movementRequest.duration_s.numerical_value_in(s), 1.752);
    EXPECT_EQ(music->song, "musicfox_hot_dogs_for_breakfast.mp3");
    EXPECT_DOUBLE_EQ(pose.torso_pose.position.x, 0.015);
    EXPECT_DOUBLE_EQ(pose.torso_pose.orientation.roll, -3.6);
    auto stop = request<RequestMusic>(dance->get().name, 17, 0);
    ASSERT_TRUE(stop);
    EXPECT_EQ(stop->song, "STOP");
}

TEST_F(SequenceParserTest, ParsesScalarFormsAndDefaultsInLexicographicOrder) {
    ASSERT_TRUE(parser_.parseString(R"(
sequences:
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
    EXPECT_DOUBLE_EQ(talking->minDuration.numerical_value_in(s), 0.0);
    listening = request<RequestListening>("scalars", 1, 0);
    ASSERT_TRUE(listening);
    EXPECT_FALSE(listening->active);
}

TEST_F(SequenceParserTest, ParsesAllMappingForms) {
    ASSERT_TRUE(parser_.parseString(R"(
sequences:
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
    auto listening = request<RequestListening>("mappings", 0, 1);
    auto movement = request<RequestMovementType>("mappings", 0, 2);
    auto music = request<RequestMusic>("mappings", 0, 3);
    auto system = request<RequestSystem>("mappings", 0, 4);
    auto talking = request<RequestTalking>("mappings", 0, 5);
    auto velocity = request<RequestVelocity>("mappings", 0, 6);
    ASSERT_TRUE(chat);
    ASSERT_TRUE(listening);
    ASSERT_TRUE(movement);
    ASSERT_TRUE(music);
    ASSERT_TRUE(system);
    ASSERT_TRUE(talking);
    ASSERT_TRUE(velocity);
    EXPECT_EQ(chat->text, "Question");
    EXPECT_EQ(chat->language, "en");
    const auto& pose = movement->movementRequest.body_pose;
    EXPECT_DOUBLE_EQ(pose.head_orientation.yaw, -12);
    EXPECT_DOUBLE_EQ(pose.head_orientation.pitch, 0);
    EXPECT_TRUE(listening->active);
    EXPECT_EQ(movement->movementRequest.type, MovementRequest::SEQUENCE_LOOK);
    EXPECT_EQ(movement->movementRequest.direction, MovementRequest::ANTICLOCKWISE);
    EXPECT_DOUBLE_EQ(movement->movementRequest.duration_s.numerical_value_in(s), 3);
    EXPECT_EQ(music->song, "dance.mp3");
    EXPECT_FLOAT_EQ(music->volume, 0.3f);
    EXPECT_DOUBLE_EQ(pose.torso_pose.position.z, -0.05);
    EXPECT_DOUBLE_EQ(pose.torso_pose.position.x, 0);
    EXPECT_DOUBLE_EQ(pose.torso_pose.orientation.pitch, 15);
    EXPECT_TRUE(system->turnOffServoRelay);
    EXPECT_TRUE(system->systemShutdown);
    EXPECT_EQ(talking->text, "Hello");
    EXPECT_EQ(talking->language, "en");
    EXPECT_DOUBLE_EQ(velocity->velocity.linear.x, 0.1);
    EXPECT_DOUBLE_EQ(velocity->velocity.angular.z, 0.2);
    EXPECT_DOUBLE_EQ(velocity->velocity.linear.y, 0);
    EXPECT_DOUBLE_EQ(movement->minDuration.numerical_value_in(s), 0.4);
}

TEST_F(SequenceParserTest, MappingDefaultsRemainUnchanged) {
    ASSERT_TRUE(parser_.parseString(R"(
sequences:
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

TEST_F(SequenceParserTest, RejectsMalformedYamlAndInvalidRoot) {
    for (const auto* text : {"sequences: [", "", "[]", "sequences: {}", "sequences: []"}) {
        EXPECT_FALSE(parser_.parseString(text)) << text;
    }
    EXPECT_FALSE(parser_.parseFile("/nonexistent/rumblex/sequences.yaml"));
}

TEST_F(SequenceParserTest, InvalidExplicitNumberIsNotReplacedWithDefault) {
    EXPECT_FALSE(parser_.parseString(R"(
sequences:
  - name: invalid
    actions:
      - RequestMusic: {song: dance.mp3, volume: loud}
)"));
}

TEST_F(SequenceParserTest, InvalidActionsRetainPreviousCatalog) {
    ASSERT_TRUE(parser_.parseDirectory(RUMBLEX_SEQUENCES_DIRECTORY));
    for (const auto* text :
         {"name: invalid\nactions: [{UnknownRequest: ignored, say: hello}]",
          "sequences: [{name: valid, actions: [{say: hi}]}, {name: invalid, actions: []}]",
          "name: invalid\nactions: [{movement: {name: SINGLE_POSE, duration_s: invalid}, say: hello}]"}) {
        EXPECT_FALSE(parser_.parseString(text));
        EXPECT_EQ(parser_.getSequences().size(), 17u);
    }
}
TEST_F(SequenceParserTest, BodyPoseUsesOneRequestAndPreservesComponentPresence) {
    ASSERT_TRUE(parser_.parseString(R"(
name: poses
actions:
  - body_pose:
      duration_s: 1.5
      head_orientation: {yaw: 12}
  - body_pose:
      torso_pose: {}
  - movement:
      name: SINGLE_POSE
      duration_s: 2
      body_pose:
        toe_positions:
          - {x: 0.2, y: -0.16, z: -0.05}
          - {x: 0, y: -0.22, z: -0.05}
          - {x: -0.2, y: -0.16, z: -0.05}
          - {x: 0.2, y: 0.16, z: -0.05}
          - {x: 0, y: 0.22, z: -0.05}
          - {x: -0.2, y: 0.16, z: -0.05}
)"));
    const auto head = request<RequestMovementType>("poses", 0, 0);
    const auto torso = request<RequestMovementType>("poses", 1, 0);
    const auto toes = request<RequestMovementType>("poses", 2, 0);
    ASSERT_TRUE(head);
    ASSERT_TRUE(torso);
    ASSERT_TRUE(toes);
    EXPECT_EQ(parser_.getSequence("poses")->get().actionGroups[0].size(), 1u);
    EXPECT_EQ(head->movementRequest.type, MovementRequest::SINGLE_POSE);
    EXPECT_TRUE(head->movementRequest.body_pose.has_head_orientation);
    EXPECT_FALSE(head->movementRequest.body_pose.has_torso_pose);
    EXPECT_FALSE(head->movementRequest.body_pose.has_toe_positions);
    EXPECT_DOUBLE_EQ(head->movementRequest.body_pose.head_orientation.yaw, 12);
    EXPECT_TRUE(torso->movementRequest.body_pose.has_torso_pose);
    EXPECT_FALSE(torso->movementRequest.body_pose.has_head_orientation);
    EXPECT_EQ(torso->movementRequest.body_pose.torso_pose, rumblex_interfaces::msg::Pose());
    EXPECT_TRUE(toes->movementRequest.body_pose.has_toe_positions);
    EXPECT_DOUBLE_EQ(toes->movementRequest.body_pose.toe_positions[5].x, -0.2);
}
TEST_F(SequenceParserTest, RejectsMalformedPoseAndConflictingActions) {
    for (const auto* action :
         {"body_pose: {}", "body_pose: {head_orientation: null}", "body_pose: {head_orientation: {yaaw: 12}}",
          "body_pose: {head_orientation: {yaw: .nan}}", "body_pose: {torso_pose: {position: {x: .inf}}}",
          "body_pose: {head_orientation: {}, duration_s: 0.5}",
          "body_pose: {head_orientation: {}, duration_s: .inf}", "body_pose: {toe_positions: []}",
          "body_pose: {head_orientation: {}, head_orientation: {yaw: 3}}",
          "body_pose: {head_orientation: {}}, movement: {name: SINGLE_POSE}",
          "body_pose: {head_orientation: {}}, RequestHeadOrientation: {yaw: 4}",
          "say: one, RequestTalking: two", "movement: {name: UNKNOWN}", "movement: {name: SINGLE_POSE}"}) {
        EXPECT_FALSE(parser_.parseString(std::string("name: invalid\nactions: [{") + action + "}]"))
            << action;
    }
}
TEST_F(SequenceFilesTest, LoopsIncludeSnippetsInOrderWithIndependentRequests) {
    write("sequences/test.yaml", R"(
name: repeated
actions:
  - say: before
  - loop:
      times: 3
      actions:
        - include: ../snippets/outer.yaml
  - say: after
)");
    write("snippets/outer.yaml", "actions: [{include: inner.yaml}]\n");
    write("snippets/inner.yaml", R"(
actions:
  - body_pose: {head_orientation: {yaw: -12}}
  - body_pose: {head_orientation: {yaw: 12}}
)");
    ASSERT_TRUE(parser_.parseFile((directory_ / "sequences/test.yaml").string()));
    ASSERT_EQ(parser_.getSequence("repeated")->get().actionGroups.size(), 8u);
    for (size_t i = 1; i <= 6; ++i) {
        auto pose = request<RequestMovementType>("repeated", i, 0);
        ASSERT_TRUE(pose);
        EXPECT_DOUBLE_EQ(pose->movementRequest.body_pose.head_orientation.yaw, i % 2 ? -12 : 12);
        if (i > 2) {
            EXPECT_NE(pose, request<RequestMovementType>("repeated", i - 2, 0));
        }
    }
    EXPECT_EQ(request<RequestTalking>("repeated", 7, 0)->text, "after");
}
TEST_F(SequenceParserTest, NestedLoopsWorkWithoutAFileAndDoNotShareRequests) {
    ASSERT_TRUE(parser_.parseString(R"(
name: nested
actions:
  - loop:
      times: 2
      actions:
        - loop:
            times: 3
            actions: [{body_pose: {torso_pose: {}}}]
)"));
    ASSERT_EQ(parser_.getSequence("nested")->get().actionGroups.size(), 6u);
    auto first = request<RequestMovementType>("nested", 0, 0);
    first->movementRequest.body_pose.torso_pose.position.x = 1;
    EXPECT_DOUBLE_EQ(
        request<RequestMovementType>("nested", 1, 0)->movementRequest.body_pose.torso_pose.position.x, 0);
}
TEST_F(SequenceParserTest, RejectsInvalidLoopsAndExcessiveExpansionAtomically) {
    ASSERT_TRUE(parser_.parseString("name: kept\nactions: [{say: hello}]"));
    for (const auto* action :
         {"loop: {times: 0, actions: [{say: hi}]}", "loop: {times: -1, actions: [{say: hi}]}",
          "loop: {times: 1.5, actions: [{say: hi}]}", "loop: {times: true, actions: [{say: hi}]}",
          "loop: {times: 1001, actions: [{say: hi}]}", "loop: {actions: [{say: hi}]}",
          "loop: {times: 2, actions: []}", "loop: {times: 2, actions: {}}, say: invalid",
          "loop: {times: 1000, actions: [{loop: {times: 11, actions: [{say: hi}]}}]}"}) {
        EXPECT_FALSE(parser_.parseString(std::string("name: invalid\nactions: [{") + action + "}]"))
            << action;
        ASSERT_TRUE(parser_.getSequence("kept"));
    }
    std::string nested = "{say: hello}";
    for (int i = 0; i < 17; ++i) nested = "{loop: {times: 1, actions: [" + nested + "]}}";
    EXPECT_FALSE(parser_.parseString("name: invalid\nactions: [" + nested + "]"));
    ASSERT_TRUE(parser_.getSequence("kept"));
}
TEST_F(SequenceParserTest, LegacyWrapperAndRequestsProduceTheSameMovement) {
    ASSERT_TRUE(parser_.parseString(R"(
behaviors:
  - name: legacy
    actions:
      - RequestMovementType: {name: SINGLE_POSE, duration_s: 1.5}
        RequestSinglePose: {orientation: {roll: 8}}
        RequestHeadOrientation: {yaw: -12}
)"));
    const auto old = request<RequestMovementType>("legacy", 0, 0)->movementRequest;
    ASSERT_TRUE(parser_.parseString(R"(
name: modern
actions:
  - body_pose:
      duration_s: 1.5
      torso_pose: {orientation: {roll: 8}}
      head_orientation: {yaw: -12}
)"));
    const auto modern = request<RequestMovementType>("modern", 0, 0)->movementRequest;
    EXPECT_EQ(old.body_pose, modern.body_pose);
    EXPECT_EQ(old.type, modern.type);
    EXPECT_EQ(old.duration_s, modern.duration_s);
}
TEST_F(SequenceFilesTest, CatalogExpansionBudgetIsSharedAcrossFiles) {
    const std::string actions = R"(
actions:
  - loop:
      times: 1000
      actions:
        - loop:
            times: 6
            actions: [{say: hello}]
)";
    write("a.yaml", "name: first\n" + actions);
    write("b.yaml", "name: second\n" + actions);
    ASSERT_TRUE(parser_.parseString("name: kept\nactions: [{say: hello}]"));
    EXPECT_FALSE(parser_.parseDirectory(directory_.string()));
    EXPECT_TRUE(parser_.getSequence("kept"));
}
TEST_F(SequenceFilesTest, IncludeCycleInsideLoopIsRejected) {
    write("a.yaml", "name: cyclic\nactions: [{loop: {times: 2, actions: [{include: b.yaml}]}}]");
    write("b.yaml", "actions: [{loop: {times: 2, actions: [{include: b.yaml}]}}]");
    EXPECT_FALSE(parser_.parseFile((directory_ / "a.yaml").string()));
}
}  // namespace
}  // namespace brain
