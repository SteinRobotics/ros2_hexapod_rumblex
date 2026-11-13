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

TEST_F(SequenceParserTest, GeneratedSectionRehearsals) {
    const char* directory = std::getenv("RUMBLEX_REHEARSAL_DIRECTORY");
    if (!directory) GTEST_SKIP() << "Run via test_section_parser for generated temporary fixtures";
    ASSERT_TRUE(parser_.parseDirectory(directory));
    ASSERT_EQ(parser_.getSequences().size(), 8u);
    for (const auto* name :
         {"intro", "verse", "guitar_solo", "body_roll", "drum_fill", "breakdown", "encore", "finale"}) {
        const std::string sequence_name = std::string("dance_rock_") + name;
        const auto sequence = parser_.getSequence(sequence_name);
        ASSERT_TRUE(sequence);
        const auto count = sequence->get().actionGroups.size();
        ASSERT_GT(count, 2u);
        const auto first = request<RequestMovementType>(sequence_name, 0, 0);
        const auto last = request<RequestMovementType>(sequence_name, count - 1, 0);
        ASSERT_TRUE(first);
        ASSERT_TRUE(last);
        EXPECT_TRUE(first->movementRequest.save_pose);
        EXPECT_TRUE(last->movementRequest.restore_pose);
    }
}

TEST_F(SequenceParserTest, EpicDancePlaysThirtySecondOpeningAndStops) {
    ASSERT_TRUE(parser_.parseFile(std::string(RUMBLEX_SEQUENCES_DIRECTORY) + "/dance_epic.yaml"));
    const auto sequence = parser_.getSequence("dance_epic");
    ASSERT_TRUE(sequence);
    const auto count = sequence->get().actionGroups.size();
    ASSERT_GT(count, 2u);
    const auto music = request<RequestMusic>("dance_epic", 0, 1);
    ASSERT_TRUE(music);
    EXPECT_EQ(music->song, "lovemusic_ai__Iron_Star_March.mp3");
    EXPECT_TRUE(music->background);
    double duration = 0;
    for (size_t i = 0; i + 1 < count; ++i) {
        const auto movement = request<RequestMovementType>("dance_epic", i, 0);
        ASSERT_TRUE(movement);
        if (sequence->get().actionGroups[i].size() == 2 && i != 0) {
            const auto fade = request<RequestMusic>("dance_epic", i, 1);
            ASSERT_TRUE(fade);
            EXPECT_EQ(fade->song, "STOP");
            EXPECT_FLOAT_EQ(fade->fade_out_s, 4.0f);
            EXPECT_NEAR(duration, 26.0, 1e-6);
        }
        duration += movement->movementRequest.duration_s.numerical_value_in(s);
        if (i == 0) {
            EXPECT_TRUE(movement->movementRequest.save_pose);
        }
        if (i + 2 == count) {
            EXPECT_TRUE(movement->movementRequest.restore_pose);
        }
    }
    EXPECT_NEAR(duration, 30.0, 1e-9);
    const auto stop = request<RequestMusic>("dance_epic", count - 1, 0);
    ASSERT_TRUE(stop);
    EXPECT_EQ(stop->song, "STOP");
}

TEST_F(SequenceParserTest, GeneratedMetalDanceRestoresEntrancePose) {
    ASSERT_TRUE(parser_.parseFile(std::string(RUMBLEX_SEQUENCES_DIRECTORY) + "/dance_metal.yaml"));
    const auto sequence = parser_.getSequence("dance_metal");
    ASSERT_TRUE(sequence);
    ASSERT_EQ(sequence->get().actionGroups.size(), 122u);
    const auto first = request<RequestMovementType>("dance_metal", 0, 0);
    const auto last = request<RequestMovementType>("dance_metal", 121, 0);
    ASSERT_TRUE(first);
    ASSERT_TRUE(last);
    EXPECT_TRUE(first->movementRequest.save_pose);
    EXPECT_TRUE(last->movementRequest.restore_pose);
    double duration = 0;
    for (size_t i = 0; i < 122; ++i) {
        const auto movement = request<RequestMovementType>("dance_metal", i, 0);
        ASSERT_TRUE(movement);
        duration += movement->movementRequest.duration_s.numerical_value_in(s);
    }
    EXPECT_DOUBLE_EQ(duration, 50.5);
}

TEST_F(SequenceParserTest, GeneratedRockDanceRestoresEntrancePose) {
    ASSERT_TRUE(parser_.parseFile(std::string(RUMBLEX_SEQUENCES_DIRECTORY) + "/dance_rock.yaml"));
    const auto sequence = parser_.getSequence("dance_rock");
    ASSERT_TRUE(sequence);
    ASSERT_EQ(sequence->get().actionGroups.size(), 117u);
    const auto first = request<RequestMovementType>("dance_rock", 0, 0);
    const auto last = request<RequestMovementType>("dance_rock", 115, 0);
    ASSERT_TRUE(first);
    ASSERT_TRUE(last);
    EXPECT_TRUE(first->movementRequest.save_pose);
    EXPECT_TRUE(last->movementRequest.restore_pose);
    EXPECT_DOUBLE_EQ(last->movementRequest.duration_s.numerical_value_in(s), 2.0);
    const auto music = request<RequestMusic>("dance_rock", 0, 1);
    ASSERT_TRUE(music);
    EXPECT_EQ(music->song, "musicfox_excess_party.mp3");
    EXPECT_TRUE(music->background);
    const auto fade = request<RequestMusic>("dance_rock", 112, 1);
    ASSERT_TRUE(fade);
    EXPECT_EQ(fade->song, "STOP");
    double remaining_duration = 0;
    for (size_t i = 112; i <= 115; ++i) {
        const auto movement = request<RequestMovementType>("dance_rock", i, 0);
        ASSERT_TRUE(movement);
        remaining_duration += movement->movementRequest.duration_s.numerical_value_in(s);
    }
    EXPECT_NEAR(fade->fade_out_s, remaining_duration, 1e-6);
    const auto stop = request<RequestMusic>("dance_rock", 116, 0);
    ASSERT_TRUE(stop);
    EXPECT_EQ(stop->song, "STOP");
    EXPECT_FLOAT_EQ(stop->fade_out_s, 0.0f);
}

TEST_F(SequenceParserTest, GeneratedCuriousLookPreservesPhraseOrder) {
    ASSERT_TRUE(parser_.parseFile(std::string(RUMBLEX_SEQUENCES_DIRECTORY) + "/curious_look.yaml"));
    const auto sequence = parser_.getSequence("curious_look");
    ASSERT_TRUE(sequence);
    ASSERT_EQ(sequence->get().actionGroups.size(), 24u);
    for (size_t phrase = 0; phrase < 4; ++phrase) {
        const auto first = request<RequestMovementType>("curious_look", phrase * 6, 0);
        const auto turn = request<RequestMovementType>("curious_look", phrase * 6 + 1, 0);
        const auto last = request<RequestMovementType>("curious_look", phrase * 6 + 5, 0);
        ASSERT_TRUE(first);
        ASSERT_TRUE(turn);
        ASSERT_TRUE(last);
        EXPECT_TRUE(first->movementRequest.save_pose);
        EXPECT_TRUE(turn->movementRequest.pose_delta.has_value());
        EXPECT_DOUBLE_EQ(turn->movementRequest.duration_s.numerical_value_in(s), 1.2);
        EXPECT_TRUE(last->movementRequest.restore_pose);
        EXPECT_EQ(last->movementRequest.interpolation, InterpolationType::SinSquared);
    }
}

TEST_F(SequenceParserTest, ReusesYamlConstantsInsideLoops) {
    ASSERT_TRUE(parser_.parseString(R"(
name: constants
variables:
  kMovement: &kMovement SINGLE_POSE
  kDuration: &kDuration 0.3
  kInterpolation: &kInterpolation sin_squared
  kCount: &kCount 2
actions:
  - loop:
      times: *kCount
      actions:
        - movement:
            name: *kMovement
            duration_s: *kDuration
            interpolation: *kInterpolation
            body_pose: {head_orientation: {yaw: 10}}
)"));
    ASSERT_EQ(parser_.getSequence("constants")->get().actionGroups.size(), 2u);
    for (size_t i = 0; i < 2; ++i) {
        const auto parsed = request<RequestMovementType>("constants", i, 0);
        ASSERT_TRUE(parsed);
        EXPECT_DOUBLE_EQ(parsed->movementRequest.duration_s.numerical_value_in(s), 0.3);
        EXPECT_EQ(parsed->movementRequest.interpolation, InterpolationType::SinSquared);
    }
}

TEST_F(SequenceParserTest, ValidatesVariablesWithoutAllowingUnknownSequenceFields) {
    for (const auto* value : {"null", "[]", "0.02", "constant"}) {
        EXPECT_FALSE(parser_.parseString(std::string("name: invalid\nvariables: ") + value +
                                         "\nactions: [{say: hello}]"));
    }
    EXPECT_TRUE(parser_.parseString("name: empty\nvariables: {}\nactions: [{say: hello}]"));
    EXPECT_FALSE(parser_.parseString("name: invalid\nconstants: {}\nactions: [{say: hello}]"));
    EXPECT_FALSE(parser_.parseString("name: invalid\nactions: [{say: *kUndefined}]"));
}

TEST_F(SequenceParserTest, InterpolationDefaultsAndExplicitCurves) {
    const std::vector<std::pair<std::string, InterpolationType>> cases = {
        {"", InterpolationType::Linear},
        {"linear", InterpolationType::Linear},
        {"sin_squared", InterpolationType::SinSquared},
        {"sin", InterpolationType::Sin},
        {"cos", InterpolationType::Cos}};
    for (const auto& [name, expected] : cases) {
        for (bool shorthand : {true, false}) {
            const std::string curve = name.empty() ? "" : ", interpolation: " + name;
            const std::string action =
                shorthand
                    ? "body_pose: {head_orientation: {yaw: 10}" + curve + "}"
                    : "movement: {name: SINGLE_POSE, body_pose: {head_orientation: {yaw: 10}}" + curve + "}";
            ASSERT_TRUE(parser_.parseString("name: curve\nactions: [{" + action + "}]"));
            const auto parsed = request<RequestMovementType>("curve", 0, 0);
            ASSERT_TRUE(parsed);
            EXPECT_EQ(parsed->movementRequest.interpolation, expected);
        }
    }
    for (const std::string invalid : {"bogus", "null", "[]", "{}", "1", "SIN"}) {
        for (const std::string prefix :
             {"body_pose: {head_orientation: {yaw: 10}, ",
              "movement: {name: SINGLE_POSE, body_pose: {head_orientation: {yaw: 10}}, "}) {
            EXPECT_FALSE(parser_.parseString("name: invalid\nactions: [{" + prefix +
                                             "interpolation: " + invalid + "}}]"));
            EXPECT_TRUE(parser_.getSequence("curve"));
        }
    }
    for (const std::string action :
         {"movement: {name: CONTINUOUS_POSE, interpolation: linear}",
          "movement: {name: MOVE_TRIPOD, interpolation: sin}",
          "movement: {name: SINGLE_POSE, body_pose: {head_orientation: {}, interpolation: cos}}",
          "body_pose: {head_orientation: {}, interpolation: sin, interpolation: cos}"}) {
        EXPECT_FALSE(parser_.parseString("name: invalid\nactions: [{" + action + "}]"));
        EXPECT_TRUE(parser_.getSequence("curve"));
    }
}

TEST_F(SequenceParserTest, RotationDeltasDefaultOmittedAxesToZeroAndRejectInvalidValues) {
    ASSERT_TRUE(parser_.parseString(R"(
name: rotate
actions:
  - movement:
      name: SINGLE_POSE
      pose_delta:
        head_rotation: {yaw: 25}
        torso_rotation: {pitch: -2}
)"));
    const auto action = request<RequestMovementType>("rotate", 0, 0);
    ASSERT_TRUE(action);
    ASSERT_TRUE(action->movementRequest.pose_delta);
    const auto& delta = *action->movementRequest.pose_delta;
    EXPECT_DOUBLE_EQ(delta.head_rotation.yaw, 25);
    EXPECT_DOUBLE_EQ(delta.head_rotation.roll, 0);
    EXPECT_DOUBLE_EQ(delta.head_rotation.pitch, 0);
    EXPECT_DOUBLE_EQ(delta.torso_rotation.pitch, -2);
    EXPECT_DOUBLE_EQ(delta.torso_rotation.roll, 0);
    EXPECT_DOUBLE_EQ(delta.torso_rotation.yaw, 0);
    for (const auto* field :
         {"head_rotation: {yaw: .nan}", "torso_rotation: {roll: .inf}", "head_rotation: {angle: 1}",
          "torso_rotation: {yaw: 1, yaw: 2}", "head_rotation: []"}) {
        SCOPED_TRACE(field);
        EXPECT_FALSE(parser_.parseString(
            std::string(
                "name: invalid\nactions:\n  - movement:\n      name: SINGLE_POSE\n      pose_delta: {") +
            field + "}"));
    }
    EXPECT_FALSE(parser_.parseString("name: removed\nactions: [{movement: {name: SEQUENCE_LOOK}}]"));
}

TEST_F(SequenceParserTest, NamedLegsResolveByNameAndLeaveOmittedLegsUnchanged) {
    ASSERT_TRUE(parser_.parseString(R"(
name: named
actions:
  - movement:
      name: SINGLE_POSE
      joint_targets:
        left_back: {torso_coxa: 1, coxa_femur: 2, femur_tibia: 3}
        right_front: {torso_coxa: -20, coxa_femur: 50, femur_tibia: 60}
  - movement:
      name: SINGLE_POSE
      pose_delta:
        toe_translation: {left_mid: {z: 0.02}, right_back: {x: 0.03}}
        coxa_rotation_deg: {left_front: 30, right_front: -30}
)"));
    const auto joints = request<RequestMovementType>("named", 0, 0);
    ASSERT_TRUE(joints);
    ASSERT_TRUE(joints->movementRequest.joint_targets[0]);
    ASSERT_TRUE(joints->movementRequest.joint_targets[5]);
    EXPECT_EQ(*joints->movementRequest.joint_targets[0], (std::array<double, 3>{-20, 50, 60}));
    EXPECT_EQ(*joints->movementRequest.joint_targets[5], (std::array<double, 3>{1, 2, 3}));
    for (size_t i = 1; i < 5; ++i) EXPECT_FALSE(joints->movementRequest.joint_targets[i]);
    const auto movement = request<RequestMovementType>("named", 1, 0);
    ASSERT_TRUE(movement);
    ASSERT_TRUE(movement->movementRequest.pose_delta);
    const auto& delta = *movement->movementRequest.pose_delta;
    EXPECT_DOUBLE_EQ(delta.toe_translation[4].z, 0.02);
    EXPECT_DOUBLE_EQ(delta.toe_translation[2].x, 0.03);
    EXPECT_DOUBLE_EQ(delta.coxa_rotation_deg[0], -30);
    EXPECT_DOUBLE_EQ(delta.coxa_rotation_deg[3], 30);
    EXPECT_EQ(delta.toe_translation[0], geometry_msgs::msg::Vector3());
    EXPECT_DOUBLE_EQ(delta.coxa_rotation_deg[5], 0);
}

TEST_F(SequenceParserTest, RejectsUnknownDuplicateAndMalformedNamedLegs) {
    for (const auto* field : {"joint_targets: {front: {torso_coxa: 0, coxa_femur: 0, femur_tibia: 0}}",
                              "joint_targets: {right_front: null}", "joint_targets: {}",
                              "joint_targets: {right_front: {torso_coxa: 0}}",
                              "pose_delta: {toe_translation: {right_back: {}, right_back: {z: 1}}}",
                              "pose_delta: {toe_translation: {left_middle: {z: 1}}}",
                              "pose_delta: {toe_translation: {right_front: {z: .nan}}}",
                              "pose_delta: {coxa_rotation_deg: {right_front: .inf}}",
                              "pose_delta: {coxa_rotation_deg: {left_front: 1, left_front: 2}}",
                              "pose_delta: {toe_translation: {}}",
                              "body_pose: {toe_positions: {right_front: {x: 0, y: 0, z: 0}}}"}) {
        SCOPED_TRACE(field);
        EXPECT_FALSE(parser_.parseString(
            std::string("name: invalid\nactions:\n  - movement:\n      name: SINGLE_POSE\n      ") + field));
    }
}

TEST_F(SequenceParserTest, RejectsInvalidJointTargetsAndPoseRestoration) {
    for (const auto* movement :
         {"{name: SEQUENCE_HIGH_FIVE}", "{name: SINGLE_POSE, joint_targets: []}",
          "{name: SINGLE_POSE, joint_targets: [null, null, null, null, null, null]}",
          "{name: SINGLE_POSE, joint_targets: [{torso_coxa: 0}, null, null, null, null, null]}",
          "{name: SINGLE_POSE, joint_targets: [{torso_coxa: .nan, coxa_femur: 0, femur_tibia: 0}, null, "
          "null, null, null, null]}",
          "{name: CONTINUOUS_POSE, restore_pose: true}",
          "{name: SINGLE_POSE, restore_pose: true, duration_s: 0}",
          "{name: SINGLE_POSE, restore_pose: true, save_pose: true}",
          "{name: SINGLE_POSE, restore_pose: true, body_pose: {head_orientation: {}}}",
          "{name: SINGLE_POSE, restore_pose: true, pose_delta: {torso_translation: {}}}"}) {
        SCOPED_TRACE(movement);
        EXPECT_FALSE(parser_.parseString(std::string("name: invalid\nactions:\n  - movement: ") + movement));
    }
}

TEST_F(SequenceParserTest, RejectsInvalidRelativePoseTargets) {
    for (const auto* movement :
         {"{name: SINGLE_POSE, pose_delta: {}}", "{name: SINGLE_POSE, pose_delta: {toe_translation: [{}]}}",
          "{name: SINGLE_POSE, pose_delta: {coxa_rotation_deg: [0, 0, 0, 0, 0]}}",
          "{name: SINGLE_POSE, pose_delta: {torso_translation: {x: .nan}}}",
          "{name: SINGLE_POSE, pose_delta: {coxa_rotation_deg: [0, 0, 0, .inf, 0, 0]}}",
          "{name: SINGLE_POSE, pose_delta: {unknown: 1}}",
          "{name: CONTINUOUS_POSE, pose_delta: {torso_translation: {x: 0.01}}}",
          "{name: SINGLE_POSE, duration_s: 0, pose_delta: {torso_translation: {x: 0.01}}}",
          "{name: SINGLE_POSE, body_pose: {torso_pose: {}}, pose_delta: {torso_translation: {}}}",
          "{name: SEQUENCE_CLAP}"}) {
        SCOPED_TRACE(movement);
        EXPECT_FALSE(parser_.parseString(std::string("name: invalid\nactions:\n  - movement: ") + movement));
    }
}

TEST_F(SequenceParserTest, SensorGaitsUseStandardMovementRequests) {
    ASSERT_TRUE(parser_.parseString(R"(
name: sensors
actions:
  - movement: {name: HOLD_DISTANCE, distance_m: 0.6, duration_s: 10}
  - movement: {name: LEVEL_TORSO, duration_s: 2}
)"));
    const auto hold = request<RequestMovementType>("sensors", 0, 0);
    const auto level = request<RequestMovementType>("sensors", 1, 0);
    ASSERT_TRUE(hold);
    ASSERT_TRUE(level);
    EXPECT_EQ(hold->movementRequest.type, MovementRequest::HOLD_DISTANCE);
    ASSERT_TRUE(hold->movementRequest.distance_m);
    EXPECT_DOUBLE_EQ(hold->movementRequest.distance_m->numerical_value_in(m), 0.6);
    EXPECT_EQ(hold->movementRequest.duration_s, 10.0 * s);
    EXPECT_EQ(level->movementRequest.type, MovementRequest::LEVEL_TORSO);
    EXPECT_FALSE(level->movementRequest.distance_m);
    EXPECT_EQ(level->movementRequest.duration_s, 2.0 * s);
    for (const auto* action :
         {"{movement: {name: HOLD_DISTANCE}}", "{movement: {name: HOLD_DISTANCE, distance_m: 0}}",
          "{movement: {name: HOLD_DISTANCE, distance_m: .nan}}",
          "{movement: {name: HOLD_DISTANCE, distance_m: 0.6, duration_s: 0}}",
          "{movement: {name: LEVEL_TORSO, duration_s: .nan}}",
          "{movement: {name: LEVEL_TORSO, distance_m: 0.6}}",
          "{movement: {name: MOVE_COMBINED, distance_m: 0.6}}",
          "{movement: {name: LEVEL_TORSO, body_pose: {head_orientation: {}}}}",
          "{movement: {name: HOLD_DISTANCE, distance_m: 0.6}, velocity: {linear: {x: 0.1}}}",
          "{hold_distance: {distance_m: 0.6, duration_s: 1}}", "{level_torso: {duration_s: 1}}"}) {
        EXPECT_FALSE(parser_.parseString(std::string("name: invalid\nactions: [") + action + "]"));
        EXPECT_TRUE(parser_.getSequence("sensors"));
    }
    ASSERT_TRUE(parser_.parseString(R"(
name: loop
actions:
  - loop:
      times: 2
      actions:
        - movement: {name: HOLD_DISTANCE, distance_m: 0.7}
        - movement: {name: LEVEL_TORSO}
)"));
    EXPECT_NE(request<RequestMovementType>("loop", 0, 0), request<RequestMovementType>("loop", 2, 0));
    EXPECT_NE(request<RequestMovementType>("loop", 1, 0), request<RequestMovementType>("loop", 3, 0));
}

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

TEST_F(SequenceFilesTest, ImportsTypedDefaultsWithLocalOverridesAndIndependentSnippets) {
    write("variables/common.yaml", R"(
variables:
  kDuration: 0.5
  kAlias: "${kDuration}"
  kCount: 2
  kMovement: SINGLE_POSE
  kPose: {head_orientation: {yaw: 10}}
)");
    write("sequence.yaml", R"(
name: shared
imports: [variables/common.yaml]
variables:
  kDuration: 0.7
actions:
  - loop:
      times: "${kCount}"
      actions:
        - movement:
            name: "${kMovement}"
            duration_s: "${kAlias}"
            body_pose: "${kPose}"
  - include: snippets/shared.yaml
)");
    write("snippets/shared.yaml", R"(
imports: [../variables/common.yaml]
variables:
  kLocalAlias: "${kAlias}"
actions:
  - movement:
      name: "${kMovement}"
      duration_s: "${kLocalAlias}"
      body_pose: "${kPose}"
)");
    ASSERT_TRUE(parser_.parseDirectory(directory_.string()));
    ASSERT_EQ(parser_.getSequences().size(), 1u);
    ASSERT_EQ(parser_.getSequence("shared")->get().actionGroups.size(), 3u);
    for (size_t i = 0; i < 3; ++i) {
        const auto parsed = request<RequestMovementType>("shared", i, 0);
        ASSERT_TRUE(parsed);
        EXPECT_DOUBLE_EQ(parsed->movementRequest.duration_s.numerical_value_in(s), i < 2 ? 0.7 : 0.5);
    }
    write("snippets/shared.yaml", "actions: [{say: \"${kMovement}\"}]");
    EXPECT_FALSE(parser_.parseFile((directory_ / "sequence.yaml").string()));
    EXPECT_TRUE(parser_.getSequence("shared"));
}

TEST_F(SequenceFilesTest, RejectsInvalidImportsWithoutReplacingCatalog) {
    ASSERT_TRUE(parser_.parseString("name: kept\nactions: [{say: hello}]"));
    write("variables/a.yaml", "variables: {kValue: 1}");
    write("variables/b.yaml", "variables: {kValue: 2}");
    for (const auto* imports :
         {"null", "{}", "variables/a.yaml", "[{}]", "['']", "[missing.yaml]",
          "[variables/a.yaml, variables/b.yaml]", "[variables/a.yaml, variables/a.yaml]"}) {
        write("sequence.yaml",
              std::string("name: invalid\nimports: ") + imports + "\nactions: [{say: hello}]");
        EXPECT_FALSE(parser_.parseFile((directory_ / "sequence.yaml").string())) << imports;
        EXPECT_TRUE(parser_.getSequence("kept"));
    }
    for (const auto* contents :
         {"{}", "variables: []", "variables: null", "variables: {kValue: 1, kValue: 2}",
          "variables: {}\nimports: []", "actions: [{say: hello}]"}) {
        write("variables/broken.yaml", contents);
        write("sequence.yaml", "name: invalid\nimports: [variables/broken.yaml]\nactions: [{say: hello}]");
        EXPECT_FALSE(parser_.parseFile((directory_ / "sequence.yaml").string())) << contents;
        EXPECT_TRUE(parser_.getSequence("kept"));
    }
}

TEST_F(SequenceParserTest, ResolvesLocalReferencesAndListsWithExistingAnchors) {
    ASSERT_TRUE(parser_.parseString(R"(
name: local
variables:
  kText: &kText hello
  kAlias: "${kText}"
  kActions: [{say: "${kAlias}"}]
actions:
  - loop:
      times: 2
      actions: "${kActions}"
  - say: *kText
)"));
    ASSERT_EQ(parser_.getSequence("local")->get().actionGroups.size(), 3u);
}

TEST_F(SequenceParserTest, RejectsInvalidReferencesAndDefinitionsTransactionally) {
    ASSERT_TRUE(parser_.parseString("name: kept\nactions: [{say: hello}]"));
    for (const auto* variables : {"{kA: \"${missing}\"}", "{kA: \"${kA}\"}", "{kA: \"${kB}\", kB: \"${kA}\"}",
                                  "{kA: 1, kA: 2}", "{'': 1}"}) {
        EXPECT_FALSE(parser_.parseString(std::string("name: invalid\nvariables: ") + variables +
                                         "\nactions: [{say: hello}]"))
            << variables;
        EXPECT_TRUE(parser_.getSequence("kept"));
    }
    EXPECT_FALSE(parser_.parseString("name: invalid\nactions: [{say: \"${missing}\"}]"));
    EXPECT_FALSE(parser_.parseString("name: invalid\nimports: [common.yaml]\nactions: [{say: hello}]"));
    EXPECT_TRUE(parser_.getSequence("kept"));
}

TEST_F(SequenceFilesTest, LoadsFeedbackFromSnippetWithIndependentLoopRequests) {
    write("hold.yaml", "name: hold\nactions: [{include: snippets/hold.yaml}]\n");
    write("snippets/hold.yaml",
          "actions:\n  - loop:\n      times: 2\n      actions: [{movement: {name: HOLD_DISTANCE, distance_m: "
          "0.6, "
          "duration_s: 1}}]\n");
    ASSERT_TRUE(parser_.parseFile((directory_ / "hold.yaml").string()));
    const auto first = request<RequestMovementType>("hold", 0, 0);
    const auto second = request<RequestMovementType>("hold", 1, 0);
    ASSERT_TRUE(first);
    ASSERT_TRUE(second);
    EXPECT_NE(first, second);
}

TEST_F(SequenceFilesTest, LoadsAllFilesAndNestedSnippetsInOrder) {
    write("sequences/b.yml", "name: second\nactions: [{say: second}]\n");
    write("sequences/a.yaml", R"(
name: first
trigger: {voice: firstCommand}
actions:
  - say: before
  - include: snippets/outer.yaml
  - include: snippets/inner.yaml
  - say: after
)");
    write("sequences/snippets/outer.yaml", "actions:\n  - include: inner.yaml\n");
    write("sequences/snippets/inner.yaml", "actions: [{say: inside, music: STOP}]\n");
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
          "actions: [", "name: invalid", "actions: [{include: [], say: mixed}]"}) {
        write("snippet.yaml", snippet);
        EXPECT_FALSE(parser_.parseFile(file)) << snippet;
        EXPECT_EQ(parser_.getSequences().size(), 30u);
        EXPECT_TRUE(parser_.getSequenceForVoiceRequest("commandStandup"));
    }
}

TEST_F(SequenceFilesTest, RejectsDuplicateNamesAndInvalidDirectoryEntries) {
    write("a.yaml", "name: duplicate\nactions: [{say: hello}]\n");
    write("b.yaml", "name: duplicate\nactions: [{say: other}]\n");
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
    EXPECT_EQ(parser_.getSequences().size(), 30u);
    EXPECT_FALSE(parser_.getSequence("missing"));
    EXPECT_FALSE(parser_.getSequenceForVoiceRequest("missing"));
    rumblex_interfaces::msg::JoystickRequest joystick;
    EXPECT_FALSE(parser_.getSequenceForJoystickRequest(joystick));
    joystick.button_a = true;
    const auto watch = parser_.getSequenceForJoystickRequest(joystick);
    ASSERT_TRUE(watch);
    EXPECT_EQ(watch->get().name, "watch");
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
    EXPECT_FALSE(music->background);
    auto party = request<RequestMusic>("dance_excess_party", 0, 0);
    ASSERT_TRUE(party);
    EXPECT_EQ(party->song, "musicfox_excess_party.mp3");
    EXPECT_TRUE(party->background);
}

TEST_F(SequenceParserTest, SquareUsesCombinedGaitAndMidrangeVelocitiesThenStops) {
    ASSERT_TRUE(parser_.parseFile(std::string(RUMBLEX_SEQUENCES_DIRECTORY) + "/run_in_a_square.yaml"));
    const auto sequence = parser_.getSequence("run_in_a_square");
    ASSERT_TRUE(sequence);
    ASSERT_EQ(sequence->get().actionGroups.size(), 10u);
    const auto standup = request<RequestMovementType>("run_in_a_square", 0, 0);
    ASSERT_TRUE(standup);
    EXPECT_EQ(standup->movementRequest.type, MovementRequest::SEQUENCE_STAND_UP);
    for (size_t i = 1; i <= 8; ++i) {
        SCOPED_TRACE(i);
        const auto movement = request<RequestMovementType>("run_in_a_square", i, 0);
        const auto velocity = request<RequestVelocity>("run_in_a_square", i, 1);
        ASSERT_TRUE(movement);
        ASSERT_TRUE(velocity);
        EXPECT_EQ(movement->movementRequest.type, MovementRequest::MOVE_COMBINED);
        geometry_msgs::msg::Twist expected;
        const double seconds = movement->movementRequest.duration_s.numerical_value_in(s);
        if (i % 2 == 1) {
            expected.linear.x = 0.05;
            EXPECT_DOUBLE_EQ(seconds, 10.0);
            EXPECT_DOUBLE_EQ(seconds * velocity->velocity.linear.x, 0.5);
        } else {
            expected.angular.z = 0.20;
            EXPECT_NEAR(seconds * velocity->velocity.angular.z, 1.5707963267948966, 1e-9);
        }
        EXPECT_EQ(velocity->velocity, expected);
    }
    const auto stationary = request<RequestMovementType>("run_in_a_square", 9, 0);
    const auto stop = request<RequestVelocity>("run_in_a_square", 9, 1);
    ASSERT_TRUE(stationary);
    ASSERT_TRUE(stop);
    EXPECT_EQ(stationary->movementRequest.type, MovementRequest::CONTINUOUS_POSE);
    EXPECT_EQ(stop->velocity, geometry_msgs::msg::Twist());
}

TEST_F(SequenceParserTest, CatalogPresetsPrepareStandupAndFinishStationary) {
    ASSERT_TRUE(parser_.parseDirectory(RUMBLEX_SEQUENCES_DIRECTORY));
    struct Preset {
        const char* name;
        uint8_t gait;
        double seconds;
    };
    for (const auto& preset : {Preset{"hold_distance", MovementRequest::HOLD_DISTANCE, 10.0},
                               Preset{"level_torso", MovementRequest::LEVEL_TORSO, 5.0},
                               Preset{"waiting", MovementRequest::SEQUENCE_WAITING, 5.0},
                               Preset{"run_demo", MovementRequest::MOVE_TRIPOD, 3.0}}) {
        SCOPED_TRACE(preset.name);
        const auto sequence = parser_.getSequence(preset.name);
        ASSERT_TRUE(sequence);
        ASSERT_EQ(sequence->get().actionGroups.size(), 3u);
        EXPECT_EQ(sequence->get().actionGroups[0].size(), 1u);
        const auto standup = request<RequestMovementType>(preset.name, 0, 0);
        ASSERT_TRUE(standup);
        EXPECT_EQ(standup->movementRequest.type, MovementRequest::SEQUENCE_STAND_UP);
        EXPECT_EQ(standup->movementRequest.duration_s, 1.5 * s);

        const auto movement = request<RequestMovementType>(preset.name, 1, 0);
        ASSERT_TRUE(movement);
        EXPECT_EQ(movement->movementRequest.type, preset.gait);
        EXPECT_EQ(movement->movementRequest.duration_s, preset.seconds * s);
        EXPECT_FALSE(movement->movementRequest.body_pose.has_head_orientation);
        EXPECT_FALSE(movement->movementRequest.body_pose.has_torso_pose);
        EXPECT_FALSE(movement->movementRequest.body_pose.has_toe_positions);
        if (preset.gait == MovementRequest::HOLD_DISTANCE) {
            ASSERT_TRUE(movement->movementRequest.distance_m);
            EXPECT_EQ(*movement->movementRequest.distance_m, 0.6 * m);
        } else {
            EXPECT_FALSE(movement->movementRequest.distance_m);
        }
        if (preset.gait == MovementRequest::MOVE_TRIPOD) {
            EXPECT_EQ(sequence->get().actionGroups[1].size(), 2u);
            const auto velocity = request<RequestVelocity>(preset.name, 1, 1);
            ASSERT_TRUE(velocity);
            geometry_msgs::msg::Twist expected;
            expected.linear.x = 0.03;
            EXPECT_EQ(velocity->velocity, expected);
        } else {
            EXPECT_EQ(sequence->get().actionGroups[1].size(), 1u);
        }

        EXPECT_EQ(sequence->get().actionGroups[2].size(), 2u);
        const auto stationary = request<RequestMovementType>(preset.name, 2, 0);
        const auto stop = request<RequestVelocity>(preset.name, 2, 1);
        ASSERT_TRUE(stationary);
        ASSERT_TRUE(stop);
        EXPECT_EQ(stationary->movementRequest.type, MovementRequest::CONTINUOUS_POSE);
        EXPECT_FALSE(stationary->movementRequest.body_pose.has_head_orientation);
        EXPECT_FALSE(stationary->movementRequest.body_pose.has_torso_pose);
        EXPECT_FALSE(stationary->movementRequest.body_pose.has_toe_positions);
        EXPECT_EQ(stop->velocity, geometry_msgs::msg::Twist());
    }
}

TEST_F(SequenceParserTest, PreservesDanceGroupsAndPoseUnits) {
    ASSERT_TRUE(parser_.parseDirectory(RUMBLEX_SEQUENCES_DIRECTORY));
    const auto dance = parser_.getSequence("dance_hot_dogs_for_breakfast");
    ASSERT_TRUE(dance);
    ASSERT_EQ(dance->get().actionGroups.size(), 66u);
    auto music = request<RequestMusic>(dance->get().name, 1, 1);
    ASSERT_TRUE(music);
    EXPECT_EQ(music->song, "musicfox_hot_dogs_for_breakfast.mp3");
    EXPECT_TRUE(music->background);
    double duration = 0;
    for (size_t i = 1; i <= 64; ++i) {
        const auto movement = request<RequestMovementType>(dance->get().name, i, 0);
        ASSERT_TRUE(movement);
        ASSERT_TRUE(movement->movementRequest.pose_delta);
        EXPECT_EQ(movement->movementRequest.interpolation, InterpolationType::SinSquared);
        EXPECT_DOUBLE_EQ(movement->movementRequest.duration_s.numerical_value_in(s), 0.438);
        duration += movement->movementRequest.duration_s.numerical_value_in(s);
    }
    EXPECT_NEAR(duration, 28.032, 1e-9);
    const auto standard = parser_.getSequence("dance_standard");
    ASSERT_TRUE(standard);
    EXPECT_EQ(standard->get().actionGroups.size(), dance->get().actionGroups.size());
    auto stop = request<RequestMusic>(dance->get().name, 65, 0);
    ASSERT_TRUE(stop);
    EXPECT_EQ(stop->song, "STOP");
}

TEST_F(SequenceParserTest, ValidatesMusicFadeDuration) {
    for (const auto* value : {"-1", ".nan", ".inf", "61", "invalid"}) {
        EXPECT_FALSE(parser_.parseString(
            std::string("name: fade\nactions: [{music: {song: STOP, fade_out_s: ") + value + "}}]"));
    }
    EXPECT_FALSE(parser_.parseString("name: fade\nactions: [{music: {song: track.mp3, fade_out_s: 4}}]"));
    ASSERT_TRUE(parser_.parseString("name: fade\nactions: [{music: {song: STOP, fade_out_s: 4}}]"));
    const auto fade = request<RequestMusic>("fade", 0, 0);
    ASSERT_TRUE(fade);
    EXPECT_FLOAT_EQ(fade->fade_out_s, 4.0f);
}

TEST_F(SequenceParserTest, AllYamlMusicPlaybackRunsInBackground) {
    ASSERT_TRUE(parser_.parseString(R"(
sequences:
  - name: background_music
    actions:
      - music: metal.mp3
      - music: {song: dance.mp3}
      - music: {song: legacy.mp3, background: false}
      - music: {song: explicit.mp3, background: true}
      - music: STOP
)"));
    for (size_t index = 0; index < 4; ++index) {
        auto music = request<RequestMusic>("background_music", index, 0);
        ASSERT_TRUE(music);
        EXPECT_TRUE(music->background);
    }
    auto stop = request<RequestMusic>("background_music", 4, 0);
    ASSERT_TRUE(stop);
    EXPECT_FALSE(stop->background);
}

TEST_F(SequenceParserTest, ParsesScalarFormsAndDefaultsInLexicographicOrder) {
    ASSERT_TRUE(parser_.parseString(R"(
sequences:
  - name: scalars
    actions:
      - say: "Hallo: Welt # gesprochen"
        music: STOP
        listening: true
        chat: Frage
      - listening: false
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
      - velocity: {linear: {x: 0.1}, angular: {z: 0.2}, minDuration: 0.9}
        say: {text: Hello, language: en, minDuration: 0.8}
        system: {turnOffServoRelay: true, systemShutdown: true, minDuration: 0.7}
        music: {song: dance.mp3, volume: 0.3, minDuration: 0.5}
        movement:
          name: SEQUENCE_WATCH
          direction: ANTICLOCKWISE
          duration_s: 3
          minDuration: 0.4
          body_pose:
            torso_pose: {position: {z: -0.05}, orientation: {pitch: 15}}
            head_orientation: {yaw: -12}
        listening: {active: true, minDuration: 0.3}
        chat: {text: Question, language: en, minDuration: 0.1}
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
    EXPECT_EQ(movement->movementRequest.type, MovementRequest::SEQUENCE_WATCH);
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
      - chat: {}
        listening: {}
        music: {}
        system: {}
        say: {}
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
      - music: {song: dance.mp3, volume: loud}
)"));
}

TEST_F(SequenceParserTest, InvalidActionsRetainPreviousCatalog) {
    ASSERT_TRUE(parser_.parseDirectory(RUMBLEX_SEQUENCES_DIRECTORY));
    for (const auto* text :
         {"name: invalid\nactions: [{UnknownRequest: ignored, say: hello}]",
          "sequences: [{name: valid, actions: [{say: hi}]}, {name: invalid, actions: []}]",
          "name: invalid\nactions: [{movement: {name: SINGLE_POSE, duration_s: invalid}, say: hello}]"}) {
        EXPECT_FALSE(parser_.parseString(text));
        EXPECT_EQ(parser_.getSequences().size(), 30u);
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
          right_front: {x: 0.2, y: -0.16, z: -0.05}
          right_mid: {x: 0, y: -0.22, z: -0.05}
          right_back: {x: -0.2, y: -0.16, z: -0.05}
          left_front: {x: 0.2, y: 0.16, z: -0.05}
          left_mid: {x: 0, y: 0.22, z: -0.05}
          left_back: {x: -0.2, y: 0.16, z: -0.05}
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
TEST_F(SequenceParserTest, AbsolutePoseAllowsShortDurationsAndKeepsOneSecondDefault) {
    ASSERT_TRUE(parser_.parseString(R"(
name: short_poses
actions:
  - body_pose: {duration_s: 0.1, head_orientation: {yaw: 2}}
  - body_pose: {duration_s: 0.5, head_orientation: {yaw: 0}}
  - body_pose: {head_orientation: {}}
  - movement: {name: SINGLE_POSE, duration_s: 0.1, save_pose: true,
               joint_targets: {right_front: {torso_coxa: 0, coxa_femur: 50, femur_tibia: 90}}}
  - movement: {name: SINGLE_POSE, duration_s: 0.1, restore_pose: true}
)"));
    const std::array durations = {0.1, 0.5, 1.0, 0.1, 0.1};
    for (size_t i = 0; i < durations.size(); ++i) {
        const auto movement = request<RequestMovementType>("short_poses", i, 0);
        ASSERT_TRUE(movement);
        EXPECT_EQ(movement->movementRequest.duration_s, durations[i] * s);
    }
}

TEST_F(SequenceParserTest, RejectsMalformedPoseAndConflictingActions) {
    for (const auto* action :
         {"body_pose: {}", "body_pose: {head_orientation: null}", "body_pose: {head_orientation: {yaaw: 12}}",
          "body_pose: {head_orientation: {yaw: .nan}}", "body_pose: {torso_pose: {position: {x: .inf}}}",
          "body_pose: {head_orientation: {}, duration_s: 0.099}",
          "body_pose: {head_orientation: {}, duration_s: 0}",
          "body_pose: {head_orientation: {}, duration_s: -1}",
          "body_pose: {head_orientation: {}, duration_s: .nan}",
          "body_pose: {head_orientation: {}, duration_s: .inf}", "body_pose: {toe_positions: []}",
          "body_pose: {head_orientation: {}, head_orientation: {yaw: 3}}",
          "body_pose: {head_orientation: {}}, movement: {name: SINGLE_POSE}",
          "body_pose: {head_orientation: {}}, RequestHeadOrientation: {yaw: 4}", "say: one, say: two",
          "movement: {name: UNKNOWN}", "movement: {name: SINGLE_POSE}"}) {
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
        - include: snippets/outer.yaml
  - say: after
)");
    write("sequences/snippets/outer.yaml", "actions: [{include: inner.yaml}]\n");
    write("sequences/snippets/inner.yaml", R"(
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
TEST_F(SequenceParserTest, RemovedLegacySyntaxIsRejectedTransactionally) {
    ASSERT_TRUE(parser_.parseString("name: kept\nactions: [{say: hello}]"));
    for (const auto* input : {"behaviors: [{name: old, actions: [{say: hello}]}]",
                              "name: old\nactions: [{RequestTalking: hello}]",
                              "name: old\nactions: [{RequestMovementType: {name: SINGLE_POSE}}]",
                              "name: old\nactions: [{movement: {type: SEQUENCE_WATCH}}]",
                              "name: old\nactions: [{RequestSinglePose: {orientation: {roll: 8}}}]",
                              "name: old\nactions: [{RequestHeadOrientation: {yaw: -12}}]"}) {
        EXPECT_FALSE(parser_.parseString(input));
        ASSERT_TRUE(parser_.getSequence("kept"));
    }
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
