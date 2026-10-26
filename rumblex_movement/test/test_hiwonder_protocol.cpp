#include <gtest/gtest.h>
#include <poll.h>
#include <pty.h>
#include <unistd.h>

#include <array>
#include <memory>
#include <vector>

#include "handler/hiwonder_protocol.hpp"

namespace rumblex_movement {
namespace {

class HiwonderProtocolTest : public ::testing::Test {
   protected:
    void SetUp() override {
        char slave_name[256];
        ASSERT_EQ(openpty(&master_fd_, &slave_fd_, slave_name, nullptr, nullptr), 0);
        // Logging is disabled in this protocol; no ROS node or hardware is needed.
        protocol_ = std::make_unique<CHiwonderProtocol>(nullptr, slave_name);
        ASSERT_TRUE(protocol_->triggerConnection());
    }

    void TearDown() override {
        if (protocol_) protocol_->closeConnection();
        if (master_fd_ >= 0) close(master_fd_);
        if (slave_fd_ >= 0) close(slave_fd_);
    }

    void expectPacket(const std::vector<uint8_t>& expected) {
        std::vector<uint8_t> actual(expected.size());
        size_t received = 0;
        while (received < actual.size()) {
            pollfd descriptor{master_fd_, POLLIN, 0};
            ASSERT_EQ(poll(&descriptor, 1, 1000), 1);
            const auto count = read(master_fd_, actual.data() + received, actual.size() - received);
            ASSERT_GT(count, 0);
            received += static_cast<size_t>(count);
        }
        EXPECT_EQ(actual, expected);
        pollfd descriptor{master_fd_, POLLIN, 0};
        EXPECT_EQ(poll(&descriptor, 1, 0), 0);
    }

    void reply(std::array<uint8_t, 10> packet) {
        uint8_t sum = 0;
        for (size_t index = 2; index < packet.size() - 1; ++index) sum += packet[index];
        packet.back() = static_cast<uint8_t>(~sum);
        ASSERT_EQ(write(master_fd_, packet.data(), packet.size()), static_cast<ssize_t>(packet.size()));
    }

    int master_fd_ = -1;
    int slave_fd_ = -1;
    std::unique_ptr<CHiwonderProtocol> protocol_;
};

TEST_F(HiwonderProtocolTest, WritesSignedVelocityAndClampsToMotorRange) {
    CServoProtocol& protocol = *protocol_;
    for (const int16_t speed : {1000, 32767}) {
        ASSERT_TRUE(protocol.setVelocity(1, speed));
        expectPacket({0x55, 0x55, 1, 7, 29, 1, 0, 0xE8, 0x03, 0xEE});
    }
    for (const int16_t speed : {-1000, -32768}) {
        ASSERT_TRUE(protocol.setVelocity(1, speed));
        expectPacket({0x55, 0x55, 1, 7, 29, 1, 0, 0x18, 0xFC, 0xC5});
    }
    ASSERT_TRUE(protocol.setVelocity(1, 0));
    expectPacket({0x55, 0x55, 1, 7, 29, 1, 0, 0, 0, 0xD9});
}

TEST_F(HiwonderProtocolTest, ExistingModeSettersWriteCompletePackets) {
    ASSERT_TRUE(protocol_->setMotorMode(1, -1000));
    expectPacket({0x55, 0x55, 1, 7, 29, 1, 0, 0x18, 0xFC, 0xC5});
    ASSERT_TRUE(protocol_->setServoMode(1));
    expectPacket({0x55, 0x55, 1, 7, 29, 0, 0, 0, 0, 0xDA});
}

TEST_F(HiwonderProtocolTest, ReadsConfiguredSignedVelocity) {
    CServoProtocol& protocol = *protocol_;
    for (const int16_t expected : {-1000, -1, 0, 1, 1000}) {
        const auto raw = static_cast<uint16_t>(expected);
        reply({0x55, 0x55, 1, 7, 30, 1, 0, static_cast<uint8_t>(raw), static_cast<uint8_t>(raw >> 8), 0});
        int16_t velocity = 1234;
        ASSERT_TRUE(protocol.getVelocity(1, velocity));
        EXPECT_EQ(velocity, expected);
        expectPacket({0x55, 0x55, 1, 3, 30, 0xDD});
    }
}

TEST_F(HiwonderProtocolTest, ExistingMotorSpeedGetterReadsVelocity) {
    reply({0x55, 0x55, 1, 7, 30, 1, 0, 0x18, 0xFC, 0});
    int16_t speed = 0;
    ASSERT_TRUE(protocol_->getMotorSpeed(1, speed));
    EXPECT_EQ(speed, -1000);
    expectPacket({0x55, 0x55, 1, 3, 30, 0xDD});
}

TEST_F(HiwonderProtocolTest, RejectsPositionModeAndMalformedRepliesWithoutChangingOutput) {
    const std::array<uint8_t, 10> valid{0x55, 0x55, 1, 7, 30, 1, 0, 0xE8, 0x03, 0xED};
    // Incorrect header, ID, length, opcode, or mode, each with an otherwise valid checksum.
    for (const size_t index : {0, 1, 2, 3, 4, 5}) {
        auto packet = valid;
        packet[index] = 0;
        reply(packet);
        int16_t velocity = 123;
        EXPECT_FALSE(protocol_->getVelocity(1, velocity));
        EXPECT_EQ(velocity, 123);
        expectPacket({0x55, 0x55, 1, 3, 30, 0xDD});
    }
    auto packet = valid;
    packet.back() ^= 1;
    ASSERT_EQ(write(master_fd_, packet.data(), packet.size()), static_cast<ssize_t>(packet.size()));
    int16_t velocity = 123;
    EXPECT_FALSE(protocol_->getVelocity(1, velocity));
    EXPECT_EQ(velocity, 123);
    expectPacket({0x55, 0x55, 1, 3, 30, 0xDD});
}

TEST_F(HiwonderProtocolTest, RejectsBroadcastAndInvalidReadIDsWithoutSending) {
    int16_t velocity = 123;
    EXPECT_FALSE(protocol_->getVelocity(SERVO_Broadcast_ID, velocity));
    EXPECT_FALSE(protocol_->getVelocity(0xFF, velocity));
    EXPECT_EQ(velocity, 123);
    expectPacket({});
}

TEST(HiwonderProtocol, DisconnectedOperationsFailWithoutChangingOutput) {
    CHiwonderProtocol protocol(nullptr, "/unused");
    int16_t velocity = 123;
    EXPECT_FALSE(protocol.setVelocity(1, 100));
    EXPECT_FALSE(protocol.getVelocity(1, velocity));
    EXPECT_EQ(velocity, 123);
}

}  // namespace
}  // namespace rumblex_movement
