#include <gtest/gtest.h>
#include <pty.h>
#include <unistd.h>

#include "SCSerial.h"

TEST(SCSerial, RejectsUnsupportedBaudRateOnOpenPort) {
    int master_fd;
    int slave_fd;
    char slave_name[256];
    ASSERT_EQ(openpty(&master_fd, &slave_fd, slave_name, nullptr, nullptr), 0);
    SCSerial serial;
    const bool opened = serial.begin(115200, slave_name);
    EXPECT_TRUE(opened);
    if (opened) {
        EXPECT_EQ(serial.setBaudRate(12345), -1);
        EXPECT_EQ(serial.setBaudRate(230400), 1);
        serial.end();
    }
    close(master_fd);
    close(slave_fd);
}
