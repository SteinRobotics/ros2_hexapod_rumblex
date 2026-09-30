/*******************************************************************************
 * Copyright (c) 2024 Christian Stein
 ******************************************************************************/

#include "handler/hiwonder_protocol.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <string>

namespace rumblex_movement {

namespace {

constexpr bool SERIAL_DEBUG = false;
constexpr uint8_t SERVO_FRAME_HEADER = 0x55;

inline constexpr uint8_t getLowByte(uint16_t value) {
    return static_cast<uint8_t>(value & 0xFF);
}

inline constexpr uint8_t getHighByte(uint16_t value) {
    return static_cast<uint8_t>(value >> 8);
}

inline constexpr uint16_t byteToHW(uint8_t high, uint8_t low) {
    return static_cast<uint16_t>((static_cast<uint16_t>(high) << 8) | low);
}

constexpr uint8_t SERVO_MOVE_TIME_WRITE = 1;
constexpr uint8_t SERVO_MOVE_TIME_READ = 2;
constexpr uint8_t SERVO_MOVE_TIME_WAIT_WRITE = 7;
constexpr uint8_t SERVO_MOVE_TIME_WAIT_READ = 8;
constexpr uint8_t SERVO_MOVE_START = 11;
constexpr uint8_t SERVO_MOVE_STOP = 12;
constexpr uint8_t SERVO_ID_WRITE = 13;
constexpr uint8_t SERVO_ID_READ = 14;
constexpr uint8_t SERVO_ANGLE_OFFSET_ADJUST = 17;
constexpr uint8_t SERVO_ANGLE_OFFSET_WRITE = 18;
constexpr uint8_t SERVO_ANGLE_OFFSET_READ = 19;
constexpr uint8_t SERVO_ANGLE_LIMITS_WRITE = 20;
constexpr uint8_t SERVO_ANGLE_LIMITS_READ = 21;
constexpr uint8_t SERVO_VIN_LIMITS_WRITE = 22;
constexpr uint8_t SERVO_VIN_LIMITS_READ = 23;
constexpr uint8_t SERVO_TEMP_LIMIT_WRITE = 24;
constexpr uint8_t SERVO_TEMP_LIMIT_READ = 25;
constexpr uint8_t SERVO_TEMP_READ = 26;
constexpr uint8_t SERVO_VIN_READ = 27;
constexpr uint8_t SERVO_POS_READ = 28;
constexpr uint8_t SERVO_OR_MOTOR_MODE_WRITE = 29;
constexpr uint8_t SERVO_OR_MOTOR_MODE_READ = 30;
constexpr uint8_t SERVO_TORQUE_WRITE = 31;
constexpr uint8_t SERVO_TORQUE_READ = 32;
constexpr uint8_t SERVO_LED_CTRL_WRITE = 33;
constexpr uint8_t SERVO_LED_CTRL_READ = 34;
constexpr uint8_t SERVO_LED_ERROR_WRITE = 35;
constexpr uint8_t SERVO_LED_ERROR_READ = 36;

constexpr uint8_t s_servo_move_time_write = 7;
constexpr uint8_t s_servo_move_time_wait_write = 7;
constexpr uint8_t s_servo_move_start = 3;
constexpr uint8_t s_servo_move_stop = 3;
constexpr uint8_t s_servo_id_write = 4;
constexpr uint8_t s_servo_angle_offset_adjust = 4;
constexpr uint8_t s_servo_angle_offset_write = 3;
constexpr uint8_t s_servo_angle_limits_write = 7;
constexpr uint8_t s_servo_vin_limits_write = 7;
constexpr uint8_t s_servo_temp_limit_write = 4;
constexpr uint8_t s_servo_mode_write = 7;
constexpr uint8_t s_servo_torque_write = 4;
constexpr uint8_t s_servo_led_ctrl_write = 4;
constexpr uint8_t s_servo_led_error_write = 4;

constexpr uint8_t s_servo_all_read_cmds = 3;

constexpr uint8_t s_servo_move_time_read = 7;
constexpr uint8_t s_servo_move_time_wait_read = 7;
constexpr uint8_t s_servo_id_read = 4;
constexpr uint8_t s_servo_angle_offset_read = 4;
constexpr uint8_t s_servo_angle_limits_read = 7;
constexpr uint8_t s_servo_vin_limits_read = 7;
constexpr uint8_t s_servo_temp_limit_read = 4;
constexpr uint8_t s_servo_temp_read = 4;
constexpr uint8_t s_servo_vin_read = 5;
constexpr uint8_t s_servo_pos_read = 5;
constexpr uint8_t s_servo_mode_read = 7;
constexpr uint8_t s_servo_torque_read = 4;
constexpr uint8_t s_servo_led_ctrl_read = 4;
constexpr uint8_t s_servo_led_error_read = 4;

}  // namespace

CHiwonderProtocol::CHiwonderProtocol(std::shared_ptr<rclcpp::Node> node, const std::string& device_name)
    : CServoProtocol(node), device_name_(device_name) {
}

bool CHiwonderProtocol::triggerConnection() {
    is_connected_ = false;

    struct termios options;

    device_ = open(device_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (device_ == -1) return false;

    fcntl(device_, F_SETFL, 0);
    tcflush(device_, TCIOFLUSH);

    if (tcgetattr(device_, &options) < 0) {
        close(device_);
        return false;
    }

    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;
    options.c_cflag |= CREAD | CLOCAL;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~(INLCR | ICRNL | IGNCR);
    options.c_oflag &= ~OPOST;

    options.c_cc[VMIN] = 1;
    options.c_cc[VTIME] = 1;

    if (tcsetattr(device_, TCSANOW, &options) != 0) {
        close(device_);
        return false;
    }

    is_connected_ = true;
    return true;
}

bool CHiwonderProtocol::isConnected() {
    return is_connected_;
}

void CHiwonderProtocol::closeConnection() {
    close(device_);
}

bool CHiwonderProtocol::writeToServo(uint8_t buf[], int num_bytes) {
    ssize_t written = write(device_, buf, num_bytes);
    if (written < 0 || written != num_bytes) {
        return false;
    }
    if (SERIAL_DEBUG) {
        std::string text = "[";
        for (auto i = 0; i < 6; ++i) {
            text += std::to_string(buf[i]) + ", ";
        }
        text += "]";
        RCLCPP_INFO_STREAM(node_->get_logger(), "serial: " << text);
    }
    return true;
}

bool CHiwonderProtocol::readCMD(uint8_t id, uint8_t cmd_opcode, uint8_t rx_buf_size, uint8_t* rx_buf) {
    uint8_t tx_buf[s_servo_all_read_cmds + 3];
    bool syntax_check = false;

    tx_buf[0] = tx_buf[1] = SERVO_FRAME_HEADER;
    tx_buf[2] = id;
    tx_buf[3] = s_servo_all_read_cmds;
    tx_buf[4] = cmd_opcode;
    tx_buf[5] = checksum(tx_buf);

    if (!writeToServo(tx_buf, sizeof(tx_buf))) {
        return false;
    }

    int bytes_read = 0;
    while (bytes_read < rx_buf_size) {
        int result = read(device_, rx_buf + bytes_read, rx_buf_size - bytes_read);
        if (result < 0) {
            if (errno == EAGAIN || errno == EINTR) {
                continue;
            }
            return false;
        } else if (result == 0) {
            break;
        }
        bytes_read += result;
    }

    int rx_size = bytes_read;
    syntax_check = (rx_size == rx_buf_size);
    syntax_check = syntax_check && (rx_buf[0] == SERVO_FRAME_HEADER);
    syntax_check = syntax_check && (rx_buf[1] == SERVO_FRAME_HEADER);
    if (id != 0xFE) {
        syntax_check = syntax_check && (rx_buf[2] == id);
    }
    syntax_check = syntax_check && (rx_buf[3] == rx_buf_size - 3);
    syntax_check = syntax_check && (rx_buf[4] == cmd_opcode);
    syntax_check = syntax_check && (rx_buf[rx_buf_size - 1] == checksum(rx_buf));

    return syntax_check;
}

uint8_t CHiwonderProtocol::checksum(uint8_t buf[]) {
    uint8_t i;
    uint16_t chksum = 0;
    for (i = 2; i < buf[3] + 2; i++) {
        chksum += buf[i];
    }
    chksum = ~chksum;
    i = (uint8_t)chksum;
    return i;
}

bool CHiwonderProtocol::getServoID(uint8_t id, uint8_t& answer) {
    uint8_t cmd_opcode = SERVO_ID_READ;
    uint8_t rx_buf[s_servo_id_read + 3];
    bool syntax_check = readCMD(id, cmd_opcode, sizeof(rx_buf), rx_buf);
    if (syntax_check) {
        answer = rx_buf[5];
    }
    return syntax_check;
}

bool CHiwonderProtocol::setServoID(uint8_t id, uint8_t new_id) {
    uint8_t buf[s_servo_id_write + 3];

    new_id = std::clamp<uint8_t>(new_id, 0, 254);
    buf[0] = buf[1] = SERVO_FRAME_HEADER;
    buf[2] = id;
    buf[3] = s_servo_id_write;
    buf[4] = SERVO_ID_WRITE;
    buf[5] = new_id;
    buf[6] = checksum(buf);

    return writeToServo(buf, sizeof(buf));
}

bool CHiwonderProtocol::setPosition(uint8_t id, uint16_t pos, uint16_t time) {
    uint8_t buf[s_servo_move_time_write + 3];

    pos = std::clamp<uint16_t>(pos, 0, 1000);
    time = std::clamp<uint16_t>(time, 0, 30000);

    buf[0] = buf[1] = SERVO_FRAME_HEADER;
    buf[2] = id;
    buf[3] = s_servo_move_time_write;
    buf[4] = SERVO_MOVE_TIME_WRITE;
    buf[5] = getLowByte(pos);
    buf[6] = getHighByte(pos);
    buf[7] = getLowByte(time);
    buf[8] = getHighByte(time);
    buf[9] = checksum(buf);

    return writeToServo(buf, sizeof(buf));
}

bool CHiwonderProtocol::setRegPos(uint8_t id, uint16_t pos, uint16_t time) {
    uint8_t buf[s_servo_move_time_wait_write + 3];

    pos = std::clamp<uint16_t>(pos, 0, 1000);
    time = std::clamp<uint16_t>(time, 0, 30000);

    buf[0] = buf[1] = SERVO_FRAME_HEADER;
    buf[2] = id;
    buf[3] = s_servo_move_time_wait_write;
    buf[4] = SERVO_MOVE_TIME_WAIT_WRITE;
    buf[5] = getLowByte(pos);
    buf[6] = getHighByte(pos);
    buf[7] = getLowByte(time);
    buf[8] = getHighByte(time);
    buf[9] = checksum(buf);

    return writeToServo(buf, sizeof(buf));
}

bool CHiwonderProtocol::actionStart(uint8_t id) {
    uint8_t buf[s_servo_move_start + 3];

    buf[0] = buf[1] = SERVO_FRAME_HEADER;
    buf[2] = id;
    buf[3] = s_servo_move_start;
    buf[4] = SERVO_MOVE_START;
    buf[5] = checksum(buf);

    return writeToServo(buf, sizeof(buf));
}

bool CHiwonderProtocol::moveStop(uint8_t id) {
    uint8_t buf[s_servo_move_stop + 3];

    buf[0] = buf[1] = SERVO_FRAME_HEADER;
    buf[2] = id;
    buf[3] = s_servo_move_stop;
    buf[4] = SERVO_MOVE_STOP;
    buf[5] = checksum(buf);

    return writeToServo(buf, sizeof(buf));
}

bool CHiwonderProtocol::getPositionOffset(uint8_t id, int8_t& deviation) {
    uint8_t cmd_opcode = SERVO_ANGLE_OFFSET_READ;
    uint8_t rx_buf[s_servo_angle_offset_read + 3];
    bool syntax_check = readCMD(id, cmd_opcode, sizeof(rx_buf), rx_buf);
    if (syntax_check) {
        uint8_t raw = rx_buf[5];
        if (raw > 127) {
            deviation = static_cast<int8_t>(raw) - 127;
        } else {
            deviation = static_cast<int8_t>(raw);
        }
    }
    return syntax_check;
}

bool CHiwonderProtocol::setPositionOffset(uint8_t id, int8_t deviation) {
    uint8_t buf[s_servo_angle_offset_adjust + 3];

    uint8_t deviation_u8 = 0;
    if (deviation < 0) {
        deviation_u8 = static_cast<uint8_t>(deviation) + 127;
    } else {
        deviation_u8 = static_cast<uint8_t>(deviation);
    }
    buf[0] = buf[1] = SERVO_FRAME_HEADER;
    buf[2] = id;
    buf[3] = s_servo_angle_offset_adjust;
    buf[4] = SERVO_ANGLE_OFFSET_ADJUST;
    buf[5] = deviation_u8;
    buf[6] = checksum(buf);

    return writeToServo(buf, sizeof(buf));
}

bool CHiwonderProtocol::savePositionOffset(uint8_t id) {
    uint8_t buf[s_servo_angle_offset_write + 3];

    buf[0] = buf[1] = SERVO_FRAME_HEADER;
    buf[2] = id;
    buf[3] = s_servo_angle_offset_write;
    buf[4] = SERVO_ANGLE_OFFSET_WRITE;
    buf[5] = checksum(buf);

    return writeToServo(buf, sizeof(buf));
}

bool CHiwonderProtocol::getPositionLimits(uint8_t id, uint16_t& min_position, uint16_t& max_position) {
    uint8_t cmd_opcode = SERVO_ANGLE_LIMITS_READ;
    uint8_t rx_buf[s_servo_angle_limits_read + 3];
    bool syntax_check = readCMD(id, cmd_opcode, sizeof(rx_buf), rx_buf);
    if (syntax_check) {
        min_position = byteToHW(rx_buf[6], rx_buf[5]);
        max_position = byteToHW(rx_buf[8], rx_buf[7]);
    }
    return syntax_check;
}

bool CHiwonderProtocol::setPositionLimits(uint8_t id, uint16_t min_position, uint16_t max_position) {
    uint8_t buf[s_servo_angle_limits_write + 3];

    min_position = std::clamp<uint16_t>(min_position, 0, 1000);
    max_position = std::clamp<uint16_t>(max_position, 0, 1000);

    buf[0] = buf[1] = SERVO_FRAME_HEADER;
    buf[2] = id;
    buf[3] = s_servo_angle_limits_write;
    buf[4] = SERVO_ANGLE_LIMITS_WRITE;
    buf[5] = getLowByte(min_position);
    buf[6] = getHighByte(min_position);
    buf[7] = getLowByte(max_position);
    buf[8] = getHighByte(max_position);
    buf[9] = checksum(buf);

    return writeToServo(buf, sizeof(buf));
}

bool CHiwonderProtocol::flashLedErrCode(uint8_t id, uint8_t code) {
    uint8_t buf[s_servo_led_error_write + 3];

    code = std::clamp<uint8_t>(code, 0, 7);

    buf[0] = buf[1] = SERVO_FRAME_HEADER;
    buf[2] = id;
    buf[3] = s_servo_led_error_write;
    buf[4] = SERVO_LED_ERROR_WRITE;
    buf[5] = code;
    buf[6] = checksum(buf);

    usleep(50 * 1000);

    return writeToServo(buf, sizeof(buf));
}

bool CHiwonderProtocol::getPosition(uint8_t id, int16_t& pos) {
    uint8_t cmd_opcode = SERVO_POS_READ;
    uint8_t rx_buf[s_servo_pos_read + 3];
    bool syntax_check = readCMD(id, cmd_opcode, sizeof(rx_buf), rx_buf);
    if (syntax_check) {
        pos = byteToHW(rx_buf[6], rx_buf[5]);
    }
    return syntax_check;
}

bool CHiwonderProtocol::getVoltage(uint8_t id, uint16_t& vin) {
    uint8_t cmd_opcode = SERVO_VIN_READ;
    uint8_t rx_buf[s_servo_vin_read + 3];
    bool syntax_check = readCMD(id, cmd_opcode, sizeof(rx_buf), rx_buf);
    if (syntax_check) {
        vin = byteToHW(rx_buf[6], rx_buf[5]);
    }
    return syntax_check;
}

bool CHiwonderProtocol::getVoltageLimits(uint8_t id, uint16_t& min_voltage, uint16_t& max_voltage) {
    uint8_t cmd_opcode = SERVO_VIN_LIMITS_READ;
    uint8_t rx_buf[s_servo_vin_limits_read + 3];
    bool syntax_check = readCMD(id, cmd_opcode, sizeof(rx_buf), rx_buf);
    if (syntax_check) {
        min_voltage = byteToHW(rx_buf[6], rx_buf[5]);
        max_voltage = byteToHW(rx_buf[8], rx_buf[7]);
    }
    return syntax_check;
}

bool CHiwonderProtocol::setVoltageLimits(uint8_t id, uint16_t min_voltage, uint16_t max_voltage) {
    uint8_t buf[s_servo_vin_limits_write + 3];

    min_voltage = std::clamp<uint16_t>(min_voltage, 4500, 12000);
    max_voltage = std::clamp<uint16_t>(max_voltage, 4500, 12000);

    buf[0] = buf[1] = SERVO_FRAME_HEADER;
    buf[2] = id;
    buf[3] = s_servo_vin_limits_write;
    buf[4] = SERVO_VIN_LIMITS_WRITE;
    buf[5] = getLowByte(min_voltage);
    buf[6] = getHighByte(min_voltage);
    buf[7] = getLowByte(max_voltage);
    buf[8] = getHighByte(max_voltage);
    buf[9] = checksum(buf);

    return writeToServo(buf, sizeof(buf));
}

bool CHiwonderProtocol::getTemperature(uint8_t id, uint8_t& temp) {
    uint8_t cmd_opcode = SERVO_TEMP_READ;
    uint8_t rx_buf[s_servo_temp_read + 3];
    bool syntax_check = readCMD(id, cmd_opcode, sizeof(rx_buf), rx_buf);
    if (syntax_check) {
        temp = rx_buf[5];
    }
    return syntax_check;
}

bool CHiwonderProtocol::getMaxTemperatureLimit(uint8_t id, uint8_t& max_temperature) {
    uint8_t cmd_opcode = SERVO_TEMP_LIMIT_READ;
    uint8_t rx_buf[s_servo_temp_limit_read + 3];
    bool syntax_check = readCMD(id, cmd_opcode, sizeof(rx_buf), rx_buf);
    if (syntax_check) {
        max_temperature = rx_buf[5];
    }
    return syntax_check;
}

bool CHiwonderProtocol::setMaxTemperatureLimit(uint8_t id, uint8_t max_temperature) {
    uint8_t buf[s_servo_temp_limit_write + 3];

    max_temperature = std::clamp<uint8_t>(max_temperature, 50, 100);

    buf[0] = buf[1] = SERVO_FRAME_HEADER;
    buf[2] = id;
    buf[3] = s_servo_temp_limit_write;
    buf[4] = SERVO_TEMP_LIMIT_WRITE;
    buf[5] = max_temperature;
    buf[6] = checksum(buf);

    return writeToServo(buf, sizeof(buf));
}

bool CHiwonderProtocol::getLedErrcode(uint8_t id, uint8_t& lederrcode) {
    uint8_t cmd_opcode = SERVO_LED_ERROR_READ;
    uint8_t rx_buf[s_servo_led_error_read + 3];
    bool syntax_check = readCMD(id, cmd_opcode, sizeof(rx_buf), rx_buf);
    if (syntax_check) {
        lederrcode = rx_buf[5];
    }
    return syntax_check;
}

bool CHiwonderProtocol::getMode(uint8_t id, uint8_t& mode) {
    uint8_t cmd_opcode = SERVO_OR_MOTOR_MODE_READ;
    uint8_t rx_buf[s_servo_mode_read + 3];
    bool syntax_check = readCMD(id, cmd_opcode, sizeof(rx_buf), rx_buf);
    if (syntax_check) {
        mode = rx_buf[5];
    }
    return syntax_check;
}

bool CHiwonderProtocol::getMotorSpeed([[maybe_unused]] uint8_t id, [[maybe_unused]] int16_t& speed) {
    return false;
}

bool CHiwonderProtocol::setServoMode(uint8_t id) {
    uint8_t buf[s_servo_mode_write + 3];

    buf[0] = buf[1] = SERVO_FRAME_HEADER;
    buf[2] = id;
    buf[3] = s_servo_mode_write;
    buf[4] = SERVO_OR_MOTOR_MODE_WRITE;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
    buf[8] = checksum(buf);

    return writeToServo(buf, sizeof(buf));
}

bool CHiwonderProtocol::setMotorMode(uint8_t id, int16_t speed) {
    uint8_t buf[s_servo_mode_write + 3];

    speed = std::clamp<int16_t>(speed, -1000, 1000);
    if (speed < 0) {
        speed += 65536;
    }

    buf[0] = buf[1] = SERVO_FRAME_HEADER;
    buf[2] = id;
    buf[3] = s_servo_mode_write;
    buf[4] = SERVO_OR_MOTOR_MODE_WRITE;
    buf[5] = 1;
    buf[6] = getLowByte(speed);
    buf[7] = getHighByte(speed);
    buf[8] = checksum(buf);

    return writeToServo(buf, sizeof(buf));
}

bool CHiwonderProtocol::setTorque(uint8_t id, bool active) {
    uint8_t buf[s_servo_torque_write + 3];

    buf[0] = buf[1] = SERVO_FRAME_HEADER;
    buf[2] = id;
    buf[3] = s_servo_torque_write;
    buf[4] = SERVO_TORQUE_WRITE;
    buf[5] = uint8_t(active);
    buf[6] = checksum(buf);

    return writeToServo(buf, sizeof(buf));
}

bool CHiwonderProtocol::isLedOn(uint8_t id) {
    uint8_t cmd_opcode = SERVO_LED_CTRL_READ;
    uint8_t rx_buf[s_servo_led_ctrl_read + 3];
    bool syntax_check = readCMD(id, cmd_opcode, sizeof(rx_buf), rx_buf);
    return syntax_check && (rx_buf[5] == 0);
}

bool CHiwonderProtocol::setLed(uint8_t id, bool on) {
    uint8_t buf[s_servo_led_ctrl_write + 3];

    buf[0] = buf[1] = SERVO_FRAME_HEADER;
    buf[2] = id;
    buf[3] = s_servo_led_ctrl_write;
    buf[4] = SERVO_LED_CTRL_WRITE;
    buf[5] = uint8_t(!on);
    buf[6] = checksum(buf);

    return writeToServo(buf, sizeof(buf));
}

}  // namespace rumblex_movement