#include <vector>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <base/Time.hpp>
#include <gps_ublox/protocols/UBX.hpp>
#include <gps_ublox/Driver.hpp>
#include <gps_ublox/BoardInfo.hpp>
#include <iodrivers_base/FixtureGTest.hpp>

using namespace std;
using namespace gps_ublox;
using namespace protocols;

struct DriverTest : public ::testing::Test, iodrivers_base::Fixture<Driver> {
    vector<uint8_t> buffer;
    DriverTest() {
        buffer.resize(256, 0);
        driver.openURI("test://");
        driver.setReadTimeout(base::Time::fromMilliseconds(100));
    }
    UBX parser;
};

TEST_F(DriverTest, it_enables_a_device_port) {
    IODRIVERS_BASE_MOCK();

    UBX::Frame frame;
    frame.msg_class = UBX::MSG_CLASS_ACK;
    frame.msg_id = UBX::MSG_ID_ACK;
    frame.payload.push_back(UBX::MSG_CLASS_CFG);
    frame.payload.push_back(UBX::MSG_ID_VALSET);

    vector<uint8_t> packet = parser.getConfigValueSetPacket(UBX::USB_ENABLED, true, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setPortEnabled(Driver::PORT_USB, true);
}

TEST_F(DriverTest, it_disables_a_device_port) {
    IODRIVERS_BASE_MOCK();

    UBX::Frame frame;
    frame.msg_class = UBX::MSG_CLASS_ACK;
    frame.msg_id = UBX::MSG_ID_ACK;
    frame.payload.push_back(UBX::MSG_CLASS_CFG);
    frame.payload.push_back(UBX::MSG_ID_VALSET);

    vector<uint8_t> packet = parser.getConfigValueSetPacket(UBX::SPI_ENABLED, false, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setPortEnabled(Driver::PORT_SPI, false);
}

TEST_F(DriverTest, it_throws_if_valset_is_rejected) {
    IODRIVERS_BASE_MOCK();

    UBX::Frame frame;
    frame.msg_class = UBX::MSG_CLASS_ACK;
    frame.msg_id = UBX::MSG_ID_NACK;
    frame.payload.push_back(UBX::MSG_CLASS_CFG);
    frame.payload.push_back(UBX::MSG_ID_VALSET);

    vector<uint8_t> packet = parser.getConfigValueSetPacket(UBX::USB_ENABLED, true, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    ASSERT_THROW(driver.setPortEnabled(Driver::PORT_USB, true), ConfigValueSetError);
}

TEST_F(DriverTest, it_throws_if_an_valset_ack_is_not_received) {
    IODRIVERS_BASE_MOCK();

    UBX::Frame frame;
    frame.msg_class = UBX::MSG_CLASS_ACK;
    frame.msg_id = UBX::MSG_ID_NACK;
    frame.payload.push_back(UBX::MSG_CLASS_ACK);
    frame.payload.push_back(UBX::MSG_ID_NACK);

    vector<uint8_t> packet = parser.getConfigValueSetPacket(UBX::USB_ENABLED, true, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    ASSERT_THROW(driver.setPortEnabled(Driver::PORT_USB, true), iodrivers_base::TimeoutError);
}

TEST_F(DriverTest, it_keeps_reading_until_an_ack_is_received) {
    UBX::Frame frame;
    frame.msg_class = UBX::MSG_CLASS_ACK;
    frame.msg_id = UBX::MSG_ID_ACK;
    frame.payload.push_back(UBX::MSG_CLASS_ACK);
    frame.payload.push_back(UBX::MSG_ID_NACK);
    pushDataToDriver(frame.toPacket());
    pushDataToDriver(frame.toPacket());
    pushDataToDriver(frame.toPacket());
    frame.payload.clear();
    frame.payload.push_back(UBX::MSG_CLASS_CFG);
    frame.payload.push_back(UBX::MSG_ID_VALSET);
    pushDataToDriver(frame.toPacket());
    driver.setPortEnabled(Driver::PORT_USB, true);
}

TEST_F(DriverTest, it_enables_an_output_protocol_on_a_given_port) {
    IODRIVERS_BASE_MOCK();

    UBX::Frame frame;
    frame.msg_class = UBX::MSG_CLASS_ACK;
    frame.msg_id = UBX::MSG_ID_ACK;
    frame.payload.push_back(UBX::MSG_CLASS_CFG);
    frame.payload.push_back(UBX::MSG_ID_VALSET);

    vector<uint8_t> packet = parser.getConfigValueSetPacket(UBX::UART1_OUT_UBX, true, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setOutputProtocol(Driver::PORT_UART1, Driver::PROTOCOL_UBX, true);
}

TEST_F(DriverTest, it_disables_an_output_protocol_on_a_given_port) {
    IODRIVERS_BASE_MOCK();

    UBX::Frame frame;
    frame.msg_class = UBX::MSG_CLASS_ACK;
    frame.msg_id = UBX::MSG_ID_ACK;
    frame.payload.push_back(UBX::MSG_CLASS_CFG);
    frame.payload.push_back(UBX::MSG_ID_VALSET);

    vector<uint8_t> packet = parser.getConfigValueSetPacket(UBX::UART2_OUT_UBX, false, false);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setOutputProtocol(Driver::PORT_UART2, Driver::PROTOCOL_UBX, false, false);
}

TEST_F(DriverTest, it_enables_odometer) {
    IODRIVERS_BASE_MOCK();

    UBX::Frame frame;
    frame.msg_class = UBX::MSG_CLASS_ACK;
    frame.msg_id = UBX::MSG_ID_ACK;
    frame.payload.push_back(UBX::MSG_CLASS_CFG);
    frame.payload.push_back(UBX::MSG_ID_VALSET);

    vector<uint8_t> packet = parser.getConfigValueSetPacket(UBX::ODO_USE_ODO, true, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setOdometer(true);
}

TEST_F(DriverTest, it_enables_heading_filter) {
    IODRIVERS_BASE_MOCK();

    UBX::Frame frame;
    frame.msg_class = UBX::MSG_CLASS_ACK;
    frame.msg_id = UBX::MSG_ID_ACK;
    frame.payload.push_back(UBX::MSG_CLASS_CFG);
    frame.payload.push_back(UBX::MSG_ID_VALSET);

    vector<uint8_t> packet = parser.getConfigValueSetPacket(UBX::ODO_USE_COG, true, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setLowSpeedCourseOverGroundFilter(true);
}

TEST_F(DriverTest, it_enables_lowpass_filtered_velocity_output) {
    IODRIVERS_BASE_MOCK();

    UBX::Frame frame;
    frame.msg_class = UBX::MSG_CLASS_ACK;
    frame.msg_id = UBX::MSG_ID_ACK;
    frame.payload.push_back(UBX::MSG_CLASS_CFG);
    frame.payload.push_back(UBX::MSG_ID_VALSET);

    vector<uint8_t> packet = parser.getConfigValueSetPacket(UBX::ODO_OUTLPVEL, true, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setOutputLowPassFilteredVelocity(true);
}

TEST_F(DriverTest, it_enables_lowpass_filtered_heading_output) {
    IODRIVERS_BASE_MOCK();

    UBX::Frame frame;
    frame.msg_class = UBX::MSG_CLASS_ACK;
    frame.msg_id = UBX::MSG_ID_ACK;
    frame.payload.push_back(UBX::MSG_CLASS_CFG);
    frame.payload.push_back(UBX::MSG_ID_VALSET);

    vector<uint8_t> packet = parser.getConfigValueSetPacket(UBX::ODO_OUTLPCOG, true, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setOutputLowPassFilteredHeading(true);
}

TEST_F(DriverTest, it_sets_odometer_profile) {
    IODRIVERS_BASE_MOCK();

    UBX::Frame frame;
    frame.msg_class = UBX::MSG_CLASS_ACK;
    frame.msg_id = UBX::MSG_ID_ACK;
    frame.payload.push_back(UBX::MSG_CLASS_CFG);
    frame.payload.push_back(UBX::MSG_ID_VALSET);

    vector<uint8_t> packet = parser.getConfigValueSetPacket<uint8_t>(UBX::ODO_PROFILE, 3, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setOdometerProfile(Driver::ODOM_CAR, true);
}

TEST_F(DriverTest, it_sets_upper_speed_limit_for_heading_filter) {
    IODRIVERS_BASE_MOCK();

    UBX::Frame frame;
    frame.msg_class = UBX::MSG_CLASS_ACK;
    frame.msg_id = UBX::MSG_ID_ACK;
    frame.payload.push_back(UBX::MSG_CLASS_CFG);
    frame.payload.push_back(UBX::MSG_ID_VALSET);

    vector<uint8_t> packet = parser.getConfigValueSetPacket<uint8_t>(UBX::ODO_COGMAXSPEED, 33, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setUpperSpeedLimitForHeadingFilter(33, true);
}

TEST_F(DriverTest, it_sets_max_position_accuracy_for_heading_filter) {
    IODRIVERS_BASE_MOCK();

    UBX::Frame frame;
    frame.msg_class = UBX::MSG_CLASS_ACK;
    frame.msg_id = UBX::MSG_ID_ACK;
    frame.payload.push_back(UBX::MSG_CLASS_CFG);
    frame.payload.push_back(UBX::MSG_ID_VALSET);

    vector<uint8_t> packet = parser.getConfigValueSetPacket<uint8_t>(UBX::ODO_COGMAXPOSACC, 12, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setMaxPositionAccuracyForLowSpeedHeadingFilter(12, true);
}

TEST_F(DriverTest, it_sets_velocity_lowpass_filter_level) {
    IODRIVERS_BASE_MOCK();

    UBX::Frame frame;
    frame.msg_class = UBX::MSG_CLASS_ACK;
    frame.msg_id = UBX::MSG_ID_ACK;
    frame.payload.push_back(UBX::MSG_CLASS_CFG);
    frame.payload.push_back(UBX::MSG_ID_VALSET);

    vector<uint8_t> packet = parser.getConfigValueSetPacket<uint8_t>(UBX::ODO_VELLPGAIN, 23, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setVelocityLowPassFilterLevel(23, true);
}

TEST_F(DriverTest, it_sets_heading_lowpass_filter_level) {
    IODRIVERS_BASE_MOCK();

    UBX::Frame frame;
    frame.msg_class = UBX::MSG_CLASS_ACK;
    frame.msg_id = UBX::MSG_ID_ACK;
    frame.payload.push_back(UBX::MSG_CLASS_CFG);
    frame.payload.push_back(UBX::MSG_ID_VALSET);

    vector<uint8_t> packet = parser.getConfigValueSetPacket<uint8_t>(UBX::ODO_COGLPGAIN, 3, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setHeadingLowPassFilterLevel(3, true);
}

TEST_F(DriverTest, it_requests_device_info) {
    IODRIVERS_BASE_MOCK();
    UBX::Frame frame;
    frame.msg_class = UBX::MSG_CLASS_MON;
    frame.msg_id = UBX::MSG_ID_VER;
    frame.payload.resize(100, 0);
    vector<uint8_t> packet = UBX::Frame({ UBX::MSG_CLASS_MON, UBX::MSG_ID_VER }).toPacket();
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    BoardInfo info = driver.readBoardInfo();
    ASSERT_EQ("", info.software_version);
    ASSERT_EQ("", info.software_version);
    ASSERT_EQ(2, info.extensions.size());
}

TEST_F(DriverTest, it_throws_if_version_response_is_invalid) {
    IODRIVERS_BASE_MOCK();
    UBX::Frame frame;
    frame.msg_class = UBX::MSG_CLASS_MON;
    frame.msg_id = UBX::MSG_ID_VALSET;
    vector<uint8_t> packet = UBX::Frame({ UBX::MSG_CLASS_MON, UBX::MSG_ID_VER }).toPacket();
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    ASSERT_THROW(driver.readBoardInfo(), iodrivers_base::TimeoutError);
}
