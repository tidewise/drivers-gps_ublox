#include <vector>
#include <stdexcept>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <base/Time.hpp>
#include <gps_ublox/UBX.hpp>
#include <gps_ublox/Driver.hpp>
#include <gps_ublox/BoardInfo.hpp>
#include <gps_ublox/RFInfo.hpp>
#include <iodrivers_base/FixtureGTest.hpp>

using namespace std;
using namespace gps_ublox;
using namespace UBX;

struct DriverTest : public ::testing::Test, iodrivers_base::Fixture<Driver> {
    vector<uint8_t> buffer;
    DriverTest() {
        buffer.resize(256, 0);
        driver.openURI("test://");
        driver.setReadTimeout(base::Time::fromMilliseconds(100));
    }
};

TEST_F(DriverTest, it_enables_a_device_port) {
    IODRIVERS_BASE_MOCK();

    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    vector<uint8_t> packet = getConfigValueSetPacket(USB_ENABLED, true, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setPortEnabled(Driver::PORT_USB, true);
}

TEST_F(DriverTest, it_disables_a_device_port) {
    IODRIVERS_BASE_MOCK();

    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    vector<uint8_t> packet = getConfigValueSetPacket(SPI_ENABLED, false, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setPortEnabled(Driver::PORT_SPI, false);
}

TEST_F(DriverTest, it_throws_if_valset_is_rejected) {
    IODRIVERS_BASE_MOCK();

    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_NACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    vector<uint8_t> packet = getConfigValueSetPacket(USB_ENABLED, true, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    ASSERT_THROW(driver.setPortEnabled(Driver::PORT_USB, true), ConfigValueSetError);
}

TEST_F(DriverTest, it_throws_if_an_valset_ack_is_not_received) {
    IODRIVERS_BASE_MOCK();

    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_NACK;
    frame.payload.push_back(MSG_CLASS_ACK);
    frame.payload.push_back(MSG_ID_NACK);

    vector<uint8_t> packet = getConfigValueSetPacket(USB_ENABLED, true, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    ASSERT_THROW(driver.setPortEnabled(Driver::PORT_USB, true), iodrivers_base::TimeoutError);
}

TEST_F(DriverTest, it_keeps_reading_until_an_ack_is_received) {
    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_ACK);
    frame.payload.push_back(MSG_ID_NACK);
    pushDataToDriver(frame.toPacket());
    pushDataToDriver(frame.toPacket());
    pushDataToDriver(frame.toPacket());
    frame.payload.clear();
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);
    pushDataToDriver(frame.toPacket());
    driver.setPortEnabled(Driver::PORT_USB, true);
}

TEST_F(DriverTest, it_enables_nmea_output_on_i2c) {
    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    pushDataToDriver(frame.toPacket());
    driver.setPortProtocol(
        Driver::PORT_I2C,
        Driver::DIRECTION_OUTPUT,
        Driver::PROTOCOL_NMEA,
        true,
        true);

    ASSERT_EQ(getConfigValueSetPacket(0x10720002, true, true), readDataFromDriver());
}

TEST_F(DriverTest, it_disables_ubx_input_on_usb) {
    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    pushDataToDriver(frame.toPacket());
    driver.setPortProtocol(
        Driver::PORT_USB,
        Driver::DIRECTION_INPUT,
        Driver::PROTOCOL_UBX,
        false,
        true);

    ASSERT_EQ(getConfigValueSetPacket(0x10770001, false, true), readDataFromDriver());
}

TEST_F(DriverTest, it_enables_rtcm_input_on_uart1) {
    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    pushDataToDriver(frame.toPacket());
    driver.setPortProtocol(
        Driver::PORT_UART1,
        Driver::DIRECTION_INPUT,
        Driver::PROTOCOL_RTCM3X,
        true,
        false);

    ASSERT_EQ(getConfigValueSetPacket(0x10730004, true, false), readDataFromDriver());
}

TEST_F(DriverTest, it_enables_rtcm_output_on_uart2) {
    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    pushDataToDriver(frame.toPacket());
    driver.setPortProtocol(
        Driver::PORT_UART2,
        Driver::DIRECTION_OUTPUT,
        Driver::PROTOCOL_RTCM3X,
        true,
        false);

    ASSERT_EQ(getConfigValueSetPacket(0x10760004, true, false), readDataFromDriver());
}

TEST_F(DriverTest, it_enables_nmea_output_on_spi) {
    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    pushDataToDriver(frame.toPacket());
    driver.setPortProtocol(
        Driver::PORT_SPI,
        Driver::DIRECTION_OUTPUT,
        Driver::PROTOCOL_NMEA,
        true,
        false);

    ASSERT_EQ(getConfigValueSetPacket(0x107a0002, true, false), readDataFromDriver());
}

TEST_F(DriverTest, it_enables_odometer) {
    IODRIVERS_BASE_MOCK();

    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    vector<uint8_t> packet = getConfigValueSetPacket(ODO_USE_ODO, true, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setOdometer(true);
}

TEST_F(DriverTest, it_enables_heading_filter) {
    IODRIVERS_BASE_MOCK();

    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    vector<uint8_t> packet = getConfigValueSetPacket(ODO_USE_COG, true, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setLowSpeedCourseOverGroundFilter(true);
}

TEST_F(DriverTest, it_enables_lowpass_filtered_velocity_output) {
    IODRIVERS_BASE_MOCK();

    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    vector<uint8_t> packet = getConfigValueSetPacket(ODO_OUTLPVEL, true, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setOutputLowPassFilteredVelocity(true);
}

TEST_F(DriverTest, it_enables_lowpass_filtered_heading_output) {
    IODRIVERS_BASE_MOCK();

    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    vector<uint8_t> packet = getConfigValueSetPacket(ODO_OUTLPCOG, true, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setOutputLowPassFilteredHeading(true);
}

TEST_F(DriverTest, it_sets_odometer_profile) {
    IODRIVERS_BASE_MOCK();

    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    vector<uint8_t> packet = getConfigValueSetPacket<uint8_t>(ODO_PROFILE, 3, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setOdometerProfile(Driver::ODOM_CAR, true);
}

TEST_F(DriverTest, it_sets_upper_speed_limit_for_heading_filter) {
    IODRIVERS_BASE_MOCK();

    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    vector<uint8_t> packet = getConfigValueSetPacket<uint8_t>(ODO_COGMAXSPEED, 33, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setUpperSpeedLimitForHeadingFilter(33, true);
}

TEST_F(DriverTest, it_sets_max_position_accuracy_for_heading_filter) {
    IODRIVERS_BASE_MOCK();

    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    vector<uint8_t> packet = getConfigValueSetPacket<uint8_t>(ODO_COGMAXPOSACC, 12, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setMaxPositionAccuracyForLowSpeedHeadingFilter(12, true);
}

TEST_F(DriverTest, it_sets_velocity_lowpass_filter_level) {
    IODRIVERS_BASE_MOCK();

    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    vector<uint8_t> packet = getConfigValueSetPacket<uint8_t>(ODO_VELLPGAIN, 23, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setVelocityLowPassFilterLevel(23, true);
}

TEST_F(DriverTest, it_sets_heading_lowpass_filter_level) {
    IODRIVERS_BASE_MOCK();

    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    vector<uint8_t> packet = getConfigValueSetPacket<uint8_t>(ODO_COGLPGAIN, 3, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setHeadingLowPassFilterLevel(3, true);
}

TEST_F(DriverTest, it_requests_device_info) {
    IODRIVERS_BASE_MOCK();
    Frame frame;
    frame.msg_class = MSG_CLASS_MON;
    frame.msg_id = MSG_ID_VER;
    frame.payload.resize(100, 0);
    vector<uint8_t> packet = Frame({ MSG_CLASS_MON, MSG_ID_VER }).toPacket();
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    BoardInfo info = driver.readBoardInfo();
    ASSERT_EQ("", info.software_version);
    ASSERT_EQ("", info.software_version);
    ASSERT_EQ(2, info.extensions.size());
}

TEST_F(DriverTest, it_throws_if_version_response_is_invalid) {
    IODRIVERS_BASE_MOCK();
    Frame frame;
    frame.msg_class = MSG_CLASS_MON;
    frame.msg_id = MSG_ID_VALSET;
    vector<uint8_t> packet = Frame({ MSG_CLASS_MON, MSG_ID_VER }).toPacket();
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    ASSERT_THROW(driver.readBoardInfo(), iodrivers_base::TimeoutError);
}

TEST_F(DriverTest, it_sets_position_measurement_period) {
    IODRIVERS_BASE_MOCK();

    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    vector<uint8_t> packet = getConfigValueSetPacket<uint16_t>(RATE_MEAS, 100, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setPositionMeasurementPeriod(100, true);
}

TEST_F(DriverTest, it_throws_if_measurements_per_solution_number_too_high) {
    ASSERT_THROW(driver.setMeasurementsPerSolutionRatio(200, true), std::invalid_argument);
}

TEST_F(DriverTest, it_sets_number_of_measurments_per_solution) {
    IODRIVERS_BASE_MOCK();

    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    vector<uint8_t> packet = getConfigValueSetPacket<uint16_t>(RATE_NAV, 93, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setMeasurementsPerSolutionRatio(93, true);
}

TEST_F(DriverTest, it_sets_the_time_system_used_to_aligh_measurements) {
    IODRIVERS_BASE_MOCK();

    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    vector<uint8_t> packet = getConfigValueSetPacket<uint8_t>(RATE_TIMEREF, 2, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setTimeSystem(UBX::GLONASS, true);
}

TEST_F(DriverTest, it_sets_the_dynamic_platform_model) {
    IODRIVERS_BASE_MOCK();

    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    vector<uint8_t> packet = getConfigValueSetPacket<uint8_t>(NAVSPG_DYNMODEL, 4, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setDynamicModel(UBX::AUTOMOTIVE, true);
}

TEST_F(DriverTest, it_sets_the_speed_threshold) {
    IODRIVERS_BASE_MOCK();

    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    vector<uint8_t> packet = getConfigValueSetPacket<uint8_t>(MOT_GNSSSPEED_THRS, 5, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setSpeedThreshold(5, true);
}

TEST_F(DriverTest, it_sets_the_distance_threshold) {
    IODRIVERS_BASE_MOCK();

    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    vector<uint8_t> packet = getConfigValueSetPacket<uint16_t>(MOT_GNSSDIST_THRS, 3, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setStaticHoldDistanceThreshold(3, true);
}

TEST_F(DriverTest, it_requests_gps_data) {
    IODRIVERS_BASE_MOCK();
    Frame frame;
    frame.msg_class = MSG_CLASS_NAV;
    frame.msg_id = MSG_ID_PVT;
    frame.payload.resize(92, 0);
    vector<uint8_t> packet = Frame({ MSG_CLASS_NAV, MSG_ID_PVT }).toPacket();
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.readGPSData();
}

TEST_F(DriverTest, it_requests_rf_info) {
    IODRIVERS_BASE_MOCK();
    Frame frame;
    frame.msg_class = MSG_CLASS_MON;
    frame.msg_id = MSG_ID_RF;
    frame.payload.resize(4, 0);
    vector<uint8_t> packet = Frame({ MSG_CLASS_MON, MSG_ID_RF }).toPacket();
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.readRFInfo();
}

TEST_F(DriverTest, it_requests_sig_info) {
    IODRIVERS_BASE_MOCK();
    Frame frame;
    frame.msg_class = MSG_CLASS_NAV;
    frame.msg_id = MSG_ID_SIG;
    frame.payload.resize(8, 0);
    vector<uint8_t> packet = Frame({ MSG_CLASS_NAV, MSG_ID_SIG }).toPacket();
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.readSignalInfo();
}

TEST_F(DriverTest, it_requests_sat_info) {
    IODRIVERS_BASE_MOCK();
    Frame frame;
    frame.msg_class = MSG_CLASS_NAV;
    frame.msg_id = MSG_ID_SAT;
    frame.payload.resize(8, 0);
    vector<uint8_t> packet = Frame({ MSG_CLASS_NAV, MSG_ID_SAT }).toPacket();
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.readSatelliteInfo();
}

TEST_F(DriverTest, it_reads_any_pending_frame) {
    Frame frame;
    frame.msg_class = MSG_CLASS_NAV;
    frame.msg_id = MSG_ID_SIG;
    frame.payload.resize(8, 0);
    pushDataToDriver(frame.toPacket());

    Frame in_frame = driver.readFrame();
    ASSERT_EQ(frame.msg_class, in_frame.msg_class);
    ASSERT_EQ(frame.msg_id, in_frame.msg_id);
    ASSERT_EQ(frame.payload, in_frame.payload);
}

TEST_F(DriverTest, it_sets_output_rate_of_pvt_on_i2c) {
    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    pushDataToDriver(frame.toPacket());
    driver.setOutputRate(
        Driver::PORT_I2C,
        Driver::MSGOUT_NAV_PVT,
        2,
        false);

    ASSERT_EQ(getConfigValueSetPacket(0x20910006, (uint8_t)2, false), readDataFromDriver());
}

TEST_F(DriverTest, it_sets_output_rate_of_sig_on_i2c) {
    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    pushDataToDriver(frame.toPacket());
    driver.setOutputRate(
        Driver::PORT_I2C,
        Driver::MSGOUT_NAV_SIG,
        5,
        false);

    ASSERT_EQ(getConfigValueSetPacket(0x20910345, (uint8_t)5, false), readDataFromDriver());
}

TEST_F(DriverTest, it_sets_output_rate_of_rf_on_i2c) {
    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    pushDataToDriver(frame.toPacket());
    driver.setOutputRate(
        Driver::PORT_I2C,
        Driver::MSGOUT_MON_RF,
        1,
        true);

    ASSERT_EQ(getConfigValueSetPacket(0x20910359, (uint8_t)1, true), readDataFromDriver());
}

TEST_F(DriverTest, it_sets_output_rate_of_rf_on_spi) {
    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    pushDataToDriver(frame.toPacket());
    driver.setOutputRate(
        Driver::PORT_SPI,
        Driver::MSGOUT_MON_RF,
        1,
        true);

    ASSERT_EQ(getConfigValueSetPacket(0x2091035d, (uint8_t)1, true), readDataFromDriver());
}