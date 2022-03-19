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

template<typename T>
void toLittleEndian(vector<uint8_t> &buffer, T value);

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

    vector<uint8_t> packet = getConfigValueSetPacket(cfg::USB_ENABLED, true, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setPortEnabled(PORT_USB, true);
}

TEST_F(DriverTest, it_disables_a_device_port) {
    IODRIVERS_BASE_MOCK();

    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    vector<uint8_t> packet = getConfigValueSetPacket(cfg::SPI_ENABLED, false, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setPortEnabled(PORT_SPI, false);
}

TEST_F(DriverTest, it_throws_if_valset_is_rejected) {
    IODRIVERS_BASE_MOCK();

    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_NACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    vector<uint8_t> packet = getConfigValueSetPacket(cfg::USB_ENABLED, true, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    ASSERT_THROW(driver.setPortEnabled(PORT_USB, true), ConfigValueSetError);
}

TEST_F(DriverTest, it_throws_if_an_valset_ack_is_not_received) {
    IODRIVERS_BASE_MOCK();

    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_NACK;
    frame.payload.push_back(MSG_CLASS_ACK);
    frame.payload.push_back(MSG_ID_NACK);

    vector<uint8_t> packet = getConfigValueSetPacket(cfg::USB_ENABLED, true, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    ASSERT_THROW(driver.setPortEnabled(PORT_USB, true), iodrivers_base::TimeoutError);
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
    driver.setPortEnabled(PORT_USB, true);
}

TEST_F(DriverTest, it_enables_nmea_output_on_i2c) {
    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    pushDataToDriver(frame.toPacket());
    driver.setPortProtocol(PORT_I2C, DIRECTION_OUTPUT, PROTOCOL_NMEA, true, true);

    ASSERT_EQ(getConfigValueSetPacket(0x10720002, true, true), readDataFromDriver());
}

TEST_F(DriverTest, it_disables_ubx_input_on_usb) {
    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    pushDataToDriver(frame.toPacket());
    driver.setPortProtocol(PORT_USB, DIRECTION_INPUT, PROTOCOL_UBX, false, true);

    ASSERT_EQ(getConfigValueSetPacket(0x10770001, false, true), readDataFromDriver());
}

TEST_F(DriverTest, it_enables_rtcm_input_on_uart1) {
    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    pushDataToDriver(frame.toPacket());
    driver.setPortProtocol(PORT_UART1, DIRECTION_INPUT, PROTOCOL_RTCM3X, true, false);

    ASSERT_EQ(getConfigValueSetPacket(0x10730004, true, false), readDataFromDriver());
}

TEST_F(DriverTest, it_enables_rtcm_output_on_uart2) {
    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    pushDataToDriver(frame.toPacket());
    driver.setPortProtocol(PORT_UART2, DIRECTION_OUTPUT, PROTOCOL_RTCM3X, true, false);

    ASSERT_EQ(getConfigValueSetPacket(0x10760004, true, false), readDataFromDriver());
}

TEST_F(DriverTest, it_enables_nmea_output_on_spi) {
    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    pushDataToDriver(frame.toPacket());
    driver.setPortProtocol(PORT_SPI, DIRECTION_OUTPUT, PROTOCOL_NMEA, true, false);

    ASSERT_EQ(getConfigValueSetPacket(0x107a0002, true, false), readDataFromDriver());
}

TEST_F(DriverTest, it_enables_odometer) {
    IODRIVERS_BASE_MOCK();

    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    vector<uint8_t> packet = getConfigValueSetPacket(cfg::ODO_USE_ODO, true, true);
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

    vector<uint8_t> packet = getConfigValueSetPacket(cfg::ODO_USE_COG, true, true);
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

    vector<uint8_t> packet = getConfigValueSetPacket(cfg::ODO_OUTLPVEL, true, true);
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

    vector<uint8_t> packet = getConfigValueSetPacket(cfg::ODO_OUTLPCOG, true, true);
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

    vector<uint8_t> packet = getConfigValueSetPacket<uint8_t>(cfg::ODO_PROFILE, 3, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setOdometerProfile(ODOM_CAR, true);
}

TEST_F(DriverTest, it_sets_upper_speed_limit_for_heading_filter) {
    IODRIVERS_BASE_MOCK();

    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    vector<uint8_t> packet = getConfigValueSetPacket<uint8_t>(cfg::ODO_COGMAXSPEED, 33, true);
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

    vector<uint8_t> packet = getConfigValueSetPacket<uint8_t>(cfg::ODO_COGMAXPOSACC, 12, true);
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

    vector<uint8_t> packet = getConfigValueSetPacket<uint8_t>(cfg::ODO_VELLPGAIN, 23, true);
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

    vector<uint8_t> packet = getConfigValueSetPacket<uint8_t>(cfg::ODO_COGLPGAIN, 3, true);
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

    vector<uint8_t> packet = getConfigValueSetPacket<uint16_t>(cfg::RATE_MEAS, 100, true);
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

    vector<uint8_t> packet = getConfigValueSetPacket<uint16_t>(cfg::RATE_NAV, 93, true);
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

    vector<uint8_t> packet = getConfigValueSetPacket<uint8_t>(cfg::RATE_TIMEREF, 2, true);
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

    vector<uint8_t> packet = getConfigValueSetPacket<uint8_t>(cfg::NAVSPG_DYNMODEL, 4, true);
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

    vector<uint8_t> packet = getConfigValueSetPacket<uint8_t>(cfg::MOT_GNSSSPEED_THRS, 5, true);
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

    vector<uint8_t> packet = getConfigValueSetPacket<uint16_t>(cfg::MOT_GNSSDIST_THRS, 3, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setStaticHoldDistanceThreshold(3, true);
}

TEST_F(DriverTest, it_requests_and_reads_a_pvt_message) {
    IODRIVERS_BASE_MOCK();
    Frame frame;
    frame.msg_class = MSG_CLASS_NAV;
    frame.msg_id = MSG_ID_PVT;
    frame.payload.resize(92, 0);
    vector<uint8_t> packet = Frame({ MSG_CLASS_NAV, MSG_ID_PVT }).toPacket();
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.readPVT();
}

TEST_F(DriverTest, it_waits_for_a_pvt_message) {
    IODRIVERS_BASE_MOCK();
    Frame frame;
    frame.msg_class = MSG_CLASS_NAV;
    frame.msg_id = MSG_ID_PVT;
    toLittleEndian<uint32_t>(frame.payload, 3600); // time of week
    frame.payload.resize(92, 0);
    pushDataToDriver(frame.toPacket());
    auto pvt = driver.waitForPVT();
    ASSERT_EQ(3600, pvt.time_of_week);
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
    driver.setOutputRate(PORT_I2C, MSGOUT_NAV_PVT, 2, false);

    ASSERT_EQ(getConfigValueSetPacket(0x20910006, (uint8_t)2, false), readDataFromDriver());
}

TEST_F(DriverTest, it_sets_output_rate_of_sig_on_i2c) {
    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    pushDataToDriver(frame.toPacket());
    driver.setOutputRate(PORT_I2C, MSGOUT_NAV_SIG, 5, false);

    ASSERT_EQ(getConfigValueSetPacket(0x20910345, (uint8_t)5, false), readDataFromDriver());
}

TEST_F(DriverTest, it_sets_output_rate_of_rf_on_i2c) {
    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    pushDataToDriver(frame.toPacket());
    driver.setOutputRate(PORT_I2C, MSGOUT_MON_RF, 1, true);

    ASSERT_EQ(getConfigValueSetPacket(0x20910359, (uint8_t)1, true), readDataFromDriver());
}

TEST_F(DriverTest, it_sets_output_rate_of_rf_on_spi) {
    Frame frame;
    frame.msg_class = MSG_CLASS_ACK;
    frame.msg_id = MSG_ID_ACK;
    frame.payload.push_back(MSG_CLASS_CFG);
    frame.payload.push_back(MSG_ID_VALSET);

    pushDataToDriver(frame.toPacket());
    driver.setOutputRate(PORT_SPI, MSGOUT_MON_RF, 1, true);

    ASSERT_EQ(getConfigValueSetPacket(0x2091035d, (uint8_t)1, true), readDataFromDriver());
}

TEST_F(DriverTest, it_parses_RTCM_messages_data_received_between_UBX_frames_but_ignores_them_in_readFrame) {
    const std::vector<uint8_t> rtcm =
    {
        0xd3, 0x00, 0x13, 0x3e, 0xd0, 0x00, 0x02, 0x36,
        0xfd, 0xb8, 0x0d, 0xde, 0x08, 0x00, 0x5b, 0x2b,
        0xc1, 0x08, 0xa7, 0xb9, 0x8d, 0x3d, 0xd8, 0xab, 0x37
    };
    pushDataToDriver(rtcm);

    Frame frame;
    frame.msg_class = MSG_CLASS_NAV;
    frame.msg_id = MSG_ID_SIG;
    frame.payload.resize(8, 0);
    pushDataToDriver(frame.toPacket());

    driver.readFrame();
    ASSERT_EQ(0, driver.getStats().bad_rx);
}

TEST_F(DriverTest, poll_calls_back_for_RTCM_data) {
    struct Callbacks : Driver::PollCallbacks {
        std::vector<uint8_t> data;

        void rtcm(uint8_t const* buffer, size_t size) {
            data.insert(data.end(), buffer, buffer + size);
        }
    };

    const std::vector<uint8_t> expected =
    {
        0xd3, 0x00, 0x13, 0x3e, 0xd0, 0x00, 0x02, 0x36,
        0xfd, 0xb8, 0x0d, 0xde, 0x08, 0x00, 0x5b, 0x2b,
        0xc1, 0x08, 0xa7, 0xb9, 0x8d, 0x3d, 0xd8, 0xab, 0x37
    };
    pushDataToDriver(expected);

    Callbacks callbacks;
    driver.poll(callbacks);
    ASSERT_EQ(expected, callbacks.data);
}

TEST_F(DriverTest, poll_calls_back_for_PVT_data) {
    struct Callbacks : Driver::PollCallbacks {
        PVT data;

        void pvt(PVT const& pvt) {
            data = pvt;
        }
    };

    IODRIVERS_BASE_MOCK();
    Frame frame;
    frame.msg_class = MSG_CLASS_NAV;
    frame.msg_id = MSG_ID_PVT;
    toLittleEndian<uint32_t>(frame.payload, 3600); // time of week
    frame.payload.resize(92, 0);
    pushDataToDriver(frame.toPacket());

    Callbacks callbacks;
    driver.poll(callbacks);

    ASSERT_EQ(3600, callbacks.data.time_of_week);
}

TEST_F(DriverTest, poll_calls_back_for_satellite_info_data) {
    struct Callbacks : Driver::PollCallbacks {
        SatelliteInfo data;

        void satelliteInfo(SatelliteInfo const& info) {
            data = info;
        }
    };

    IODRIVERS_BASE_MOCK();
    Frame frame;
    frame.msg_class = MSG_CLASS_NAV;
    frame.msg_id = MSG_ID_SAT;
    frame.payload.resize(8 + (12 * 2));
    frame.payload[5] = 2;
    frame.payload[8] = 25;
    pushDataToDriver(frame.toPacket());

    Callbacks callbacks;
    driver.poll(callbacks);

    ASSERT_EQ(2, callbacks.data.signals.size());
    ASSERT_EQ(25, callbacks.data.signals[0].gnss_id);
}

TEST_F(DriverTest, poll_calls_back_for_rf_info_data) {
    struct Callbacks : Driver::PollCallbacks {
        RFInfo data;

        void rfInfo(RFInfo const& info) {
            data = info;
        }
    };

    IODRIVERS_BASE_MOCK();
    Frame frame;
    frame.msg_class = MSG_CLASS_MON;
    frame.msg_id = MSG_ID_RF;
    frame.payload.resize(4 + (24 * 2));
    frame.payload[1] = 2;
    frame.payload[4] = 25;
    pushDataToDriver(frame.toPacket());

    Callbacks callbacks;
    driver.poll(callbacks);

    ASSERT_EQ(2, callbacks.data.blocks.size());
    ASSERT_EQ(25, callbacks.data.blocks[0].block_id);
}

template<typename T>
void toLittleEndian(vector<uint8_t> &buffer, T value)
{
    uint8_t shifter = 0;
    T bytes = reinterpret_cast<const T&>(value);
    for (size_t i = 0; i < sizeof(T); i++) {
        buffer.push_back((bytes >> shifter) & 0xFF);
        shifter += 8;
    }
}
