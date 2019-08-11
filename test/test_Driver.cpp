#include <vector>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <base/Time.hpp>
#include <gps_ublox/protocols/UBX.hpp>
#include <gps_ublox/Driver.hpp>
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
    frame.msg_class = UBX::UBX_ACK;
    frame.msg_id = UBX::ACK;
    frame.payload.push_back(UBX::UBX_CFG);
    frame.payload.push_back(UBX::VALSET);

    vector<uint8_t> packet = parser.getCfgValSetPacket(UBX::USB_ENABLED, true, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setPortEnabled(Driver::PORT_USB, true);
}

TEST_F(DriverTest, it_disables_a_device_port) {
    IODRIVERS_BASE_MOCK();

    UBX::Frame frame;
    frame.msg_class = UBX::UBX_ACK;
    frame.msg_id = UBX::ACK;
    frame.payload.push_back(UBX::UBX_CFG);
    frame.payload.push_back(UBX::VALSET);

    vector<uint8_t> packet = parser.getCfgValSetPacket(UBX::SPI_ENABLED, false, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setPortEnabled(Driver::PORT_SPI, false);
}

TEST_F(DriverTest, it_throws_if_valset_is_rejected) {
    IODRIVERS_BASE_MOCK();

    UBX::Frame frame;
    frame.msg_class = UBX::UBX_ACK;
    frame.msg_id = UBX::NACK;
    frame.payload.push_back(UBX::UBX_CFG);
    frame.payload.push_back(UBX::VALSET);

    vector<uint8_t> packet = parser.getCfgValSetPacket(UBX::USB_ENABLED, true, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    ASSERT_THROW(driver.setPortEnabled(Driver::PORT_USB, true), ConfigValSetFailed);
}

TEST_F(DriverTest, it_throws_if_an_valset_ack_is_not_received) {
    IODRIVERS_BASE_MOCK();

    UBX::Frame frame;
    frame.msg_class = UBX::UBX_ACK;
    frame.msg_id = UBX::NACK;
    frame.payload.push_back(UBX::UBX_ACK);
    frame.payload.push_back(UBX::NACK);

    vector<uint8_t> packet = parser.getCfgValSetPacket(UBX::USB_ENABLED, true, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    ASSERT_THROW(driver.setPortEnabled(Driver::PORT_USB, true), iodrivers_base::TimeoutError);
}

TEST_F(DriverTest, it_keeps_reading_until_an_ack_is_received) {
    UBX::Frame frame;
    frame.msg_class = UBX::UBX_ACK;
    frame.msg_id = UBX::ACK;
    frame.payload.push_back(UBX::UBX_ACK);
    frame.payload.push_back(UBX::NACK);
    pushDataToDriver(frame.toPacket());
    pushDataToDriver(frame.toPacket());
    pushDataToDriver(frame.toPacket());
    frame.payload.clear();
    frame.payload.push_back(UBX::UBX_CFG);
    frame.payload.push_back(UBX::VALSET);
    pushDataToDriver(frame.toPacket());
    driver.setPortEnabled(Driver::PORT_USB, true);
}

TEST_F(DriverTest, it_enables_an_output_protocol_on_a_given_port) {
    IODRIVERS_BASE_MOCK();

    UBX::Frame frame;
    frame.msg_class = UBX::UBX_ACK;
    frame.msg_id = UBX::ACK;
    frame.payload.push_back(UBX::UBX_CFG);
    frame.payload.push_back(UBX::VALSET);

    vector<uint8_t> packet = parser.getCfgValSetPacket(UBX::UART1_OUTPROT_UBX, true, true);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setPortOutProt(Driver::PORT_UART1, Driver::PROTOCOL_UBX, true);
}

TEST_F(DriverTest, it_disables_an_output_protocol_on_a_given_port) {
    IODRIVERS_BASE_MOCK();

    UBX::Frame frame;
    frame.msg_class = UBX::UBX_ACK;
    frame.msg_id = UBX::ACK;
    frame.payload.push_back(UBX::UBX_CFG);
    frame.payload.push_back(UBX::VALSET);

    vector<uint8_t> packet = parser.getCfgValSetPacket(UBX::UART2_OUTPROT_UBX, false, false);
    vector<uint8_t> reply = frame.toPacket();
    EXPECT_REPLY(packet, reply);
    driver.setPortOutProt(Driver::PORT_UART2, Driver::PROTOCOL_UBX, false, false);
}
