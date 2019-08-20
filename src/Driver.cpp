#include <gps_ublox/Driver.hpp>
#include <base/Time.hpp>
#include <iostream>
#include <map>
#include <stdexcept>

using namespace std;
using namespace gps_ublox;
using namespace UBX;

Driver::Driver() : iodrivers_base::Driver(BUFFER_SIZE)
{
}

int Driver::extractPacket(const uint8_t *buffer, size_t buffer_size) const
{
    return UBX::extractPacket(buffer, buffer_size);
}

BoardInfo Driver::readBoardInfo() {
    Frame frame = pollFrame(MSG_CLASS_MON, MSG_ID_VER);

    BoardInfo info;
    info.software_version = string(reinterpret_cast<char*>(&frame.payload[0]));
    info.hardware_version = string(reinterpret_cast<char*>(&frame.payload[30]));

    for (size_t i = 40; i < frame.payload.size(); i += 30) {
        info.extensions.push_back(string(reinterpret_cast<char*>(&frame.payload[i])));
    }
    return info;
}

Frame Driver::pollFrame(uint8_t class_id, uint8_t msg_id)
{
    Frame frame = {
        .msg_class = class_id,
        .msg_id    = msg_id
    };
    vector<uint8_t> packet = frame.toPacket();
    writePacket(&packet[0], packet.size());
    return waitForFrame(class_id, msg_id);
}

Frame Driver::waitForPacket(const uint8_t *class_id, const uint8_t *msg_id,
                            const std::vector<uint8_t> *payload)
{
    base::Time deadline = base::Time::now() + getReadTimeout();
    while (base::Time::now() < deadline) {
        base::Time remaining = deadline - base::Time::now();
        int bytes = readPacket(mReadBuffer, BUFFER_SIZE, remaining);

        Frame frame = Frame::fromPacket(mReadBuffer, bytes);
        if (class_id && *class_id != frame.msg_class) continue;
        if (msg_id && *msg_id != frame.msg_id) continue;
        if (payload && *payload != frame.payload) continue;
        return frame;
    }

    throw iodrivers_base::TimeoutError(
        iodrivers_base::TimeoutError::PACKET,
        "Did not receive the expected packet within the timeout");
}

Frame Driver::waitForFrame(uint8_t class_id, uint8_t msg_id)
{
    return waitForPacket(&class_id, &msg_id);
}

bool Driver::waitForAck(uint8_t class_id, uint8_t msg_id)
{
    uint8_t ack_class = MSG_CLASS_ACK;
    vector<uint8_t> payload = { class_id, msg_id };

    return waitForPacket(&ack_class, nullptr, &payload).msg_id == MSG_ID_ACK;
}

template<typename T>
void Driver::setConfigKeyValue(ConfigKeyId key_id, T value, bool persist)
{
    vector<uint8_t> packet = getConfigValueSetPacket<T>(key_id, value, persist);
    writePacket(&packet[0], packet.size());

    if (!waitForAck(MSG_CLASS_CFG, MSG_ID_VALSET)) {
        throw ConfigValueSetError("Configuration rejected by the device");
    }
}

template void Driver::setConfigKeyValue<bool>(ConfigKeyId, bool, bool);
template void Driver::setConfigKeyValue<uint8_t>(ConfigKeyId, uint8_t, bool);
template void Driver::setConfigKeyValue<uint16_t>(ConfigKeyId, uint16_t, bool);

void Driver::setPortEnabled(DevicePort port, bool state, bool persist)
{
    ConfigKeyId key_id;
    switch (port) {
        case PORT_I2C: key_id = I2C_ENABLED; break;
        case PORT_SPI: key_id = SPI_ENABLED; break;
        case PORT_UART1: key_id = UART1_ENABLED; break;
        case PORT_UART2: key_id = UART2_ENABLED; break;
        case PORT_USB: key_id = USB_ENABLED; break;
    }
    setConfigKeyValue(key_id, state, persist);
}

static ConfigKeyId keyIdForProtocol(Driver::DeviceProtocol protocol,
                                    const vector<ConfigKeyId> &keys)
{
    if (protocol == Driver::PROTOCOL_NMEA) return keys[0];
    if (protocol == Driver::PROTOCOL_UBX) return keys[1];
    return keys[2];
}

void Driver::setInputProtocol(DevicePort port, DeviceProtocol protocol, bool state, bool persist)
{
    ConfigKeyId key_id;
    switch (port) {
        case PORT_I2C:
            key_id = keyIdForProtocol(protocol,
                { I2C_IN_NMEA, I2C_IN_UBX, I2C_IN_RTCM3X });
            break;
        case PORT_SPI:
            key_id = keyIdForProtocol(protocol,
                { SPI_IN_NMEA, SPI_IN_UBX, SPI_IN_RTCM3X });
            break;
        case PORT_UART1:
            key_id = keyIdForProtocol(protocol,
                { UART1_IN_NMEA, UART1_IN_UBX, UART1_IN_RTCM3X });
            break;
        case PORT_UART2:
            key_id = keyIdForProtocol(protocol,
                { UART2_IN_NMEA, UART2_IN_UBX, UART2_IN_RTCM3X });
            break;
        case PORT_USB:
            key_id = keyIdForProtocol(protocol,
                { USB_IN_NMEA, USB_IN_UBX, USB_IN_RTCM3X });
            break;
    }
    setConfigKeyValue(key_id, state, persist);
}

void Driver::setOutputProtocol(DevicePort port, DeviceProtocol protocol, bool state, bool persist)
{
    ConfigKeyId key_id;
    switch (port) {
        case PORT_I2C:
            key_id = keyIdForProtocol(protocol,
                { I2C_OUT_NMEA, I2C_OUT_UBX, I2C_OUT_RTCM3X });
            break;
        case PORT_SPI:
            key_id = keyIdForProtocol(protocol,
                { SPI_OUT_NMEA, SPI_OUT_UBX, SPI_OUT_RTCM3X });
            break;
        case PORT_UART1:
            key_id = keyIdForProtocol(protocol,
                { UART1_OUT_NMEA, UART1_OUT_UBX, UART1_OUT_RTCM3X });
            break;
        case PORT_UART2:
            key_id = keyIdForProtocol(protocol,
                { UART2_OUT_NMEA, UART2_OUT_UBX, UART2_OUT_RTCM3X });
            break;
        case PORT_USB:
            key_id = keyIdForProtocol(protocol,
                { USB_OUT_NMEA, USB_OUT_UBX, USB_OUT_RTCM3X });
            break;
    }
    setConfigKeyValue(key_id, state, persist);
}

void Driver::setOdometer(bool state, bool persist)
{
    setConfigKeyValue(ODO_USE_ODO, state, persist);
}

void Driver::setLowSpeedCourseOverGroundFilter(bool state, bool persist)
{
    setConfigKeyValue(ODO_USE_COG, state, persist);
}

void Driver::setOutputLowPassFilteredVelocity(bool state, bool persist)
{
    setConfigKeyValue(ODO_OUTLPVEL, state, persist);
}

void Driver::setOutputLowPassFilteredHeading(bool state, bool persist)
{
    setConfigKeyValue(ODO_OUTLPCOG, state, persist);
}

void Driver::setOdometerProfile(OdometerProfile profile, bool persist)
{
    setConfigKeyValue(ODO_PROFILE, static_cast<uint8_t>(profile), persist);
}

void Driver::setUpperSpeedLimitForHeadingFilter(uint8_t speed, bool persist)
{
    setConfigKeyValue(ODO_COGMAXSPEED, speed, persist);
}

void Driver::setMaxPositionAccuracyForLowSpeedHeadingFilter(uint8_t accuracy, bool persist)
{
    setConfigKeyValue(ODO_COGMAXPOSACC, accuracy, persist);
}

void Driver::setVelocityLowPassFilterLevel(uint8_t gain, bool persist)
{
    setConfigKeyValue(ODO_VELLPGAIN, gain, persist);
}

void Driver::setHeadingLowPassFilterLevel(uint8_t gain, bool persist)
{
    setConfigKeyValue(ODO_COGLPGAIN, gain, persist);
}

void Driver::setPositionMeasurementPeriod(uint16_t period, bool persist)
{
    setConfigKeyValue(RATE_MEAS, period, persist);
}

void Driver::setMeasurementsPerSolutionRatio(uint16_t ratio, bool persist)
{
    if (ratio > 127) {
        throw std::invalid_argument("Maximum number of measurements per solution is 127");
    }
    setConfigKeyValue(RATE_NAV, ratio, persist);
}