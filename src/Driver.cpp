#include <gps_ublox/Driver.hpp>
#include <base/Time.hpp>
#include <iostream>
#include <map>

using namespace std;
using namespace gps_ublox;
using namespace protocols;

Driver::Driver() : iodrivers_base::Driver(BUFFER_SIZE)
{
}

int Driver::extractPacket(const uint8_t *buffer, size_t buffer_size) const
{
    return mUBXParser.extractPacket(buffer, buffer_size);
}

BoardInfo Driver::readBoardInfo() {
    auto frame = pollFrame(UBX::MSG_CLASS_MON, UBX::MSG_ID_VER);

    BoardInfo info;
    info.software_version = string(reinterpret_cast<char*>(&frame.payload[0]));
    info.hardware_version = string(reinterpret_cast<char*>(&frame.payload[30]));

    for (size_t i = 40; i < frame.payload.size(); i += 30) {
        info.extensions.push_back(string(reinterpret_cast<char*>(&frame.payload[i])));
    }
    return info;
}

UBX::Frame Driver::pollFrame(uint8_t class_id, uint8_t msg_id)
{
    UBX::Frame frame = {
        .msg_class = class_id,
        .msg_id    = msg_id
    };
    vector<uint8_t> packet = frame.toPacket();
    writePacket(&packet[0], packet.size());
    return waitForFrame(class_id, msg_id);
}

UBX::Frame Driver::waitForFrame(uint8_t class_id, uint8_t msg_id)
{
    base::Time deadline = base::Time::now() + getReadTimeout();
    while (base::Time::now() < deadline) {
        base::Time remaining = deadline - base::Time::now();
        int bytes = readPacket(mReadBuffer, BUFFER_SIZE, remaining);

        UBX::Frame frame = UBX::Frame::fromPacket(mReadBuffer, bytes);
        if (frame.msg_class == class_id && frame.msg_id == msg_id)
            return frame;
    }

    throw iodrivers_base::TimeoutError(
        iodrivers_base::TimeoutError::PACKET,
        "Did not receive an acknowledgement within the expected timeout");
}

bool Driver::waitForAck(uint8_t class_id, uint8_t msg_id)
{
    base::Time deadline = base::Time::now() + getReadTimeout();
    while (base::Time::now() < deadline) {
        base::Time remaining = deadline - base::Time::now();
        int bytes = readPacket(mReadBuffer, BUFFER_SIZE, remaining);

        UBX::Frame frame = UBX::Frame::fromPacket(mReadBuffer, bytes);

        if (frame.msg_class != UBX::MSG_CLASS_ACK) continue;
        if (frame.payload[0] != class_id) continue;
        if (frame.payload[1] != msg_id) continue;

        return frame.msg_id == UBX::MSG_ID_ACK ? true : false;
    }

    throw iodrivers_base::TimeoutError(
        iodrivers_base::TimeoutError::PACKET,
        "Did not receive an acknowledgement within the expected timeout");
}

template<typename T>
void Driver::setConfigKeyValue(protocols::UBX::ConfigKeyId key_id, T value, bool persist)
{
    vector<uint8_t> packet = mUBXParser.getConfigValueSetPacket<T>(key_id, value, persist);
    writePacket(&packet[0], packet.size());

    if (!waitForAck(UBX::MSG_CLASS_CFG, UBX::MSG_ID_VALSET)) {
        throw ConfigValueSetError("Configuration rejected by the device");
    }
}

template void Driver::setConfigKeyValue<bool>(protocols::UBX::ConfigKeyId, bool, bool);
template void Driver::setConfigKeyValue<uint8_t>(protocols::UBX::ConfigKeyId, uint8_t, bool);
template void Driver::setConfigKeyValue<uint16_t>(protocols::UBX::ConfigKeyId, uint16_t, bool);

void Driver::setPortEnabled(DevicePort port, bool state, bool persist)
{
    UBX::ConfigKeyId key_id;
    switch (port) {
        case PORT_I2C: key_id = UBX::I2C_ENABLED; break;
        case PORT_SPI: key_id = UBX::SPI_ENABLED; break;
        case PORT_UART1: key_id = UBX::UART1_ENABLED; break;
        case PORT_UART2: key_id = UBX::UART2_ENABLED; break;
        case PORT_USB: key_id = UBX::USB_ENABLED; break;
    }
    setConfigKeyValue(key_id, state, persist);
}

static UBX::ConfigKeyId keyIdForProtocol(Driver::DeviceProtocol protocol,
                                                const vector<UBX::ConfigKeyId> &keys)
{
    if (protocol == Driver::PROTOCOL_NMEA) return keys[0];
    if (protocol == Driver::PROTOCOL_UBX) return keys[1];
    return keys[2];
}

void Driver::setInputProtocol(DevicePort port, DeviceProtocol protocol, bool state, bool persist)
{
    UBX::ConfigKeyId key_id;
    switch (port) {
        case PORT_I2C:
            key_id = keyIdForProtocol(protocol,
                { UBX::I2C_IN_NMEA, UBX::I2C_IN_UBX, UBX::I2C_IN_RTCM3X });
            break;
        case PORT_SPI:
            key_id = keyIdForProtocol(protocol,
                { UBX::SPI_IN_NMEA, UBX::SPI_IN_UBX, UBX::SPI_IN_RTCM3X });
            break;
        case PORT_UART1:
            key_id = keyIdForProtocol(protocol,
                { UBX::UART1_IN_NMEA, UBX::UART1_IN_UBX, UBX::UART1_IN_RTCM3X });
            break;
        case PORT_UART2:
            key_id = keyIdForProtocol(protocol,
                { UBX::UART2_IN_NMEA, UBX::UART2_IN_UBX, UBX::UART2_IN_RTCM3X });
            break;
        case PORT_USB:
            key_id = keyIdForProtocol(protocol,
                { UBX::USB_IN_NMEA, UBX::USB_IN_UBX, UBX::USB_IN_RTCM3X });
            break;
    }
    setConfigKeyValue(key_id, state, persist);
}

void Driver::setOutputProtocol(DevicePort port, DeviceProtocol protocol, bool state, bool persist)
{
    UBX::ConfigKeyId key_id;
    switch (port) {
        case PORT_I2C:
            key_id = keyIdForProtocol(protocol,
                { UBX::I2C_OUT_NMEA, UBX::I2C_OUT_UBX, UBX::I2C_OUT_RTCM3X });
            break;
        case PORT_SPI:
            key_id = keyIdForProtocol(protocol,
                { UBX::SPI_OUT_NMEA, UBX::SPI_OUT_UBX, UBX::SPI_OUT_RTCM3X });
            break;
        case PORT_UART1:
            key_id = keyIdForProtocol(protocol,
                { UBX::UART1_OUT_NMEA, UBX::UART1_OUT_UBX, UBX::UART1_OUT_RTCM3X });
            break;
        case PORT_UART2:
            key_id = keyIdForProtocol(protocol,
                { UBX::UART2_OUT_NMEA, UBX::UART2_OUT_UBX, UBX::UART2_OUT_RTCM3X });
            break;
        case PORT_USB:
            key_id = keyIdForProtocol(protocol,
                { UBX::USB_OUT_NMEA, UBX::USB_OUT_UBX, UBX::USB_OUT_RTCM3X });
            break;
    }
    setConfigKeyValue(key_id, state, persist);
}
