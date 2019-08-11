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

bool Driver::waitForAck(uint8_t class_id, uint8_t msg_id)
{
    base::Time deadline = base::Time::now() + getReadTimeout();
    while (base::Time::now() < deadline) {
        base::Time remaining = deadline - base::Time::now();
        int bytes = readPacket(mReadBuffer, BUFFER_SIZE, remaining);

        UBX::Frame frame = UBX::Frame::fromPacket(mReadBuffer, bytes);

        if (frame.msg_class != UBX::UBX_ACK) continue;
        if (frame.payload[0] != class_id) continue;
        if (frame.payload[1] != msg_id) continue;

        return frame.msg_id == UBX::ACK ? true : false;
    }

    throw iodrivers_base::TimeoutError(
        iodrivers_base::TimeoutError::PACKET,
        "Did not receive an acknowledgement within the expected timeout");
}

void Driver::setCfgKeyValue(UBX::ConfigurationKeyId key_id, bool state, bool persist)
{
    vector<uint8_t> packet = mUBXParser.getCfgValSetPacket(key_id, state, persist);
    writePacket(&packet[0], packet.size());

    if (!waitForAck(UBX::UBX_CFG, UBX::VALSET)) {
        throw ConfigValSetFailed("Configuration rejected by the device");
    }
}

void Driver::setPortEnabled(DevicePort port, bool state, bool persist)
{
    UBX::ConfigurationKeyId key_id;
    switch (port) {
        case PORT_I2C: key_id = UBX::I2C_ENABLED; break;
        case PORT_SPI: key_id = UBX::SPI_ENABLED; break;
        case PORT_UART1: key_id = UBX::UART1_ENABLED; break;
        case PORT_UART2: key_id = UBX::UART2_ENABLED; break;
        case PORT_USB: key_id = UBX::USB_ENABLED; break;
    }
    setCfgKeyValue(key_id, state, persist);
}

static UBX::ConfigurationKeyId keyIdForProtocol(Driver::DeviceProtocol protocol,
                                                const vector<UBX::ConfigurationKeyId> &keys)
{
    if (protocol == Driver::PROTOCOL_NMEA) return keys[0];
    if (protocol == Driver::PROTOCOL_UBX) return keys[1];
    return keys[2];
}

void Driver::setPortInProt(DevicePort port, DeviceProtocol protocol, bool state, bool persist)
{
    UBX::ConfigurationKeyId key_id;
    switch (port) {
        case PORT_I2C:
            key_id = keyIdForProtocol(protocol,
                { UBX::I2C_INPROT_NMEA, UBX::I2C_INPROT_UBX, UBX::I2C_INPROT_RTCM3X });
            break;
        case PORT_SPI:
            key_id = keyIdForProtocol(protocol,
                { UBX::SPI_INPROT_NMEA, UBX::SPI_INPROT_UBX, UBX::SPI_INPROT_RTCM3X });
            break;
        case PORT_UART1:
            key_id = keyIdForProtocol(protocol,
                { UBX::UART1_INPROT_NMEA, UBX::UART1_INPROT_UBX, UBX::UART1_INPROT_RTCM3X });
            break;
        case PORT_UART2:
            key_id = keyIdForProtocol(protocol,
                { UBX::UART2_INPROT_NMEA, UBX::UART2_INPROT_UBX, UBX::UART2_INPROT_RTCM3X });
            break;
        case PORT_USB:
            key_id = keyIdForProtocol(protocol,
                { UBX::USB_INPROT_NMEA, UBX::USB_INPROT_UBX, UBX::USB_INPROT_RTCM3X });
            break;
    }
    setCfgKeyValue(key_id, state, persist);
}

void Driver::setPortOutProt(DevicePort port, DeviceProtocol protocol, bool state, bool persist)
{
    UBX::ConfigurationKeyId key_id;
    switch (port) {
        case PORT_I2C:
            key_id = keyIdForProtocol(protocol,
                { UBX::I2C_OUTPROT_NMEA, UBX::I2C_OUTPROT_UBX, UBX::I2C_OUTPROT_RTCM3X });
            break;
        case PORT_SPI:
            key_id = keyIdForProtocol(protocol,
                { UBX::SPI_OUTPROT_NMEA, UBX::SPI_OUTPROT_UBX, UBX::SPI_OUTPROT_RTCM3X });
            break;
        case PORT_UART1:
            key_id = keyIdForProtocol(protocol,
                { UBX::UART1_OUTPROT_NMEA, UBX::UART1_OUTPROT_UBX, UBX::UART1_OUTPROT_RTCM3X });
            break;
        case PORT_UART2:
            key_id = keyIdForProtocol(protocol,
                { UBX::UART2_OUTPROT_NMEA, UBX::UART2_OUTPROT_UBX, UBX::UART2_OUTPROT_RTCM3X });
            break;
        case PORT_USB:
            key_id = keyIdForProtocol(protocol,
                { UBX::USB_OUTPROT_NMEA, UBX::USB_OUTPROT_UBX, UBX::USB_OUTPROT_RTCM3X });
            break;
    }
    setCfgKeyValue(key_id, state, persist);
}

