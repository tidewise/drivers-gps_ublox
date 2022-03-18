#include <gps_ublox/cfg.hpp>
#include <stdexcept>

using namespace gps_ublox;
using namespace std;

std::uint32_t cfg::getPortControlKey(DevicePort port) {
    switch (port) {
        case PORT_I2C:
            return I2C_ENABLED;
        case PORT_SPI:
            return SPI_ENABLED;
        case PORT_UART1:
            return UART1_ENABLED;
        case PORT_UART2:
            return UART2_ENABLED;
        case PORT_USB:
            return USB_ENABLED;
        default:
            throw std::invalid_argument("getPortControlKey: invalid device port");
    }
}

static int getMsgOutPortOffset(DevicePort port) {
    switch (port) {
        case PORT_I2C:
            return 0;
        case PORT_UART1:
            return 1;
        case PORT_UART2:
            return 2;
        case PORT_USB:
            return 3;
        case PORT_SPI:
            return 4;
        default:
            throw std::invalid_argument("getMsgOutPortOffset: invalid device port");
    }
}

std::uint32_t cfg::getOutputRateKey(DevicePort port, MessageOutputType type) {
    return getMsgOutPortOffset(port) + type;
}

struct RTCMConfig {
    static constexpr uint32_t BASE = 0x20910000;

    uint16_t rtcm_message;
    // Configuration offset from
    uint16_t cfg_offset;
};

static const RTCMConfig RTCM_MESSAGES[] = {
    { 1005, 0x2bd }, // Stationary RTK reference station ARP
    { 1074, 0x35e }, //
    { 1077, 0x2cc },
    { 1084, 0x367 },
    { 1087, 0x2d1 },
    { 1094, 0x368 },
    { 1097, 0x318 },
    { 1124, 0x36d },
    { 1127, 0x2d6 },
    { 1230, 0x303 },
    { 4072, 0x2fe },
    { 40720, 0x2fe },
    { 40721, 0x381 }
};

std::uint32_t cfg::getRTCMOutputKey(DevicePort port, uint32_t rtcm_message) {
    for (auto const& msg: RTCM_MESSAGES) {
        if (msg.rtcm_message == rtcm_message) {
            return getMsgOutPortOffset(port) + RTCMConfig::BASE + msg.cfg_offset;
        }
    }

    throw std::invalid_argument(
        "RTCM message " + to_string(rtcm_message) + " not supported as output"
    );
}
