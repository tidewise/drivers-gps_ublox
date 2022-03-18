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
