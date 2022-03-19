#ifndef GPS_UBLOX_CONFIGURATION_HPP
#define GPS_UBLOX_CONFIGURATION_HPP

#include <cstdint>

namespace gps_ublox {
    /** Identifies the device's ports
     */
    enum DevicePort {
        PORT_I2C = 0x10710000,
        PORT_SPI = 0x10790000,
        PORT_UART1 = 0x10730000,
        PORT_UART2 = 0x10750000,
        PORT_USB = 0x10770000
    };

    /** The protocols supported by the device
     */
    enum DeviceProtocol {
        PROTOCOL_UBX = 1,
        PROTOCOL_NMEA = 2,
        PROTOCOL_RTCM3X = 4
    };

    /** Odometer profile enumeration
     */
    enum OdometerProfile {
        ODOM_RUNNING = 0,
        ODOM_CYCLING,
        ODOM_SWIMMING,
        ODOM_CAR,
        ODOM_CUSTOM
    };

    /** Data direction
     */
    enum DataDirection {
        DIRECTION_INPUT = 0,
        DIRECTION_OUTPUT = 0x00010000,
    };

    /** Message types
     */
    enum MessageOutputType {
        MSGOUT_NAV_PVT = 0x20910006,
        MSGOUT_NAV_RELPOSNED = 0x2091008d,
        MSGOUT_NAV_SAT = 0x20910015,
        MSGOUT_NAV_SIG = 0x20910345,
        MSGOUT_MON_RF = 0x20910359
    };

    namespace cfg {
        /** The unique key id of a configuration value
         */
        enum ConfigKeyId {
            I2C_ENABLED = 0x10510003,
            SPI_ENABLED = 0x10640006,
            UART1_ENABLED = 0x10520005,
            UART2_ENABLED = 0x10530005,
            USB_ENABLED = 0x10650001,
            ODO_USE_ODO = 0x10220001,
            ODO_USE_COG = 0x10220002,
            ODO_OUTLPVEL = 0x10220003,
            ODO_OUTLPCOG = 0x10220004,
            ODO_PROFILE = 0x20220005,
            ODO_COGMAXSPEED = 0x20220021,
            ODO_COGMAXPOSACC = 0x20220022,
            ODO_VELLPGAIN = 0x20220031,
            ODO_COGLPGAIN = 0x20220032,
            RATE_MEAS = 0x30210001,
            RATE_NAV = 0x30210002,
            RATE_TIMEREF = 0x20210003,
            NAVSPG_DYNMODEL = 0x20110021,
            MOT_GNSSSPEED_THRS = 0x20250038,
            MOT_GNSSDIST_THRS = 0x3025003b
        };

        std::uint32_t getPortControlKey(DevicePort port);
        std::uint32_t getPortProtocolKey(
            DevicePort port, DataDirection direction, DeviceProtocol protocol
        );
        std::uint32_t getOutputRateKey(DevicePort port, MessageOutputType type);

        /** Configuration key for the RTCM messages
         *
         * @param rtcm ID of the RTCM messages. For the 4072.0 and .1 messages,
         *    use 40720 and 40721 to specify which message to set. 4072 will
         *    set 4072.0
         */
        std::uint32_t getRTCMOutputKey(DevicePort port, std::uint32_t rtcm);
    };
}

#endif