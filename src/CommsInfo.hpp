#ifndef GPS_UBLOX_COMMSINFO_HPP
#define GPS_UBLOX_COMMSINFO_HPP

#include <cstdint>
#include <vector>

#include <base/Time.hpp>

#include <gps_ublox/cfg.hpp>

namespace gps_ublox {
    struct CommsInfo {
        base::Time time;

        enum TXErrors {
            TX_ERROR_MEM = 1,
            TX_ERROR_BUFFER_FULL = 2
        };

        enum Protocols {
            UBX = 0,
            NMEA = 1,
            RTCM2 = 2,
            RTCM3 = 5,
            SPARTN = 6
        };

        struct MessageCount {
            uint16_t count = 0;
            Protocols protocol = UBX;
        };

        struct PortInfo {
            DevicePort device_port = PORT_USB;

            uint16_t tx_pending = 0;
            uint32_t tx_bytes = 0;
            uint8_t tx_usage = 0;
            uint8_t tx_peak_usage = 0;

            uint16_t rx_pending = 0;
            uint32_t rx_bytes = 0;
            uint8_t rx_usage = 0;
            uint8_t rx_peak_usage = 0;
            uint16_t overrun_errors = 0;

            std::vector<MessageCount> message_count;
            uint32_t skipped_bytes = 0;
        };

        uint8_t tx_errors;
        std::vector<PortInfo> per_port;
    };
}

#endif