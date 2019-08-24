#ifndef GPS_UBLOX_RFINFO_HPP
#define GPS_UBLOX_RFINFO_HPP

#include <cstdint>
#include <vector>

namespace gps_ublox {
    /**
     * Information for each RF block
     */

    struct RFInfo {
        enum AntennaStatus {
            ANTENNA_INIT = 0x00,
            ANTENNA_STATUS_UNKNOWN = 0x01,
            ANTENNA_OK = 0x02,
            ANTENNA_SHORT = 0x03,
            ANTENNA_OPEN = 0x04
        };

        enum AntennaPower {
            ANTENNA_OFF = 0x00,
            ANTENNA_ON = 0x01,
            ANTENNA_POWER_UNKNOWN = 0x02
        };

        enum JammingState {
            JAMMING_UNKNOWN = 0,
            JAMMING_OK = 1,
            JAMMING_WARNING = 2,
            JAMMING_CRITICAL = 3
        };

        uint8_t version;
        uint8_t n_blocks;

        /**
         * The actual RF info
         */
        struct Data {
            uint8_t block_id;
            JammingState jamming_state;
            AntennaStatus antenna_status;
            AntennaPower antenna_power;
            uint32_t post_status;
            uint16_t noise_per_measurement;
            uint16_t agc_count;
            uint8_t jamming_indicator;
            int8_t ofs_i;
            uint8_t mag_i;
            int8_t ofs_q;
            uint8_t mag_q;
        };
        std::vector<Data> blocks;
    };
}

#endif