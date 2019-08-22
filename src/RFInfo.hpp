#ifndef GPS_UBLOX_RFINFO_HPP
#define GPS_UBLOX_RFINFO_HPP

#include <cstdint>
#include <vector>

namespace gps_ublox {
    /**
     * Information for each RF block
     */
    struct RFInfo {
        uint8_t version;
        uint8_t n_blocks;

        /**
         * The actual RF info
         */
        struct Data {
            uint8_t block_id;
            uint8_t flags;
            uint8_t antenna_status;
            uint8_t antenna_power;
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