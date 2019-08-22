#ifndef GPS_UBLOX_SIGNALINFO_HPP
#define GPS_UBLOX_SIGNALINFO_HPP

#include <string>
#include <vector>

namespace gps_ublox {
    /**
     * Signal information
     */
    struct SignalInfo {
        uint32_t time_of_week;
        uint8_t version;
        uint8_t n_signals;

        /**
         * The actual signal info
         */
        struct Data {
            uint8_t gnss_id;
            uint8_t satellite_id;
            uint8_t signal_id;
            uint8_t frequency_id;
            int16_t pseudorange_residual;
            uint16_t signal_strength;
            uint8_t quality_indicator;
            uint8_t correction_source;
            uint8_t ionospheric_model;
            uint16_t signal_flags;
        };
        std::vector<Data> signals;
    };
}

#endif