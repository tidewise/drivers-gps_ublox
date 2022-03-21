#ifndef GPS_UBLOX_SATELLITEINFO_HPP
#define GPS_UBLOX_SATELLITEINFO_HPP

#include <string>
#include <vector>
#include <base/Angle.hpp>

namespace gps_ublox {
    /**
     * Satellite information
     */
    struct SatelliteInfo {
        uint32_t time_of_week;  // in ms
        uint8_t version;

        enum Flags {
            USED_IN_SOLUTION = 1 << 3,
            DIFFERENTIAL_AVAILABLE = 1 << 6,
            RTCM_CORRECTIONS_USED = 1 << 17,
            PSEUDORANGE_CORRECTIONS_USED = 1 << 20,
            CARRIER_RANGE_CORRECTIONS_USED = 1 << 21
        };

        /**
         * The actual satellite info
         */
        struct Data {
            uint8_t gnss_id;
            uint8_t satellite_id;
            uint8_t signal_strength;  // in dBHz
            base::Angle elevation;
            base::Angle azimuth;
            double pseudorange_residual;  // in m
            uint32_t signal_flags;
        };
        std::vector<Data> signals;
    };
}

#endif