#ifndef GPS_UBLOX_RELPOSNED_HPP
#define GPS_UBLOX_RELPOSNED_HPP

#include <base/Float.hpp>
#include <base/Time.hpp>
#include <cstdint>

namespace gps_ublox {
    struct RelPosNED {
        enum Flags {
            FLAGS_FIX_OK = 1,
            FLAGS_USE_DIFFERENTIAL = 2,
            FLAGS_RELATIVE_POSITION_VALID = 4,
            FLAGS_RTK_FLOAT = 8,
            FLAGS_RTK_FIXED = 16,
            FLAGS_MOVING_BASE = 32,
            FLAGS_REFERENCE_POSITION_MISSING = 64,
            FLAGS_REFERENCE_OBSERVATIONS_MISSING = 128,
            FLAGS_HEADING_VALID = 256,
            FLAGS_RELATIVE_POSITION_NORMALIZED = 512
        };

        std::uint32_t time_of_week = 0;  // in milliseconds
        std::uint16_t reference_station_id = 0;

        base::Vector3d relative_position_NED = base::Vector3d(
            base::unknown<double>(), base::unknown<double>(), base::unknown<double>()
        );
        float relative_position_length = base::unknown<float>();
        base::Angle relative_position_heading;
        base::Vector3d accuracy_NED = base::Vector3d(
            base::unknown<double>(), base::unknown<double>(), base::unknown<double>()
        );
        float accuracy_length = base::unknown<float>();
        base::Angle accuracy_heading;

        std::uint32_t flags = 0;
    };
}

#endif