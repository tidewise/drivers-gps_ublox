#ifndef GPS_UBLOX_TIME_UTC_HPP
#define GPS_UBLOX_TIME_UTC_HPP

#include <base/Time.hpp>

namespace gps_ublox {
    struct TimeUTC {
        enum ValidityFlags {
            TIME_VALID_TIME_OF_WEEK = 1,
            TIME_VALID_WEEK_NUMBER = 2,
            TIME_VALID_UTC = 4
        };

        /** The time at which the message was parsed */
        base::Time timestamp;

        /** The UTC time */
        base::Time utc;

        /** The GPS time of week */
        base::Time gps_time_of_week;

        /** Time accuracy in nanoseconds */
        uint32_t accuracy_ns = 0;

        /** Whether the time is valid
         *
         * @meta bitfield /gps_ublox/TimeUTC/ValidityFlags
         */
        uint8_t validity = 0;
    };
}

#endif