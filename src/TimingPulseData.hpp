#ifndef GPS_UBLOX_TIMING_TIME_PULSE_HPP
#define GPS_UBLOX_TIMING_TIME_PULSE_HPP

namespace gps_ublox {
    /** Unmarshalled representation of the UBX-TIM-TP message */
    struct TimingPulseData {
        base::Time timestamp;

        /** The time_of_week with a microseconds precision */
        base::Time time_of_week;
        /** The nanosecond part of the time_of_week information */
        int16_t time_of_week_ns;

        /** Validity and RAIM flags, see Ublox documentation */
        uint8_t flags;

        /** Validity and RAIM flags, see Ublox documentation */
        uint8_t reference_info;

        int32_t quantization_error_ns;

        /** The GPS week number
         *
         * This does not seem to be affected by the time reference indicated in the flags
         */
        uint32_t week_number;

        /** Whether the time reference is UTC
         *
         * If true, time() returns the UTC time represented by this struct
         */
        bool isUTC() const {
            return (flags & 1) == 1;
        }

        /** Return the time since UNIX Epoch represented by this struct
         *
         * Note that whether the time reference is determined by the flags
         */
        base::Time time() const {
            uint64_t week_time_ms =
                static_cast<uint64_t>(week_number) * 7 * 24 * 3600 * 1000;
            base::Time week_time =
                base::Time::fromMilliseconds(315964800000ULL) +
                base::Time::fromMilliseconds(week_time_ms);
            return week_time + time_of_week;
        }
    };
}

#endif