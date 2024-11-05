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

        uint32_t week_number;

        bool isUTC() const {
            return (flags & 1) == 1;
        }
    };
}

#endif