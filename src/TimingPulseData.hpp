#ifndef GPS_UBLOX_TIMING_TIME_PULSE_HPP
#define GPS_UBLOX_TIMING_TIME_PULSE_HPP

namespace gps_ublox {
    /** Unmarshalled representation of the UBX-TIM-TP message */
    struct TimingPulseData {
        base::Time timestamp;

        base::Time time_of_week;
        uint32_t submilliseconds;
        int32_t quantization_error_ns;
        uint32_t week_number;

        /** Validity and RAIM flags, see Ublox documentation */
        uint8_t flags;

        /** Validity and RAIM flags, see Ublox documentation */
        uint8_t reference_info;

        bool isUTC() const {
            return (flags & 1) == 1;
        }
    };
}

#endif