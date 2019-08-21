#ifndef GPS_UBLOX_GPSDATA_HPP
#define GPS_UBLOX_GPSDATA_HPP

#include <cstdint>

namespace gps_ublox {
    /**
     * GPS data
     */
    struct GPSData {
        uint32_t time_of_week;
        uint16_t year;
        uint8_t month;
        uint8_t day;
        uint8_t hour;
        uint8_t min;
        uint8_t sec;
        uint8_t valid;
        uint32_t time_accuracy;
        int32_t fraction;
        uint8_t fix_type;
        uint8_t fix_flags;
        uint8_t additional_flags;
        uint8_t num_sats;
        int32_t longitude;
        int32_t latitude;
        int32_t height;
        int32_t height_above_mean_sea_level;
        uint32_t horizontal_accuracy;
        uint32_t vertical_accuracy;
        int32_t vel_north;
        int32_t vel_east;
        int32_t vel_down;
        int32_t ground_speed;
        int32_t heading_of_motion;
        uint32_t speed_accuracy;
        uint32_t heading_accuracy;
        uint16_t position_dop;
        uint8_t more_flags;
        int32_t heading_of_vehicle;
        int16_t magnetic_declination;
        uint16_t magnetic_declination_accuracy;
    };
}

#endif