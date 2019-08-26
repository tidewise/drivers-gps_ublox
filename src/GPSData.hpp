#ifndef GPS_UBLOX_GPSDATA_HPP
#define GPS_UBLOX_GPSDATA_HPP

#include <base/Time.hpp>
#include <cstdint>

namespace gps_ublox {
    /**
     * GPS data
     */
    struct GPSData {

        enum GNSSFixType {
            NO_FIX = 0,
            DEAD_RECKONING = 1,
            FIX_2D = 2,
            FIX_3D = 3,
            GNSS_PLUS_DEAD_RECKONING = 4,
            TIME_ONLY = 5
        };

        uint32_t time_of_week;  // in milliseconds
        uint8_t valid;
        base::Time time;
        uint32_t time_accuracy;  // in nanoseconds
        GNSSFixType fix_type;
        uint8_t fix_flags;
        uint8_t additional_flags;
        uint8_t num_sats;
        double longitude;  // in degrees
        double latitude;  // in degrees
        double height;  // in meters
        double height_above_mean_sea_level;  // in meters
        double horizontal_accuracy;  // in meters
        double vertical_accuracy;  // in meters
        double vel_north;  // in m/s
        double vel_east;  // in m/s
        double vel_down;  // in m/s
        double ground_speed;  // in m/s
        double heading_of_motion;  // in degrees
        double speed_accuracy;  // in m/s
        double heading_accuracy;  // in degrees
        double position_dop;
        uint8_t more_flags;
        double heading_of_vehicle;  // in degrees
        double magnetic_declination;  // in degrees
        double magnetic_declination_accuracy;  // in degrees
    };
}

#endif