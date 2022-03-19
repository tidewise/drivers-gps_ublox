#ifndef GPS_UBLOX_GPSDATA_HPP
#define GPS_UBLOX_GPSDATA_HPP

#include <base/Time.hpp>
#include <base/Eigen.hpp>
#include <base/Angle.hpp>
#include <cstdint>

namespace gps_ublox {
    /**
     * Ublox' PVT message
     */
    struct PVT {
        enum GNSSFixType {
            NO_FIX = 0,
            DEAD_RECKONING = 1,
            FIX_2D = 2,
            FIX_3D = 3,
            GNSS_PLUS_DEAD_RECKONING = 4,
            TIME_ONLY = 5
        };

        enum GNSSFixFlags {
            FIX_OK = 1,
            FIX_DIFFERENTIAL = 2,
            FIX_RTK_FLOAT = 64,
            FIX_RTK_FIXED = 128
        };

        uint32_t time_of_week;  // in milliseconds
        uint8_t valid;
        base::Time time;
        uint32_t time_accuracy;  // in nanoseconds
        GNSSFixType fix_type;
        uint8_t fix_flags;
        uint8_t additional_flags;
        base::Time age_of_differential_corrections;
        uint8_t num_sats;
        base::Angle longitude;  // in rad
        base::Angle latitude;  // in rad
        double height;  // in meters
        double height_above_mean_sea_level;  // in meters
        double horizontal_accuracy;  // in meters
        double vertical_accuracy;  // in meters
        Eigen::Vector3d vel_ned;  // in m/s
        double ground_speed;  // in m/s
        base::Angle heading_of_motion;  // in rad
        double speed_accuracy;  // in m/s
        base::Angle heading_accuracy;  // in rad
        double position_dop;
        uint8_t more_flags;
        base::Angle heading_of_vehicle;  // in rad
        base::Angle magnetic_declination;  // in rad
        base::Angle magnetic_declination_accuracy;  // in rad
    };
}

#endif