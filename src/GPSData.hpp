#ifndef GPS_UBLOX_GPSDATA_HPP
#define GPS_UBLOX_GPSDATA_HPP

#include <base/Time.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <gps_base/UTMConverter.hpp>
#include <cstdint>

namespace gps_ublox {
    template<typename T>
    T may_invalidate(T const& value)
    {
        if (value == T::Zero())
            return T::Ones() * base::unknown<double>();
        else return value;
    }

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

        base::samples::RigidBodyState toWorld(int zone,  bool north,
                                              Eigen::Vector3d const& local_origin) const {
            base::samples::RigidBodyState rbs;
            gps_base::UTMConverter UTMConverter;
            gps_base::Solution geodeticPosition;

            Eigen::Vector3d body2ned_velocity = Eigen::Vector3d(
                vel_north, vel_east, vel_down);

            rbs.time = time;
            rbs.velocity = Eigen::AngleAxisd(
                M_PI, Eigen::Vector3d::UnitX()) * may_invalidate(body2ned_velocity);

            UTMConverter.setUTMZone(zone);
            UTMConverter.setUTMNorth(north);
            UTMConverter.setNWUOrigin(local_origin);
            geodeticPosition.latitude = latitude;
            geodeticPosition.longitude = longitude;
            geodeticPosition.altitude = height;

            rbs.position = UTMConverter.convertToNWU(geodeticPosition).position;

            return rbs;
        }

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