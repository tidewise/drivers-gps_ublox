#ifndef GPS_UBLOX_RTKINFO_HPP
#define GPS_UBLOX_RTKINFO_HPP

#include <cstdint>
#include <vector>

#include <base/Time.hpp>

namespace gps_ublox {
    struct PVT;
    struct SatelliteInfo;
    struct RTCMReceivedMessage;

    /** Structure gathering information that allows to track the state of the RTK
     * process
     */
    struct RTKInfo {
        struct RTCMMessageStats {
            uint16_t message_type = 0;
            uint16_t reference_station_id = 0;

            uint16_t received = 0;
            uint16_t used = 0;
            uint16_t rejected_crc = 0;
        };

        base::Time time;
        std::vector<RTCMMessageStats> rtcm_message_stats;

        enum RTKSolutionType { RTK_NONE, RTK_FLOAT, RTK_FIXED };
        /** Whether we have a fix or float solution */
        RTKSolutionType solution_type = RTK_NONE;

        /** How many satellites in this solution */
        uint8_t satellites_in_use = 0;
        /** How many satellites are tracking pseudorange (i.e. float phase) */
        uint8_t satellites_with_pseudorange = 0;
        /** How many satellites are tracking carrier range (i.e. fixed phase) */
        uint8_t satellites_with_carrier_range = 0;

        /** Amount of RTCM data transmitted */
        size_t tx = 0;
        /** Amount of RTCM data received */
        size_t rx = 0;

        void update(PVT const& pvt);
        void update(SatelliteInfo const& info);
        void update(RTCMReceivedMessage const& msg);
        void addRX(size_t bytes);
        void addTX(size_t bytes);
    };
}

#endif