#ifndef GPS_UBLOX_RTCMRECEIVEDMESSAGE_HPP
#define GPS_UBLOX_RTCMRECEIVEDMESSAGE_HPP

#include <cstdint>

#include <base/Time.hpp>

namespace gps_ublox {
    /** Representation of UBX' MXM-RTCM message which gives information about
     * received RTCM messages
     */
    struct RTCMReceivedMessage {
        /** Time of reception of the MXM-RTCM message */
        base::Time time;

        enum Flags {
            CRC_FAILED = 1,
            MESSAGE_USED = 4,
            MESSAGE_NOT_USED = 2,
        };

        uint8_t flags = 0;

        /** Reference station ID, for messages that embed one
         *
         * The value (between 0 and 4095) for messages that embed a reference station
         * ID. 0xFFFF for all other messages
         */
        uint16_t reference_station_id = 0;

        /** RTCM message type
         *
         * The u-blox proprietary message 4072 is represented as 40720 plus
         * the subtype (i.e. 4072.1 is 40721)
         */
        uint16_t message_type = 0;
    };
}

#endif