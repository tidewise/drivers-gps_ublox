#ifndef GPS_UBLOX_PROTOCOLS_PARSER_HPP
#define GPS_UBLOX_PROTOCOLS_PARSER_HPP

#include <stdint.h>
#include <stddef.h>

namespace gps_ublox
{
    namespace protocols {
        /** An interface that allows to abstract away the protocol
         * currently being used by a Driver instance
         */
        class Parser {
            public:
                virtual ~Parser() {}
                virtual int extractPacket(const uint8_t *buffer, size_t buffer_size) const = 0;
        };
    } // end namespace protocols
} // end namespace gps_ublox
#endif // GPS_UBLOX_PROTOCOLS_UBX_HPP
