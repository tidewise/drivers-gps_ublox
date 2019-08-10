#ifndef GPS_UBLOX_PROTOCOLS_UBX_HPP
#define GPS_UBLOX_PROTOCOLS_UBX_HPP

#include <stdint.h>
#include <stddef.h>

#include <vector>

#include <gps_ublox/protocols/Parser.hpp>

namespace gps_ublox
{
    namespace protocols {
        class UBX : public Parser
        {
            private:
                /** Computes the 2-byte checksum over the given buffer
                 */
                static std::vector<uint8_t> checksum(const uint8_t *buffer, const uint8_t *end);

            public:
                static const uint8_t SYNC_1;
                static const uint8_t SYNC_2;
                static const uint8_t SYNC_1_IDX;
                static const uint8_t SYNC_2_IDX;
                static const uint8_t MSG_CLASS_IDX;
                static const uint8_t MSG_ID_IDX;
                static const uint8_t LENGTH_LSB_IDX; // the payload-only length (LSB)
                static const uint8_t LENGTH_MSB_IDX; // the payload-only length (MSB)
                static const uint8_t PAYLOAD_IDX;
                static const size_t FRAMING_SIZE_OVERHEAD;

                /** Represents an UBX binary data frame
                 *
                 * the over-the-wire format is:
                 * ,------------------------------------------------------------------------------,
                 * | SYNC_1 | SYNC_2 | MSG_CLASS | MSG_ID | 2-byte LENGTH | PAYLOAD | CK_A | CK_B |
                 * `------------------------------------------------------------------------------'
                 *
                 * Length is the size of the payload and nothing else (in Little-Endian order)
                 */
                struct Frame {
                    uint8_t msg_class;
                    uint8_t msg_id;
                    std::vector<uint8_t> payload;

                    /** Serializes the frame into a packet for transmission
                     */
                    std::vector<uint8_t> toPacket() const;

                    /** Deserializes a packet into an structured object
                     */
                    static Frame fromPacket(const uint8_t *buffer, size_t size);
                };

                /** Implements iodrivers_base's extractPacket protocol
                 *
                 * See iodrivers_base::extractPacket for detailed information
                 */
                int extractPacket(const uint8_t *buffer, size_t buffer_size) const;
        };
    } // end namespace protocols
} // end namespace gps_ublox
#endif // GPS_UBLOX_PROTOCOLS_UBX_HPP
