#ifndef GPS_UBLOX_PROTOCOLS_UBX_HPP
#define GPS_UBLOX_PROTOCOLS_UBX_HPP

#include <stdint.h>
#include <stddef.h>

#include <array>
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
                static std::array<uint8_t, 2> checksum(const uint8_t *buffer, const uint8_t *end);

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

                /** The message class enumeration
                 */
                enum MsgClass {
                    MSG_CLASS_ACK = 0x05,
                    MSG_CLASS_CFG = 0x06
                };

                /** The message id enumeration
                 */
                enum MsgId {
                    MSG_ID_NACK = 0x00,
                    MSG_ID_ACK = 0x01,
                    MSG_ID_VALSET = 0x8A
                };

                /** The layer where a configuration data is stored
                 */
                enum ConfigLayer {
                    LAYER_RAM = 1,
                    LAYER_BBR = 2,
                    LAYER_FLASH = 4,
                    LAYER_ALL = 7
                };

                /** The unique key id of a configuration value
                 */
                enum ConfigKeyId {
                    I2C_ENABLED = 0x10510003,
                    I2C_IN_UBX = 0x10710001,
                    I2C_IN_NMEA = 0x10710002,
                    I2C_IN_RTCM3X = 0x10710004,
                    I2C_OUT_UBX = 0x10720001,
                    I2C_OUT_NMEA = 0x10720002,
                    I2C_OUT_RTCM3X = 0x10720004,
                    SPI_ENABLED = 0x10640006,
                    SPI_IN_UBX = 0x10790001,
                    SPI_IN_NMEA = 0x10790002,
                    SPI_IN_RTCM3X = 0x10790004,
                    SPI_OUT_UBX = 0x107a0001,
                    SPI_OUT_NMEA = 0x107a0002,
                    SPI_OUT_RTCM3X = 0x107a0004,
                    UART1_ENABLED = 0x10520005,
                    UART1_IN_UBX = 0x10730001,
                    UART1_IN_NMEA = 0x10730002,
                    UART1_IN_RTCM3X = 0x10730004,
                    UART1_OUT_UBX = 0x10740001,
                    UART1_OUT_NMEA = 0x10740002,
                    UART1_OUT_RTCM3X = 0x10740004,
                    UART2_ENABLED = 0x10530005,
                    UART2_IN_UBX = 0x10750001,
                    UART2_IN_NMEA = 0x10750002,
                    UART2_IN_RTCM3X = 0x10750004,
                    UART2_OUT_UBX = 0x10760001,
                    UART2_OUT_NMEA = 0x10760002,
                    UART2_OUT_RTCM3X = 0x10760004,
                    USB_ENABLED = 0x10650001,
                    USB_IN_UBX = 0x10770001,
                    USB_IN_NMEA = 0x10770002,
                    USB_IN_RTCM3X = 0x10770004,
                    USB_OUT_UBX = 0x10780001,
                    USB_OUT_NMEA = 0x10780002,
                    USB_OUT_RTCM3X = 0x10780004
                };

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

                /** Serializes a packet with a UBX-CFG-VALSET message
                 */
                template<typename T>
                std::vector<uint8_t> getConfigValueSetPacket(ConfigKeyId key_id,
                                                             T value,
                                                             bool persist = false) const;
        };
    } // end namespace protocols
} // end namespace gps_ublox
#endif // GPS_UBLOX_PROTOCOLS_UBX_HPP
