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
                    UBX_ACK = 0x05,
                    UBX_CFG = 0x06
                };

                /** The message id enumeration
                 */
                enum MsgId {
                    NACK = 0x00,
                    ACK = 0x01,
                    VALSET = 0x8A
                };

                /** The layer where a configuration data is stored
                 */
                enum ConfigurationLayer {
                    LAYER_RAM = 1,
                    LAYER_BBR = 2,
                    LAYER_FLASH = 4,
                    LAYER_ALL = 7
                };

                /** The unique key id of a configuration value
                 */
                enum ConfigurationKeyId {
                    I2C_ENABLED = 0x10510003,
                    I2C_INPROT_UBX = 0x10710001,
                    I2C_INPROT_NMEA = 0x10710002,
                    I2C_INPROT_RTCM3X = 0x10710004,
                    I2C_OUTPROT_UBX = 0x10720001,
                    I2C_OUTPROT_NMEA = 0x10720002,
                    I2C_OUTPROT_RTCM3X = 0x10720004,
                    SPI_ENABLED = 0x10640006,
                    SPI_INPROT_UBX = 0x10790001,
                    SPI_INPROT_NMEA = 0x10790002,
                    SPI_INPROT_RTCM3X = 0x10790004,
                    SPI_OUTPROT_UBX = 0x107a0001,
                    SPI_OUTPROT_NMEA = 0x107a0002,
                    SPI_OUTPROT_RTCM3X = 0x107a0004,
                    UART1_ENABLED = 0x10520005,
                    UART1_INPROT_UBX = 0x10730001,
                    UART1_INPROT_NMEA = 0x10730002,
                    UART1_INPROT_RTCM3X = 0x10730004,
                    UART1_OUTPROT_UBX = 0x10740001,
                    UART1_OUTPROT_NMEA = 0x10740002,
                    UART1_OUTPROT_RTCM3X = 0x10740004,
                    UART2_ENABLED = 0x10530005,
                    UART2_INPROT_UBX = 0x10750001,
                    UART2_INPROT_NMEA = 0x10750002,
                    UART2_INPROT_RTCM3X = 0x10750004,
                    UART2_OUTPROT_UBX = 0x10760001,
                    UART2_OUTPROT_NMEA = 0x10760002,
                    UART2_OUTPROT_RTCM3X = 0x10760004,
                    USB_ENABLED = 0x10650001,
                    USB_INPROT_UBX = 0x10770001,
                    USB_INPROT_NMEA = 0x10770002,
                    USB_INPROT_RTCM3X = 0x10770004,
                    USB_OUTPROT_UBX = 0x10780001,
                    USB_OUTPROT_NMEA = 0x10780002,
                    USB_OUTPROT_RTCM3X = 0x10780004
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
                std::vector<uint8_t> getCfgValSetPacket(ConfigurationKeyId key_id,
                                                        bool value,
                                                        bool persist = true) const;
        };
    } // end namespace protocols
} // end namespace gps_ublox
#endif // GPS_UBLOX_PROTOCOLS_UBX_HPP
