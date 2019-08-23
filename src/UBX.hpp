#ifndef GPS_UBLOX_UBX_HPP
#define GPS_UBLOX_UBX_HPP

#include <stdint.h>
#include <stddef.h>

#include <array>
#include <vector>

#include <gps_ublox/GPSData.hpp>
#include <gps_ublox/RFInfo.hpp>
#include <gps_ublox/SignalInfo.hpp>

namespace gps_ublox
{
    namespace UBX
    {
        static const uint8_t SYNC_1 = 0xB5;
        static const uint8_t SYNC_2 = 0x62;
        static const uint8_t SYNC_1_IDX = 0;
        static const uint8_t SYNC_2_IDX = 1;
        static const uint8_t MSG_CLASS_IDX = 2;
        static const uint8_t MSG_ID_IDX = 3;
        static const uint8_t LENGTH_LSB_IDX = 4;
        static const uint8_t LENGTH_MSB_IDX = 5;
        static const uint8_t PAYLOAD_IDX = 6;
        static const size_t FRAMING_SIZE_OVERHEAD = 8;

        /** Computes the 2-byte checksum over the given buffer
         */
        std::array<uint8_t, 2> checksum(const uint8_t *buffer, const uint8_t *end);

        /** The message class enumeration
         */
        enum MsgClass {
            MSG_CLASS_ACK = 0x05,
            MSG_CLASS_CFG = 0x06,
            MSG_CLASS_MON = 0x0A,
            MSG_CLASS_NAV = 0x01
        };

        /** The message id enumeration
         */
        enum MsgId {
            MSG_ID_NACK = 0x00,
            MSG_ID_ACK = 0x01,
            MSG_ID_VALSET = 0x8A,
            MSG_ID_VER = 0x04,
            MSG_ID_PVT = 0x07,
            MSG_ID_RF = 0x38,
            MSG_ID_SIG = 0x43
        };

        /** The layer where a configuration data is stored
         */
        enum ConfigLayer {
            LAYER_RAM = 1,
            LAYER_BBR = 2,
            LAYER_FLASH = 4,
            LAYER_ALL = 7
        };

        /** Time systems used to align measurements
         */
        enum TimeSystem {
            UTC = 0,
            GPS = 1,
            GLONASS = 2,
            BEIDOU = 3,
            GALILEO = 4
        };

        /** The unique key id of a configuration value
         */
        enum ConfigKeyId {
            I2C_ENABLED = 0x10510003,
            SPI_ENABLED = 0x10640006,
            UART1_ENABLED = 0x10520005,
            UART2_ENABLED = 0x10530005,
            USB_ENABLED = 0x10650001,
            ODO_USE_ODO = 0x10220001,
            ODO_USE_COG = 0x10220002,
            ODO_OUTLPVEL = 0x10220003,
            ODO_OUTLPCOG = 0x10220004,
            ODO_PROFILE = 0x20220005,
            ODO_COGMAXSPEED = 0x20220021,
            ODO_COGMAXPOSACC = 0x20220022,
            ODO_VELLPGAIN = 0x20220031,
            ODO_COGLPGAIN = 0x20220032,
            RATE_MEAS = 0x30210001,
            RATE_NAV = 0x30210002,
            RATE_TIMEREF = 0x20210003
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

        /**
         * Parses an UBX-NAV-PVT payload
         */
        GPSData parsePVT(const std::vector<uint8_t> &payload);

        /**
         * Parses an UBX-MON-RF payload
         */
        RFInfo parseRF(const std::vector<uint8_t> &payload);

        /**
         * Parses an UBX-NAV-SIG payload
         */
        SignalInfo parseSIG(const std::vector<uint8_t> &payload);


        /** Implements iodrivers_base's extractPacket protocol
         *
         * See iodrivers_base::extractPacket for detailed information
         */
        int extractPacket(const uint8_t *buffer, size_t buffer_size);

        /** Serializes a packet with a UBX-CFG-VALSET message
         */
        template<typename T>
        std::vector<uint8_t> getConfigValueSetPacket(uint32_t key_id,
                                                     T value,
                                                     bool persist = true);
    } // end namespace UBX
} // end namespace gps_ublox
#endif // GPS_UBLOX_UBX_HPP
