#ifndef GPS_UBLOX_DRIVER_HPP
#define GPS_UBLOX_DRIVER_HPP

#include <iodrivers_base/Driver.hpp>
#include <gps_ublox/protocols/UBX.hpp>
#include <stdexcept>

namespace gps_ublox
{
    /** Thrown when a UBX-CFG-VALSET command fails
     */
    struct ConfigValueSetError : public std::runtime_error {
        using std::runtime_error::runtime_error;
    };

    class Driver : public iodrivers_base::Driver
    {
        private:
            static const size_t BUFFER_SIZE = 256 * 15;
            uint8_t mWriteBuffer[BUFFER_SIZE];
            uint8_t mReadBuffer[BUFFER_SIZE];
            protocols::UBX mUBXParser;

            bool waitForAck(uint8_t class_id, uint8_t msg_id);

        protected:
            /** Implements iodrivers_base's extractPacket protocol
             *
             * See iodrivers_base::extractPacket for detailed information
             */
            int extractPacket(const uint8_t *buffer, size_t buffer_size) const override;

        public:
            template<typename T>
            void setConfigKeyValue(protocols::UBX::ConfigKeyId key_id, T state, bool persist = false);

            /** Identifies the device's ports
             */
            enum DevicePort {
                PORT_I2C,
                PORT_SPI,
                PORT_UART1,
                PORT_UART2,
                PORT_USB
            };

            /** The protocols supported by the device
             */
            enum DeviceProtocol {
                PROTOCOL_UBX,
                PROTOCOL_NMEA,
                PROTOCOL_RTCM3X
            };

            Driver();

            /** Enables/disables a port
             *
             * @param port Port to be toggled
             * @param state True if the port is to be enabled
             * @param persist Wheter the configuration should be persisted
             */
            void setPortEnabled(DevicePort port, bool state, bool persist = true);

            /** Enables/disables a protocol on a given port
             *
             * @param port Port to be configured
             * @param protocol A protocol to enable/disable
             * @param state True if the protocol is to be enabled
             * @param persis Wheter the configuration should be persisted
             */
            void setInputProtocol(DevicePort port, DeviceProtocol protocol, bool state, bool persist = true);
            void setOutputProtocol(DevicePort port, DeviceProtocol protocol, bool state, bool persist = true);
    };

} // end namespace gps_ublox
#endif // GPS_UBLOX_DRIVER_HPP
