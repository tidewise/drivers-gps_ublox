#ifndef GPS_UBLOX_DRIVER_HPP
#define GPS_UBLOX_DRIVER_HPP

#include <gps_ublox/BoardInfo.hpp>
#include <gps_ublox/cfg.hpp>
#include <gps_ublox/PVT.hpp>
#include <gps_ublox/RelPosNED.hpp>
#include <gps_ublox/RFInfo.hpp>
#include <gps_ublox/SatelliteInfo.hpp>
#include <gps_ublox/SignalInfo.hpp>
#include <gps_ublox/UBX.hpp>
#include <iodrivers_base/Driver.hpp>

namespace gps_ublox
{
    /** Thrown when a UBX-CFG-VALSET command fails
     */
    struct ConfigValueSetError : public std::runtime_error {
        using std::runtime_error::runtime_error;
    };

    class Driver : public iodrivers_base::Driver
    {
        public:
            struct PollCallbacks {
                virtual ~PollCallbacks() {}

                virtual void rtcm(uint8_t const* buffer, size_t size) {};
                virtual void rtcmReceivedMessage(RTCMReceivedMessage const& info) {};
                virtual void pvt(PVT const& pvt) {};
                virtual void relposned(RelPosNED const& relposned) {};
                virtual void satelliteInfo(SatelliteInfo const& info) {};
                virtual void signalInfo(SignalInfo const& info) {};
                virtual void rfInfo(RFInfo const& info) {};
            };

        private:
            static const size_t BUFFER_SIZE = 256 * 15;
            uint8_t mWriteBuffer[BUFFER_SIZE];
            uint8_t mReadBuffer[BUFFER_SIZE];

            UBX::Frame pollFrame(uint8_t class_id, uint8_t msg_id);
            UBX::Frame waitForFrame(uint8_t class_id, uint8_t msg_id);
            UBX::Frame waitForPacket(const uint8_t *class_id = nullptr,
                                     const uint8_t *msg_id = nullptr,
                                     const std::vector<uint8_t> *payload = nullptr);
            bool waitForAck(uint8_t class_id, uint8_t msg_id);
            void pollOneFrame(PollCallbacks& callbacks, base::Time const& timeout);

        protected:
            /** Implements iodrivers_base's extractPacket protocol
             *
             * See iodrivers_base::extractPacket for detailed information
             */
            int extractPacket(const uint8_t *buffer, size_t buffer_size) const override;

        public:
            template<typename T>
            void setConfigKeyValue(uint32_t key_id, T state, bool persist = false);

            Driver();

            /** Enables/disables a port
             *
             * @param port Port to be toggled
             * @param state True if the port is to be enabled
             * @param persist Whether the configuration should be persisted
             */
            void setPortEnabled(DevicePort port, bool state, bool persist = true);

            /** Whether odometer is used or not
             *
             * @param state True if odometer is to be enabled
             * @param persist Whether the configuration should be persisted
             */
            void setOdometer(bool state, bool persist = true);

            /** Whether low-speed course over ground filter is used or not
             *
             * @param state True if the filter is to be enabled
             * @param persist Whether the configuration should be persisted
             */
            void setLowSpeedCourseOverGroundFilter(bool state, bool persist = true);

            /** Whether low-pass filtered should be output or not
             *
             * @param state True if the filter is to be enabled
             * @param persist Whether the configuration should be persisted
             */
            void setOutputLowPassFilteredVelocity(bool state, bool persist = true);

            /** Whether low-pass filtered course over ground (heading) should be output or not
             *
             * @param state True if the filter is to be enabled
             * @param persist Wheter the configuration should be persisted
             */
            void setOutputLowPassFilteredHeading(bool state, bool persist = true);

            /** Sets odometer profile
             *
             * @param profile The odomter profile to use
             * @param persist Whether the configuration should be persisted
             */
            void setOdometerProfile(OdometerProfile profile, bool persist = true);

            /** Sets upper speed limit for low-speed course over ground (heading) filter
             *
             * @param speed The upper speed limit
             * @param persist Whether the configuration should be persisted
             */
            void setUpperSpeedLimitForHeadingFilter(uint8_t speed, bool persist = true);

            /** Sets maximum acceptable position accuracy for computing
             * low-speed filtered course over ground
             *
             * @param accuracy Maximum accuracy
             * @param persist Whether the configuration should be persisted
             */
            void setMaxPositionAccuracyForLowSpeedHeadingFilter(uint8_t accuracy, bool persist = true);

            /** Sets velocity low-pass velocity filter level
             *
             * @param gain Range is from 0 to 255
             * @param persist Whether the configuration should be persisted
             */
            void setVelocityLowPassFilterLevel(uint8_t gain, bool persist = true);

            /** Sets course over ground low-pass filter level (at speed < 8 m/s)
             *
             * @param gain Range is from 0 to 255
             * @param persist Whether the configuration should be persisted
             */
            void setHeadingLowPassFilterLevel(uint8_t gain, bool persist = true);

            /** Sets the period between GNSS measurements
             *
             * @param period Period, in milliseconds
             * @param persist Whether the configuration should be persisted
             */
            void setPositionMeasurementPeriod(uint16_t period, bool persist = true);

            /** Sets the number of measurements between navigation solutions
             *
             * @param ratio Maximum of 127 measruements between solutions
             * @param persist Whether the configuration should be persisted
             */
            void setMeasurementsPerSolutionRatio(uint16_t ratio, bool persist = true);

            /** Sets the time system to which measurements are aligned
             *
             * @param system Time system
             * @param persist Whether the configuration should be persisted
             */
            void setTimeSystem(UBX::TimeSystem system, bool persist = true);

            /** Sets the dynamic platform model
             *
             * @param model Dynamic platform model
             * @param persist Whether the configuration should be persisted
             */
            void setDynamicModel(UBX::DynamicModel model, bool persist = true);

            /** Sets GNSS speed threshold below which platform is considered as
             * stationary (a.k.a. static hold threshold)
             *
             * @param speed Speed, in m/s
             * @param persist Whether the configuration should be persisted
             */
            void setSpeedThreshold(uint8_t speed, bool persist = true);

            /** Sets distance above which GNSS- based stationary motion is exit
             * (a. k.a. static hold distance threshold)
             *
             * @param distance Distance
             * @param persist Whether the configuration should be persisted
             */
            void setStaticHoldDistanceThreshold(uint16_t distance, bool persist = true);

            /** Sets the output rate of the message on a given port
             *
             * @param port Port to be toggled
             * @param msg Message type
             * @param rate Per second, per epoch
             * @param persist Whether the configuration should be persisted
             */
            void setOutputRate(DevicePort port, MessageOutputType msg, uint8_t rate, bool persist = true);

            /** Sets the output rate of a RTCM message on a given port
             *
             * @param port Port to be toggled
             * @param rtcm_message ID of the RTCM message
             * @param period Message period as multiples of the measurement period. Set to zero to disable.
             * @param persist Whether the configuration should be persisted
             */
            void setRTCMOutputRate(DevicePort port, uint16_t msg, uint8_t rate = 1, bool persist = true);

            /** Requests device version information
             */
            BoardInfo readBoardInfo();

            /** Enables/disables input and output protocols on a port
             *
             * @param port Port to configure
             * @param direction Whether to configure an input or output protocol
             * @param protocol Protocol to enable/disable
             * @param state Whether protocol should be enabled or disabled
             * @param persist Whether the configuration should be persisted
             */
            void setPortProtocol(DevicePort port, DataDirection direction,
                                 DeviceProtocol protocol, bool state, bool persist = true);

            /** Requests GPS data
             */
            PVT readPVT();

            /** Wait for a PVT message */
            PVT waitForPVT();

            /** Wait for a RelPosNED message */
            RelPosNED waitForRelPosNED();

            /** Requests RF info
             */
            RFInfo readRFInfo();

            /** Requests Signal info
             */
            SignalInfo readSignalInfo();

            /** Requests Satellite info
             */
            SatelliteInfo readSatelliteInfo();

            /** Reads any frame
             */
            UBX::Frame readFrame();

            void poll(PollCallbacks& callbacks);
    };

} // end namespace gps_ublox
#endif // GPS_UBLOX_DRIVER_HPP
