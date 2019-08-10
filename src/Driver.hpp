#ifndef GPS_UBLOX_DRIVER_HPP
#define GPS_UBLOX_DRIVER_HPP

#include <iodrivers_base/Driver.hpp>

namespace gps_ublox
{
    class Driver : public iodrivers_base::Driver
    {
        private:
            static const int BUFFER_SIZE = 256 * 12;
            uint8_t mWriteBuffer[BUFFER_SIZE];
            uint8_t mReadBuffer[BUFFER_SIZE];

        public:
            Driver();
    };

} // end namespace gps_ublox
#endif // GPS_UBLOX_DRIVER_HPP
