#ifndef GPS_UBLOX_CHRONY_HPP
#define GPS_UBLOX_CHRONY_HPP

#include <fcntl.h>
#include <memory>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <utility>

#include <base/Time.hpp>
#include <iodrivers_base/Driver.hpp>

#include <gps_ublox/TimingPulseData.hpp>

namespace gps_ublox {
    namespace chrony {
        /** Representation of a single PPS pulse */
        struct PPSPulse {
            base::Time time;
            int16_t time_ns = 0;
            uint64_t sequence = 0;

            bool valid() const
            {
                return !time.isNull();
            }
        };

        /** Management of the PPS source */
        class PPS {
            /** Private struct used to hide the dependency on ppstime.h */
            struct Handle;
            std::unique_ptr<Handle> m_handle;

            PPS(Handle const& handle);

        public:
            ~PPS();
            PPS(PPS const&) = delete;
            PPS(PPS&& other);

            PPSPulse wait();

            static PPS open(std::string const& path);
        };

        /** Handling of the data channel to Chrony itself */
        class ChronySocket : public iodrivers_base::Driver {
            static constexpr int BUFFER_SIZE = 32768;

            int extractPacket(uint8_t const* buffer, size_t buffer_size) const override;

        public:
            ChronySocket();

            double send(PPSPulse const& pps, TimingPulseData const& pulse_data);
        };
    }
}

#endif