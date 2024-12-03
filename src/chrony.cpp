#include <gps_ublox/chrony.hpp>

#if HAVE_SYS_TIMEPPS
#include <sys/timepps.h>
#else
#include <timepps.h>
#endif

using namespace gps_ublox;
using namespace gps_ublox::chrony;
using namespace std;

using iodrivers_base::UnixError;

namespace {
    struct PPSGuard {
        pps_handle_t m_handle;
        bool m_released = false;

        PPSGuard(pps_handle_t handle)
            : m_handle(handle)
        {
        }
        PPSGuard(PPSGuard const&) = delete;

        ~PPSGuard()
        {
            if (!m_released) {
                time_pps_destroy(m_handle);
            }
        }

        pps_handle_t release()
        {
            m_released = true;
            return m_handle;
        }
    };
}

struct PPS::Handle {
    pps_handle_t handle;
    int fd = -1;
};

PPS::PPS(Handle const& handle)
    : m_handle(new Handle(handle))
{
}

PPS::PPS(PPS&& other)
    : m_handle(std::move(other.m_handle))
{
    other.m_handle->fd = -1;
}

PPS::~PPS()
{
    if (m_handle->fd != -1) {
        time_pps_destroy(m_handle->handle);
        close(m_handle->fd);
    }
}

optional<PPSPulse> PPS::wait(base::Time const& timeout)
{
    uint64_t timeout_us = timeout.toMicroseconds();
    timespec ts;
    ts.tv_sec = timeout_us / 1000000;
    ts.tv_nsec = (timeout_us % 1000000) * 1000;

    pps_info_t pps_info;
    if (time_pps_fetch(m_handle->fd, PPS_TSFMT_TSPEC, &pps_info, &ts) < 0) {
        if (errno == ETIMEDOUT) {
            return nullopt;
        }
        else {
            throw UnixError("failed to fetch PPS");
        }
    }

    ts = pps_info.assert_timestamp;

    PPSPulse pulse;
    pulse.receive_time = base::Time::now();
    pulse.time = base::Time::fromMicroseconds(
        static_cast<uint64_t>(ts.tv_sec) * 1000000ULL +
        static_cast<uint64_t>(ts.tv_nsec / 1000));
    pulse.time_ns = ts.tv_nsec % 1000;
    pulse.sequence = pps_info.assert_sequence;
    return pulse;
}

PPS PPS::open(std::string const& path)
{
    int fd = ::open(path.c_str(), O_RDWR);
    if (fd < 0) {
        throw UnixError("could not open PPS device " + path);
    }

    iodrivers_base::FileGuard guard(fd);

    pps_handle_t handle;
    if (time_pps_create(fd, &handle) < 0) {
        throw UnixError("could not create PPS handle for " + path);
    }

    PPSGuard pps_guard(handle);

    int mode;
    if (time_pps_getcap(handle, &mode) < 0) {
        throw UnixError("could not query PPS mode for " + path);
    }
    if (!(mode & PPS_CAPTUREASSERT)) {
        throw std::runtime_error(
            "pps " + path + " does not support capturing the rising edge");
    }

    pps_params_t params;
    if (time_pps_getparams(handle, &params) < 0) {
        throw UnixError("could not query PPS params for " + path);
    }
    if (!(params.mode & PPS_CANWAIT)) {
        throw std::runtime_error("need a PPS that has CANWAIT support");
    }

    params.mode |= PPS_CAPTUREASSERT;
    params.mode &= ~PPS_CAPTURECLEAR;

    if (time_pps_setparams(handle, &params) < 0) {
        throw UnixError("could not set PPS params for " + path);
    }

    return PPS({ pps_guard.release(), guard.release() });
}

int ChronySocket::extractPacket(uint8_t const* buffer, size_t buffer_size) const
{
    return 0;
}

ChronySocket::ChronySocket()
    : Driver(BUFFER_SIZE)
{
}

#define CHRONY_SOCK_MAGIC 0x534f434b

/** One sample as expected by chrony */
struct SocketSample {
    /* Time of the measurement (system time) */
    struct timeval tv;

    /* Offset between the true time and the system time (in seconds)
     *
     * Seems to be sys + offset = true
     * https://github.com/mlichvar/chrony/blob/master/refclock_sock.c#L135
     */
    double offset;

    /* Non-zero if the sample is from a PPS signal, i.e. another source
       is needed to obtain seconds */
    int pulse;

    /* 0 - normal, 1 - insert leap second, 2 - delete leap second */
    int leap;

    /* Padding, ignored */
    int _pad;

    /* Protocol identifier (0x534f434b) */
    int magic;
};

double ChronySocket::send(PPSPulse const& pps, TimingPulseData const& pulse_data)
{
    auto utctime = pulse_data.time();
    auto systime = pps.time;

    SocketSample sample;
    sample.tv.tv_sec = systime.toMicroseconds() / 1000000ULL;
    sample.tv.tv_usec = systime.toMicroseconds() % 1000000ULL;
    sample.offset = (utctime - systime).toSeconds();
    sample.pulse = 0;
    sample.leap = 0;
    sample._pad = 0;
    sample.magic = CHRONY_SOCK_MAGIC;

    writePacket(reinterpret_cast<uint8_t const*>(&sample), sizeof(sample));
    return sample.offset;
}