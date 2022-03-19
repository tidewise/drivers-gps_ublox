#include <iostream>
#include <gps_ublox/Driver.hpp>
#include <gps_base/rtcm3.hpp>
#include <map>
#include <iomanip>
#include <poll.h>

using namespace base;
using namespace gps_ublox;
using namespace std;

struct RTCMForwarder : public iodrivers_base::Driver {
    int extractPacket(uint8_t const* buffer, size_t size) const {
        return gps_base::rtcm3::extractPacket(buffer, size);
    }

    RTCMForwarder()
        : iodrivers_base::Driver(4096) {}
};

static const map<string, DevicePort> PORT_S_TO_E = {
    { "i2c", PORT_I2C },
    { "spi", PORT_SPI },
    { "uart1", PORT_UART1 },
    { "uart2", PORT_UART2 },
    { "usb", PORT_USB }
};

static const map<string, DeviceProtocol> PROTOCOL_S_TO_E = {
    { "ubx", PROTOCOL_UBX },
    { "nmea", PROTOCOL_NMEA },
    { "rtcm3x", PROTOCOL_RTCM3X }
};

static const map<PVT::GNSSFixType, string> FIX_TYPE_TO_STRING = {
    { PVT::NO_FIX, "None" },
    { PVT::FIX_2D, "2D" },
    { PVT::FIX_3D, "3D" }
};

static const map<PVT::GNSSFixFlags, string> PVT_FIX_FLAG_TO_STRING = {
    { PVT::FIX_DIFFERENTIAL, "D" },
    { PVT::FIX_RTK_FLOAT, "FloatRTK" },
    { PVT::FIX_RTK_FIXED, "FixRTK" }
};

static const map<RelPosNED::Flags, string> RELPOSNED_FLAG_TO_STRING = {
    { RelPosNED::FLAGS_USE_DIFFERENTIAL, "D" },
    { RelPosNED::FLAGS_MOVING_BASE, "Moving" },
    { RelPosNED::FLAGS_RTK_FLOAT, "FloatRTK" },
    { RelPosNED::FLAGS_RTK_FIXED, "FixRTK" }
};

int usage()
{
    cerr
        << "gps_ublox_ctl URI [COMMAND] [ARGS]\n"
        << "  URI        a valid iodrivers_base URI, e.g. serial:///dev/ttyACM0:38400\n"
        << "\n"
        << "Known commands:\n"
        << "  enable-input PORT PROTOCOL ENABLED   enable input protocol on a given port\n"
        << "  enable-output PORT PROTOCOL ENABLED  enable output protocol on a given port\n"
        << "  enable-port PORT ENABLED             enable communication on a given port\n"
        << "  poll-solution                        poll and display solution\n"
        << "  reference-station TARGET             configure itself as a RTK reference\n"
        << "                                       station, forward RTCM messages to TARGET\n"
        << flush;

    return 0;
}

template <typename T>
void printKeys(const map<string, T> &keyMap) {
    for (const auto &port : keyMap) {
        cout << " " << port.first;
    }
    cout << endl;
}

void printPorts()
{
    cout << "Valid ports:";
    printKeys(PORT_S_TO_E);
}

void printProtocols()
{
    cout << "Valid protocols:";
    printKeys(PROTOCOL_S_TO_E);
}

struct PollCallbacks : Driver::PollCallbacks {
    int tx = 0;
    int this_tx = 0;
    int rx = 0;
    int this_rx = 0;

    int sats = 0;
    int satsWithCorrections = 0;

    RTCMForwarder* rtcm_out = nullptr;

    PollCallbacks(RTCMForwarder* rtcm_out = nullptr)
        : rtcm_out(rtcm_out) {}

    void satelliteInfo(SatelliteInfo const& data) override {
        int sats = 0, satsWithCorrections = 0;
        for (auto const& satinfo: data.signals) {
            if (satinfo.signal_flags & SatelliteInfo::USED_IN_SOLUTION) {
                sats++;
                if (satinfo.signal_flags & SatelliteInfo::RTCM_CORRECTIONS_USED) {
                    satsWithCorrections++;
                }
            }
        }

        this->sats = sats;
        this->satsWithCorrections = satsWithCorrections;
    }

    void pvt(PVT const& pvt) override {
        string fix_type = FIX_TYPE_TO_STRING.at(pvt.fix_type);
        for (auto const& flag: PVT_FIX_FLAG_TO_STRING) {
            if ((flag.first & pvt.fix_flags) == flag.first) {
                fix_type += "/" + flag.second;
            }
        }

        tx += this_tx;
        std::cout
            << base::Time::now() << " "
            << fixed << setw(5) << pvt.latitude.getDeg() << " " << pvt.longitude.getDeg() << " "
            << pvt.height << " " << pvt.height_above_mean_sea_level << " "
            << fix_type << " " << tx << " " << this_tx << "\n";
        this_tx = 0;
    }

    void rtcm(uint8_t const* buffer, size_t size) override {
        if (!rtcm_out) {
            return;
        }

        this_tx += size;
        rtcm_out->writePacket(buffer, size);
    }

    void rtcmReceivedMessage(RTCMReceivedMessage const& message) override {
        string flags = "";
        if (message.flags & RTCMReceivedMessage::MESSAGE_NOT_USED) {
            flags = "NOT_USED";
        }
        else if (message.flags & RTCMReceivedMessage::MESSAGE_USED) {
            flags = "USED";
        }
        else {
            flags = "USAGE_UNKNOWN";
        }
        if (message.flags & RTCMReceivedMessage::CRC_FAILED) {
            flags += "/CRC_FAILED";
        }


        std::cout << message.time << " RTCM" << message.message_type << " " << flags << "\n";

    }

    void relposned(RelPosNED const& data) override {
        string flags;
        for (auto const& flag: RELPOSNED_FLAG_TO_STRING) {
            if ((flag.first & data.flags) == flag.first) {
                if (!flags.empty()) {
                    flags += "/";
                }
                flags += flag.second;
            }
        }

        rx += this_rx;

        auto relpos = data.relative_position_NED;
        std::cout
            << base::Time::now() << " "
            << relpos.x() << " " << relpos.y() << " " << relpos.z() << " "
            << data.relative_position_length << "m " << data.relative_position_heading << " "
            << flags << " " << rx << " " << this_rx << " "
            << sats << "/" << satsWithCorrections << "\n";
        this_rx = 0;
    }
};

void setDefaults(Driver& driver) {
    driver.setPortProtocol(PORT_USB, DIRECTION_INPUT, PROTOCOL_UBX, true, false);
    driver.setPortProtocol(PORT_USB, DIRECTION_INPUT, PROTOCOL_RTCM3X, true, false);
    driver.setPortProtocol(PORT_USB, DIRECTION_OUTPUT, PROTOCOL_UBX, true, false);
    driver.setPortProtocol(PORT_USB, DIRECTION_OUTPUT, PROTOCOL_RTCM3X, false, false);
    driver.setPortProtocol(PORT_USB, DIRECTION_OUTPUT, PROTOCOL_NMEA, false, false);
    driver.setOutputRate(PORT_USB, MSGOUT_NAV_PVT, 0, false);
    driver.setOutputRate(PORT_USB, MSGOUT_NAV_RELPOSNED, 0, false);
    driver.setOutputRate(PORT_USB, MSGOUT_NAV_SAT, 0, false);
    driver.setReadTimeout(base::Time::fromMilliseconds(2000));
}

void poll(Driver& driver, RTCMForwarder* rtcm_in, RTCMForwarder* rtcm_out) {
    PollCallbacks callbacks(rtcm_out);

    size_t poll_n = 1;
    pollfd fds[2];
    fds[0].fd = driver.getFileDescriptor();
    fds[0].events = POLLIN;
    fds[1].revents = 0;
    if (rtcm_in) {
        ++poll_n;
        fds[1].fd = rtcm_in->getFileDescriptor();
        fds[1].events = POLLIN;
    }

    uint8_t rtcmBuffer[4096];
    while(true) {
        int ret = poll(fds, poll_n, 2000);
        if (ret <= 0) {
            continue;
        }

        if (fds[1].revents & POLLIN) {
            do {
                size_t s = rtcm_in->readPacket(rtcmBuffer, 4096, base::Time::fromMilliseconds(10));
                callbacks.this_rx += s;
                driver.writePacket(rtcmBuffer, s);
            }
            while (rtcm_in->hasPacket());
        }

        if (fds[0].revents & POLLIN) {
            driver.poll(callbacks);
        }
    }
}

int main(int argc, char** argv)
{
    if (argc < 3) {
        cerr << "not enough arguments" << endl;
        return usage();
    }

    string uri(argv[1]);
    string cmd(argv[2]);

    Driver driver;
    driver.setReadTimeout(base::Time::fromMilliseconds(100));
    driver.setWriteTimeout(base::Time::fromMilliseconds(100));

    if (cmd == "enable-port") {
        if (argc == 3) {
            printPorts();
            return 0;
        }
        else if (argc != 5) {
            cerr << "enable-port expects exactly two more parameters, ";
            cerr << "PORT and ENABLED" << endl;
            return 1;
        }

        string port = argv[3];
        string enabled_s = argv[4];
        if (enabled_s != "on" && enabled_s != "off") {
            cerr << "ENABLED parameter must be 'on' or 'off'" << endl;
            return 1;
        }
        bool enable = enabled_s == "on";

        if (PORT_S_TO_E.count(port)) {
            driver.openURI(uri);
            driver.setPortEnabled(PORT_S_TO_E.at(port), enable);
        } else {
            printPorts();
            return 1;
        }
    } else if ((cmd == "enable-input") || (cmd == "enable-output")) {
        bool input = cmd == "enable-input";

        if (argc == 3) {
            printProtocols();
            return 0;
        }
        else if (argc != 6) {
            cerr << cmd << " expects exactly three more parameters, "
                 << "PORT, PROTOCOL and ENABLED" << endl;
            return 1;
        }

        string port = argv[3];
        string protocol = argv[4];
        string enabled_s = argv[5];
        if (enabled_s != "on" && enabled_s != "off") {
            cerr << "ENABLED parameter must be 'on' or 'off'" << endl;
            return 1;
        }
        bool enable = enabled_s == "on";

        if (!PORT_S_TO_E.count(port)) {
            printPorts();
            return 1;
        }
        if (!PROTOCOL_S_TO_E.count(protocol)) {
            printProtocols();
            return 1;
        }
        driver.openURI(uri);
        driver.setPortProtocol(PORT_S_TO_E.at(port),
                               input ? DIRECTION_INPUT : DIRECTION_OUTPUT,
                               PROTOCOL_S_TO_E.at(protocol),
                               enable);
    } else if (cmd == "poll-solution") {
        string rtk_source;
        RTCMForwarder rtcm_in;

        if (argc > 4) {
            usage();
            return 1;
        }
        else if (argc == 4) {
            rtk_source = argv[3];
            rtcm_in.openURI(rtk_source);
        }

        driver.openURI(uri);
        setDefaults(driver);
        driver.setOutputRate(PORT_USB, MSGOUT_NAV_PVT, 1, false);
        driver.setOutputRate(PORT_USB, MSGOUT_NAV_SAT, 5, false);
        driver.setReadTimeout(base::Time::fromSeconds(2));

        cout << "Lat Lon Height HAboveMSL Fix" << endl;
        poll(driver, (rtcm_in.isValid() ? &rtcm_in : nullptr), nullptr);
    } else if (cmd == "poll-relposned") {
        if (argc != 4) {
            usage();
            return 1;
        }

        string rtk_source = argv[3];
        RTCMForwarder rtcm_in;
        rtcm_in.openURI(rtk_source);

        driver.openURI(uri);
        setDefaults(driver);
        driver.setOutputRate(PORT_USB, MSGOUT_NAV_RELPOSNED, 1, false);
        driver.setOutputRate(PORT_USB, MSGOUT_NAV_SAT, 5, false);
        driver.setReadTimeout(base::Time::fromMilliseconds(2000));

        cout << "Lat Lon Height HAboveMSL Fix" << endl;
        poll(driver, &rtcm_in, nullptr);
    } else if (cmd == "reference-station") {
        if (argc != 4) {
            usage();
            return 1;
        }

        string target = argv[3];
        RTCMForwarder rtcm_out;
        rtcm_out.openURI(target);

        driver.openURI(uri);
        setDefaults(driver);
        driver.setPortProtocol(PORT_USB, DIRECTION_OUTPUT, PROTOCOL_RTCM3X, true, false);
        driver.setOutputRate(PORT_USB, MSGOUT_NAV_PVT, 1, false);
        driver.setOutputRate(PORT_USB, MSGOUT_NAV_SAT, 5, false);
        driver.setRTCMOutputRate(PORT_USB, 1074, 1, false);
        driver.setRTCMOutputRate(PORT_USB, 1084, 1, false);
        driver.setRTCMOutputRate(PORT_USB, 1094, 1, false);
        driver.setRTCMOutputRate(PORT_USB, 1124, 1, false);
        driver.setRTCMOutputRate(PORT_USB, 1230, 1, false);
        driver.setRTCMOutputRate(PORT_USB, 4072, 1, false);
        driver.setReadTimeout(base::Time::fromSeconds(2));

        cout << "Lat Lon Height HAboveMSL Fix TX" << endl;
        poll(driver, nullptr, &rtcm_out);
    } else {
        cerr << "unexpected command " << cmd << endl;
        return usage();
    }
    return 0;
}
