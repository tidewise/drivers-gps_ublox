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
        throw std::logic_error("only use readRaw");
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
        << "Below:\n"
        << "  PORT is one of i2c, spi, uart1, uart2 or usb\n"
        << "  PROTOCOL is one of ubx, nmea, rtcm3x\n"
        << "\n"
        << "Known commands:\n"
        << "  enable-input PORT PROTOCOL on|off    enable input protocol on a given port\n"
        << "  enable-output PORT PROTOCOL on|off   enable output protocol on a given port\n"
        << "  cfg-reset                            reset configuration to factory defaults\n"
        << "                                       It does not save the reset configuration\n"
        << "  cfg-save                             save configuration to non-volatile memory\n"
        << "  cfg-uart PORT BAUDRATE               configure a UART port\n"
        << "  enable-port PORT ENABLED             enable communication on a given port\n"
        << "  poll-solution PORT [CORR_SOURCE]     poll and display solution, optionally receiving\n"
        << "                                       corrections on the given iodrivers_base-compatible URI\n"
        << "  poll-relposned PORT CORR_SOURCE [--monitor-rtcm]\n"
        << "     read and display position relative to a RTK base station (in static or\n"
        << "     moving base mode). With --monitor-rtcm, show received RTCM messages (for\n"
        << "     debugging of the transmission)\n"
        << "  reference-station PORT TARGET        configure itself as a RTK reference\n"
        << "                                       station, forward RTCM messages to TARGET\n"
        << flush;

    return 0;
}

struct PollCallbacks : Driver::PollCallbacks {
    int sats = 0;
    int satsWithCorrections = 0;
    int satsWithCarrierRange = 0;
    int satsWithPseudoRange = 0;

    RTCMForwarder* rtcm_out = nullptr;

    PollCallbacks(RTCMForwarder* rtcm_out = nullptr)
        : rtcm_out(rtcm_out) {}

    void satelliteInfo(SatelliteInfo const& data) override {
        int sats = 0, satsWithCorrections = 0, satsWithCarrierRange = 0, satsWithPseudoRange = 0;
        for (auto const& satinfo: data.signals) {
            if (satinfo.signal_flags & SatelliteInfo::USED_IN_SOLUTION) {
                sats++;
                if (satinfo.signal_flags & SatelliteInfo::RTCM_CORRECTIONS_USED) {
                    satsWithCorrections++;
                }
                if (satinfo.signal_flags & SatelliteInfo::CARRIER_RANGE_CORRECTIONS_USED) {
                    satsWithCarrierRange++;
                }
                if (satinfo.signal_flags & SatelliteInfo::PSEUDORANGE_CORRECTIONS_USED) {
                    satsWithPseudoRange++;
                }
            }
        }

        this->sats = sats;
        this->satsWithCorrections = satsWithCorrections;
        this->satsWithCarrierRange = satsWithCarrierRange;
        this->satsWithPseudoRange = satsWithPseudoRange;
    }

    static void pvtHeader(std::ostream& out) {
        out
            << left
            << setw(18) << "Time" << " "
            << setw(9) << "Lat" << " "
            << setw(9) << "Lon" << " "
            << setw(7) << "H+Ellip" << " "
            << setw(7) << "H+MSL" << " "
            << setw(7) << "Fix" << " "
            << setw(1) << "D" << " "
            << setw(5) << "RTK" << " "
            << setw(4) << "Sats" << " "
            << setw(4) << "S/C" << " "
            << setw(4) << "S/PR" << " "
            << setw(4) << "S/CR" << "\n";
    }

    void pvt(PVT const& pvt) override {
        string fix_type = FIX_TYPE_TO_STRING.at(pvt.fix_type);

        string differential = "N";
        if (pvt.fix_flags & PVT::FIX_DIFFERENTIAL) {
            differential = "Y";
        }

        string rtk = "N";
        if (pvt.fix_flags & PVT::FIX_RTK_FLOAT) {
            rtk = "Float";
        }
        else if (pvt.fix_flags & PVT::FIX_RTK_FIXED) {
            rtk = "Fixed";
        }


        std::cout
            << left
            << base::Time::now() << " "
            << fixed << setprecision(5)
            << setw(9) << pvt.latitude.getDeg() << " "
            << setw(9) << pvt.longitude.getDeg() << " "
            << fixed << setprecision(2)
            << setw(7) << pvt.height << " "
            << setw(7) << pvt.height_above_mean_sea_level << " "
            << setw(7) << fix_type << " "
            << setw(1) << differential << " "
            << setw(5) << rtk << " "
            << setw(4) << sats << " "
            << setw(4) << satsWithCorrections << " "
            << setw(4) << satsWithPseudoRange << " "
            << setw(4) << satsWithCarrierRange << "\n";
    }

    void rtcm(uint8_t const* buffer, size_t size) override {
        if (!rtcm_out) {
            return;
        }

        rtcm_out->writePacket(buffer, size);
    }

    void rtcmReceivedMessage(RTCMReceivedMessage const& message) override {
        cout << base::Time::now() << " RTCM " << message.message_type;

        if (message.flags & RTCMReceivedMessage::MESSAGE_NOT_USED) {
            cout << " NOT_USED";
        }
        else if (message.flags & RTCMReceivedMessage::MESSAGE_USED) {
            cout << " USED";
        }
        else {
            cout << " USAGE_UNKNOWN";
        }
        if (message.flags & RTCMReceivedMessage::CRC_FAILED) {
            cout << " CRC_FAILED";
        }

        if (message.reference_station_id != 0xffff) {
            cout << " ref_station_id=" << message.reference_station_id;
        }
        cout << "\n";
    }

    static void relposnedHeader(std::ostream& out) {
        out
            << left
            << setw(18) << "Time" << " "
            << setw(6) << "X" << " "
            << setw(6) << "Y" << " "
            << setw(6) << "Z" << " "
            << setw(6) << "L" << " "
            << setw(5) << "Hdg" << " "
            << setw(1) << "D" << " "
            << setw(5) << "RTK" << " "
            << setw(3) << "Mov" << " "
            << setw(4) << "Sats" << " "
            << setw(4) << "S/C" << " "
            << setw(4) << "S/PR" << " "
            << setw(4) << "S/CR" << "\n";
    }

    void relposned(RelPosNED const& data) override {
        string differential = "N";
        if (data.flags & RelPosNED::FLAGS_USE_DIFFERENTIAL) {
            differential = "Y";
        }

        string moving = "N";
        if (data.flags & RelPosNED::FLAGS_MOVING_BASE) {
            moving = "Y";
        }

        string rtk = "N";
        if (data.flags & RelPosNED::FLAGS_RTK_FLOAT) {
            rtk = "Float";
        }
        else if (data.flags & RelPosNED::FLAGS_RTK_FIXED) {
            rtk = "Fixed";
        }

        auto relpos = data.relative_position_NED;
        std::cout
            << left
            << base::Time::now() << " "
            << fixed << setprecision(2)
            << setw(6) << relpos.x() << " "
            << setw(6) << relpos.y() << " "
            << setw(6) << relpos.z() << " "
            << setw(6) << data.relative_position_length << " "
            << fixed << setprecision(1)
            << setw(5) << data.relative_position_heading.getDeg() << " "
            << setw(1) << differential << " "
            << setw(5) << rtk << " "
            << setw(3) << moving << " "
            << setw(4) << sats << " "
            << setw(4) << satsWithCorrections << " "
            << setw(4) << satsWithPseudoRange << " "
            << setw(4) << satsWithCarrierRange << "\n";
    }
};

void setDefaults(DevicePort port, Driver& driver, bool rtcm_in) {
    driver.setPortProtocol(port, DIRECTION_INPUT, PROTOCOL_UBX, true, false);
    driver.setPortProtocol(port, DIRECTION_INPUT, PROTOCOL_RTCM3X, rtcm_in, false);
    driver.setPortProtocol(port, DIRECTION_OUTPUT, PROTOCOL_UBX, true, false);
    driver.setPortProtocol(port, DIRECTION_OUTPUT, PROTOCOL_RTCM3X, false, false);
    driver.setPortProtocol(port, DIRECTION_OUTPUT, PROTOCOL_NMEA, false, false);
    driver.setOutputRate(port, MSGOUT_NAV_PVT, 0, false);
    driver.setOutputRate(port, MSGOUT_NAV_RELPOSNED, 0, false);
    driver.setOutputRate(port, MSGOUT_NAV_SAT, 0, false);
    driver.setOutputRate(port, MSGOUT_RXM_RTCM, 0, false);
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

    vector<uint8_t> rtcmBuffer;
    rtcmBuffer.reserve(4096);
    while(true) {
        int ret = poll(fds, poll_n, 2000);
        if (ret <= 0) {
            continue;
        }

        if (fds[1].revents & POLLIN) {
            size_t s = rtcm_in->readRaw(rtcmBuffer.data(), 4096);
            rtcmBuffer.resize(s);
            driver.writeRTCM(rtcmBuffer);
        }

        if (fds[0].revents & POLLIN) {
            driver.poll(callbacks);
        }
    }
}

DeviceProtocol protocolFromString(string const& arg) {
    try {
        return PROTOCOL_S_TO_E.at(arg);
    }
    catch(std::out_of_range&) {
        throw std::invalid_argument(arg + " is not a valid protocol name");
    }
}

DevicePort portFromString(string const& arg) {
    try {
        return PORT_S_TO_E.at(arg);
    }
    catch(std::out_of_range&) {
        throw std::invalid_argument(arg + " is not a valid port name");
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

    if (cmd == "cfg-reset") {
        driver.openURI(uri);
        driver.resetConfigurationToDefaults();
    }
    else if (cmd == "cfg-save") {
        driver.openURI(uri);
        driver.saveConfiguration();
    }
    else if (cmd == "enable-port") {
        if (argc != 5) {
            usage();
            return 1;
        }

        DevicePort port = portFromString(argv[3]);
        string enabled_s = argv[4];
        if (enabled_s != "on" && enabled_s != "off") {
            cerr << "ENABLED parameter must be 'on' or 'off'" << endl;
            return 1;
        }
        bool enable = enabled_s == "on";

        driver.openURI(uri);
        driver.setPortEnabled(port, enable);
    }
    else if (cmd == "cfg-uart") {
        if (argc != 5) {
            usage();
            return 1;
        }

        DevicePort port = portFromString(argv[3]);
        uint32_t rate = std::atoi(argv[4]);

        driver.openURI(uri);
        driver.setUARTBaudrate(port, rate, false);
    } else if ((cmd == "enable-input") || (cmd == "enable-output")) {
        bool input = cmd == "enable-input";

        if (argc != 6) {
            usage();
            return 1;
        }

        auto port = portFromString(argv[3]);
        string protocol = argv[4];
        string enabled_s = argv[5];
        if (enabled_s != "on" && enabled_s != "off") {
            cerr << "ENABLED parameter must be 'on' or 'off'" << endl;
            return 1;
        }
        bool enable = enabled_s == "on";

        driver.openURI(uri);
        driver.setPortProtocol(
            port, input ? DIRECTION_INPUT : DIRECTION_OUTPUT,
            protocolFromString(protocol), enable
        );
    } else if (cmd == "poll-solution") {
        string rtk_source;
        RTCMForwarder rtcm_in;

        if (argc == 5) {
            rtk_source = argv[4];
            rtcm_in.openURI(rtk_source);
        }
        else if (argc != 4) {
            usage();
            return 1;
        }

        auto port = portFromString(argv[3]);

        driver.openURI(uri);
        setDefaults(port, driver, !rtk_source.empty());
        driver.setOutputRate(port, MSGOUT_NAV_PVT, 1, false);
        driver.setOutputRate(port, MSGOUT_NAV_SAT, 5, false);
        driver.setReadTimeout(base::Time::fromSeconds(2));

        PollCallbacks::pvtHeader(cout);
        poll(driver, (rtcm_in.isValid() ? &rtcm_in : nullptr), nullptr);
    } else if (cmd == "poll-relposned") {
        bool monitor_rtcm = false;
        if (argc < 5 || argc > 6) {
            usage();
            return 1;
        }
        else if (argc == 6) {
            if (argv[5] != string("--monitor-rtcm")) {
                usage();
                return 1;
            }
            monitor_rtcm = true;
        }

        auto port = portFromString(argv[3]);
        string rtk_source = argv[4];
        RTCMForwarder rtcm_in;
        rtcm_in.openURI(rtk_source);

        driver.openURI(uri);
        setDefaults(port, driver, true);
        driver.setOutputRate(port, MSGOUT_NAV_RELPOSNED, 1, false);
        driver.setOutputRate(port, MSGOUT_NAV_SAT, 5, false);
        if (monitor_rtcm) {
            driver.setOutputRate(port, MSGOUT_RXM_RTCM, 1, false);
        }
        driver.setReadTimeout(base::Time::fromMilliseconds(2000));

        PollCallbacks::relposnedHeader(cout);
        poll(driver, &rtcm_in, nullptr);
    } else if (cmd == "reference-station") {
        if (argc != 5) {
            usage();
            return 1;
        }

        auto port = portFromString(argv[3]);
        string target = argv[4];
        RTCMForwarder rtcm_out;
        rtcm_out.openURI(target);

        driver.openURI(uri);
        setDefaults(port, driver, false);
        driver.setPortProtocol(port, DIRECTION_OUTPUT, PROTOCOL_RTCM3X, true, false);
        driver.setOutputRate(port, MSGOUT_NAV_PVT, 1, false);
        driver.setOutputRate(port, MSGOUT_NAV_SAT, 5, false);
        driver.setRTCMOutputRate(port, 1074, 1, false);
        driver.setRTCMOutputRate(port, 1084, 1, false);
        driver.setRTCMOutputRate(port, 1094, 1, false);
        driver.setRTCMOutputRate(port, 1124, 1, false);
        driver.setRTCMOutputRate(port, 1230, 1, false);
        driver.setRTCMOutputRate(port, 4072, 1, false);
        driver.setReadTimeout(base::Time::fromSeconds(2));

        PollCallbacks::pvtHeader(cout);
        poll(driver, nullptr, &rtcm_out);
    } else {
        cerr << "unexpected command " << cmd << endl;
        return usage();
    }
    return 0;
}
