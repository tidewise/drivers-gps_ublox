#include <iostream>
#include <gps_ublox/Driver.hpp>
#include <map>
#include <iomanip>

using namespace std;
using namespace gps_ublox;

static const map<string, Driver::DevicePort> PORT_S_TO_E = {
    { "i2c", Driver::PORT_I2C },
    { "spi", Driver::PORT_SPI },
    { "uart1", Driver::PORT_UART1 },
    { "uart2", Driver::PORT_UART2 },
    { "usb", Driver::PORT_USB }
};

static const map<string, Driver::DeviceProtocol> PROTOCOL_S_TO_E = {
    { "ubx", Driver::PROTOCOL_UBX },
    { "nmea", Driver::PROTOCOL_NMEA },
    { "rtcm3x", Driver::PROTOCOL_RTCM3X }
};

static const map<GPSData::GNSSFixType, string> FIX_TO_STRING = {
    { GPSData::NO_FIX, "None" },
    { GPSData::FIX_2D, "2D" },
    { GPSData::FIX_3D, "3D" }
};

static const map<GPSData::GNSSFixFlags, string> FIX_FLAG_TO_STRING = {
    { GPSData::FIX_DIFFERENTIAL, "D" },
    { GPSData::FIX_RTK_FLOAT, "FloatRTK" },
    { GPSData::FIX_RTK_FIXED, "FixRTK" }
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
                               input ? Driver::DIRECTION_INPUT : Driver::DIRECTION_OUTPUT,
                               PROTOCOL_S_TO_E.at(protocol),
                               enable);
    } else if (cmd == "poll-solution") {
        driver.openURI(uri);
        driver.setPortProtocol(Driver::PORT_USB, Driver::DIRECTION_OUTPUT, Driver::PROTOCOL_UBX, true, false);
        driver.setOutputRate(Driver::PORT_USB, Driver::MSGOUT_NAV_PVT, 1, false);
        driver.setReadTimeout(base::Time::fromSeconds(2));

        cout << "Lat Lon Height HAboveMSL Fix" << endl;

        while(true) {
            auto data = driver.readGPSData();
            string fix_type = FIX_TO_STRING.at(data.fix_type);
            for (auto const& flag: FIX_FLAG_TO_STRING) {
                if ((flag.first & data.fix_flags) == flag.first) {
                    fix_type += "/" + flag.second;
                }
            }

            std::cout
                << data.latitude.getDeg() << " " << data.longitude.getDeg() << " "
                << data.height << " " << data.height_above_mean_sea_level << " "
                << fix_type << "\n";
        }
    } else {
        cerr << "unexpected command " << cmd << endl;
        return usage();
    }
    return 0;
}