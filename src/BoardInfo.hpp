#ifndef GPS_UBLOX_BOARDINFO_HPP
#define GPS_UBLOX_BOARDINFO_HPP

#include <string>
#include <vector>

namespace gps_ublox {
    /**
     * Information about the board
     */
    struct BoardInfo {
        std::string software_version;
        std::string hardware_version;

        std::vector<std::string> extensions;
    };
}

#endif