rock_library(gps_ublox
    SOURCES Driver.cpp UBX.cpp Driver.cpp
    HEADERS Driver.hpp UBX.hpp Driver.hpp
      BoardInfo.hpp SignalInfo.hpp RFInfo.hpp GPSData.hpp
    DEPS_PKGCONFIG iodrivers_base base-types)

rock_executable(gps_ublox_ctl Main.cpp
    DEPS gps_ublox)
