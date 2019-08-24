rock_library(gps_ublox
    SOURCES Driver.cpp UBX.cpp Driver.cpp
    HEADERS Driver.hpp UBX.hpp Driver.hpp BoardInfo.hpp
    DEPS_PKGCONFIG iodrivers_base base-types gps_base)

rock_executable(gps_ublox_ctl Main.cpp
    DEPS gps_ublox)
