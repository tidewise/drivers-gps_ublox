rock_library(gps_ublox
    SOURCES Driver.cpp
    HEADERS Driver.hpp
    DEPS_PKGCONFIG iodrivers_base)

rock_executable(gps_ublox_cli Main.cpp
    DEPS gps_ublox)

