rock_library(gps_ublox
    SOURCES Driver.cpp protocols/UBX.cpp Driver.cpp
    HEADERS Driver.hpp protocols/Parser.hpp protocols/UBX.hpp Driver.hpp
    DEPS_PKGCONFIG iodrivers_base base-types)

rock_executable(gps_ublox_ctl Main.cpp
    DEPS gps_ublox)
