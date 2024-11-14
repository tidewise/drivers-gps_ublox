rock_library(gps_ublox
    SOURCES Driver.cpp UBX.cpp Driver.cpp cfg.cpp RTKInfo.cpp chrony.cpp
    HEADERS Driver.hpp UBX.hpp Driver.hpp
      BoardInfo.hpp SignalInfo.hpp RFInfo.hpp PVT.hpp SatelliteInfo.hpp
      cfg.hpp RelPosNED.hpp RTCMReceivedMessage.hpp RTKInfo.hpp CommsInfo.hpp
      TimeUTC.hpp TimingPulseData.hpp chrony.hpp
    DEPS_PKGCONFIG iodrivers_base base-types gps_base
)

include(CheckIncludeFileCXX)
check_include_file_cxx("sys/timepps.h" HAVE_SYS_TIMEPPS)
if (HAVE_SYS_TIMEPPS)
    target_compile_definitions(gps_ublox PRIVATE HAVE_SYS_TIMEPPS)
endif()

find_package(Threads REQUIRED)
rock_executable(gps_ublox_ctl Main.cpp
    DEPS gps_ublox
    DEPS_TARGET Threads::Threads)
