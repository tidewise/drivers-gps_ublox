#include <gps_ublox/RTKInfo.hpp>

#include <gps_ublox/PVT.hpp>
#include <gps_ublox/RTCMReceivedMessage.hpp>
#include <gps_ublox/SatelliteInfo.hpp>

using namespace gps_ublox;

void RTKInfo::addTX(size_t bytes) {
    tx += bytes;
}

void RTKInfo::addRX(size_t bytes) {
    rx += bytes;
}

void RTKInfo::update(PVT const& pvt) {
    time = pvt.time;
    if (pvt.fix_flags & PVT::FIX_RTK_FIXED) {
        solution_type = RTKInfo::RTK_FIXED;
    }
    else if (pvt.fix_flags & PVT::FIX_RTK_FLOAT) {
        solution_type = RTKInfo::RTK_FLOAT;
    }
    else {
        solution_type = RTKInfo::RTK_NONE;
    }
}

void RTKInfo::update(SatelliteInfo const& info) {
    uint8_t satellites_in_use = 0;
    uint8_t satellites_with_pseudorange = 0;
    uint8_t satellites_with_carrier_range = 0;

    for (auto& sat: info.signals) {
        if (!(sat.signal_flags & SatelliteInfo::USED_IN_SOLUTION)) {
            continue;
        }

        satellites_in_use++;
        if (sat.signal_flags & SatelliteInfo::PSEUDORANGE_CORRECTIONS_USED) {
            satellites_with_pseudorange++;
        }
        if (sat.signal_flags & SatelliteInfo::CARRIER_RANGE_CORRECTIONS_USED) {
            satellites_with_carrier_range++;
        }
    }

    this->satellites_in_use = satellites_in_use;
    this->satellites_with_pseudorange = satellites_with_pseudorange;
    this->satellites_with_carrier_range = satellites_with_carrier_range;
}

void RTKInfo::update(RTCMReceivedMessage const& msg) {
    auto it = std::find_if(
        rtcm_message_stats.begin(), rtcm_message_stats.end(),
        [&msg](RTCMMessageStats& stat) {
            return stat.message_type == msg.message_type &&
                stat.reference_station_id == msg.reference_station_id;
        }
    );

    if (it == rtcm_message_stats.end()) {
        RTCMMessageStats newEntry;
        newEntry.message_type = msg.message_type;
        newEntry.reference_station_id = msg.reference_station_id;
        rtcm_message_stats.push_back(newEntry);
        it = rtcm_message_stats.begin() + (rtcm_message_stats.size() - 1);
    }

    it->received++;
    if (msg.flags & RTCMReceivedMessage::MESSAGE_USED) {
        it->used++;
    }
    if (msg.flags & RTCMReceivedMessage::CRC_FAILED) {
        it->rejected_crc++;
    }
}
