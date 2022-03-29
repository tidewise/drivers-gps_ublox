#include <gtest/gtest.h>
#include <gps_ublox/PVT.hpp>
#include <gps_ublox/RTCMReceivedMessage.hpp>
#include <gps_ublox/RTKInfo.hpp>
#include <gps_ublox/SatelliteInfo.hpp>

using namespace gps_ublox;

struct RTKInfoTest : public ::testing::Test {
};

TEST_F(RTKInfoTest, update_sets_time_from_PVT_time) {
    RTKInfo info;
    PVT pvt;
    pvt.time = base::Time::now();
    pvt.fix_flags = 0;
    info.update(pvt);
    ASSERT_EQ(pvt.time, info.time);
}

TEST_F(RTKInfoTest, update_sets_solution_type_to_RTK_NONE_if_the_PVT_reports_no_RTK) {
    RTKInfo info;
    PVT pvt;
    pvt.fix_flags = 0;
    info.update(pvt);
    ASSERT_EQ(RTKInfo::RTK_NONE, info.solution_type);
}

TEST_F(RTKInfoTest, update_sets_solution_type_to_RTK_NONE_if_the_PVT_reports_differential) {
    RTKInfo info;
    PVT pvt;
    pvt.fix_flags = PVT::FIX_DIFFERENTIAL;
    info.update(pvt);
    ASSERT_EQ(RTKInfo::RTK_NONE, info.solution_type);
}

TEST_F(RTKInfoTest, update_sets_solution_type_to_RTK_FLOAT_if_the_PVT_reports_it) {
    RTKInfo info;
    PVT pvt;
    pvt.fix_flags = PVT::FIX_RTK_FLOAT;
    info.update(pvt);
    ASSERT_EQ(RTKInfo::RTK_FLOAT, info.solution_type);
}

TEST_F(RTKInfoTest, update_sets_solution_type_to_RTK_FIXED_if_the_PVT_reports_it) {
    RTKInfo info;
    PVT pvt;
    pvt.fix_flags = PVT::FIX_RTK_FIXED;
    info.update(pvt);
    ASSERT_EQ(RTKInfo::RTK_FIXED, info.solution_type);
}

TEST_F(RTKInfoTest, update_categorizes_the_state_of_each_satellite) {
    RTKInfo info;
    SatelliteInfo sats;
    sats.signals.resize(6);
    sats.signals[0].signal_flags = ~8; // not used, ignored
    sats.signals[1].signal_flags = 8; // used
    sats.signals[2].signal_flags = 8 | 1 << 20; // pseudorange
    sats.signals[3].signal_flags = 8 | 1 << 21; // carrier range
    sats.signals[4].signal_flags = 8 | 1 << 20 | 1 << 21; // both pseudo and carrier
    sats.signals[5].signal_flags = 8 | 1 << 21; // both pseudo and carrier
    info.update(sats);

    ASSERT_EQ(5, info.satellites_in_use);
    ASSERT_EQ(2, info.satellites_with_pseudorange);
    ASSERT_EQ(3, info.satellites_with_carrier_range);
}

TEST_F(RTKInfoTest, update_adds_a_new_entry_for_a_never_seen_used_RTCM_message) {
    RTCMReceivedMessage msg;
    msg.message_type = 10;
    msg.reference_station_id = 1;
    msg.flags = RTCMReceivedMessage::MESSAGE_USED;

    RTKInfo info;
    info.update(msg);

    ASSERT_EQ(1, info.rtcm_message_stats.size());
    auto entry = info.rtcm_message_stats[0];
    ASSERT_EQ(10, entry.message_type);
    ASSERT_EQ(1, entry.reference_station_id);
    ASSERT_EQ(1, entry.received);
    ASSERT_EQ(1, entry.used);
    ASSERT_EQ(0, entry.rejected_crc);
}

TEST_F(RTKInfoTest, update_modifies_an_existing_entry_for_a_used_RTCM_message) {
    RTCMReceivedMessage msg;
    msg.message_type = 10;
    msg.reference_station_id = 1;
    msg.flags = RTCMReceivedMessage::MESSAGE_USED;

    RTKInfo info;
    info.update(msg);
    info.update(msg);

    ASSERT_EQ(1, info.rtcm_message_stats.size());
    auto entry = info.rtcm_message_stats[0];
    ASSERT_EQ(10, entry.message_type);
    ASSERT_EQ(1, entry.reference_station_id);
    ASSERT_EQ(2, entry.received);
    ASSERT_EQ(2, entry.used);
    ASSERT_EQ(0, entry.rejected_crc);
}

TEST_F(RTKInfoTest, update_adds_a_new_entry_for_a_never_seen_unused_RTCM_message) {
    RTCMReceivedMessage msg;
    msg.message_type = 10;
    msg.reference_station_id = 1;
    msg.flags = 0;

    RTKInfo info;
    info.update(msg);

    ASSERT_EQ(1, info.rtcm_message_stats.size());
    auto entry = info.rtcm_message_stats[0];
    ASSERT_EQ(10, entry.message_type);
    ASSERT_EQ(1, entry.reference_station_id);
    ASSERT_EQ(1, entry.received);
    ASSERT_EQ(0, entry.used);
    ASSERT_EQ(0, entry.rejected_crc);
}

TEST_F(RTKInfoTest, update_modifies_an_existing_entry_for_an_unused_RTCM_message) {
    RTCMReceivedMessage msg;
    msg.message_type = 10;
    msg.reference_station_id = 1;
    msg.flags = 0;

    RTKInfo info;
    info.update(msg);
    info.update(msg);

    ASSERT_EQ(1, info.rtcm_message_stats.size());
    auto entry = info.rtcm_message_stats[0];
    ASSERT_EQ(10, entry.message_type);
    ASSERT_EQ(1, entry.reference_station_id);
    ASSERT_EQ(2, entry.received);
    ASSERT_EQ(0, entry.used);
    ASSERT_EQ(0, entry.rejected_crc);
}

TEST_F(RTKInfoTest, update_adds_a_new_entry_for_a_never_seen_RTCM_message_that_has_not_passed_CRC) {
    RTCMReceivedMessage msg;
    msg.message_type = 10;
    msg.reference_station_id = 1;
    msg.flags = RTCMReceivedMessage::CRC_FAILED;

    RTKInfo info;
    info.update(msg);

    ASSERT_EQ(1, info.rtcm_message_stats.size());
    auto entry = info.rtcm_message_stats[0];
    ASSERT_EQ(10, entry.message_type);
    ASSERT_EQ(1, entry.reference_station_id);
    ASSERT_EQ(1, entry.received);
    ASSERT_EQ(0, entry.used);
    ASSERT_EQ(1, entry.rejected_crc);
}

TEST_F(RTKInfoTest, update_modifies_an_existing_entry_for_a_RTCM_message_that_has_not_passed_CRC) {
    RTCMReceivedMessage msg;
    msg.message_type = 10;
    msg.reference_station_id = 1;
    msg.flags = RTCMReceivedMessage::CRC_FAILED;

    RTKInfo info;
    info.update(msg);
    info.update(msg);

    ASSERT_EQ(1, info.rtcm_message_stats.size());
    auto entry = info.rtcm_message_stats[0];
    ASSERT_EQ(10, entry.message_type);
    ASSERT_EQ(1, entry.reference_station_id);
    ASSERT_EQ(2, entry.received);
    ASSERT_EQ(0, entry.used);
    ASSERT_EQ(2, entry.rejected_crc);
}

TEST_F(RTKInfoTest, update_categories_entries_by_both_message_type_and_reference_station) {
    RTCMReceivedMessage msg;
    msg.message_type = 10;
    msg.reference_station_id = 1;
    msg.flags = 0;

    RTKInfo info;
    info.update(msg);

    msg.reference_station_id = 2;
    info.update(msg);

    ASSERT_EQ(2, info.rtcm_message_stats.size());

    auto entry = info.rtcm_message_stats[0];
    ASSERT_EQ(10, entry.message_type);
    ASSERT_EQ(1, entry.reference_station_id);
    ASSERT_EQ(1, entry.received);

    entry = info.rtcm_message_stats[1];
    ASSERT_EQ(10, entry.message_type);
    ASSERT_EQ(2, entry.reference_station_id);
    ASSERT_EQ(1, entry.received);
}

TEST_F(RTKInfoTest, addRX_add_bytes_to_rx) {
    RTKInfo info;
    info.addRX(10);
    info.addRX(15);
    ASSERT_EQ(25, info.rx);
}

TEST_F(RTKInfoTest, addTX_add_bytes_to_tx) {
    RTKInfo info;
    info.addTX(10);
    info.addTX(15);
    ASSERT_EQ(25, info.tx);
}
