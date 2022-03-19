#include <vector>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <gps_ublox/cfg.hpp>
#include <gps_ublox/UBX.hpp>

using namespace std;
using namespace gps_ublox;
using namespace UBX;

struct UBXTest : public ::testing::Test {
    vector<uint8_t> buffer;
    UBXTest() {
        buffer.resize(256, 0);
    }
};

TEST_F(UBXTest, it_waits_for_more_bytes_if_the_buffer_is_too_small) {
    ASSERT_EQ(0, extractPacket(&buffer[0], 7));
}
TEST_F(UBXTest, it_discards_a_byte_if_it_is_not_a_start_byte) {
    buffer[0] = 0x00;
    ASSERT_EQ(-1, extractPacket(&buffer[0], 8));
}
TEST_F(UBXTest, it_discards_a_byte_if_the_start_sequence_is_invalid) {
    buffer[0] = UBX::SYNC_1;
    buffer[1] = 0x00;
    ASSERT_EQ(-1, extractPacket(&buffer[0], 8));
}
TEST_F(UBXTest, it_discards_a_byte_if_the_packet_has_an_invalid_checksum) {
    buffer[0] = UBX::SYNC_1;
    buffer[1] = UBX::SYNC_2;
    buffer[2] = buffer[3] = buffer[4] = buffer[5] = 0;
    buffer[6] = buffer[7] = 1;
    ASSERT_EQ(-1, extractPacket(&buffer[0], 8));
}

/* Copied from device documentation */
static vector<uint8_t> checksum(const uint8_t *buffer, size_t size) {
    uint8_t ck_a = 0, ck_b = 0;
    for (size_t i = 0; i < size; i++) {
        ck_a = ck_a + buffer[i];
        ck_b = ck_b + ck_a;
    }
    return vector<uint8_t>{ck_a, ck_b};
}

static void insertChecksum(vector<uint8_t> &buffer) {
    vector<uint8_t> ck = checksum(&buffer[UBX::MSG_CLASS_IDX], buffer.size() - 2);
    buffer.push_back(ck[0]);
    buffer.push_back(ck[1]);
}

TEST_F(UBXTest, it_accepts_a_valid_zero_length_payload_packet) {
    buffer = { UBX::SYNC_1, UBX::SYNC_2, 0x01, 0x02, 0x00, 0x00 };
    insertChecksum(buffer);
    ASSERT_EQ(8, extractPacket(&buffer[0], 10));
}

TEST_F(UBXTest, it_waits_for_more_bytes_if_packet_is_incomplete) {
    buffer = { UBX::SYNC_1, UBX::SYNC_2, 0x01, 0x02, 0x01, 0x00, 0x00, 0x00 };
    ASSERT_EQ(0, extractPacket(&buffer[0], 8));
}

TEST_F(UBXTest, it_accepts_a_valid_full_packet) {
    buffer = { UBX::SYNC_1, UBX::SYNC_2, 0x01, 0x02, 0x02, 0x00, 0xAA, 0xBB };
    insertChecksum(buffer);
    ASSERT_EQ(10, extractPacket(&buffer[0], 12));
}

TEST_F(UBXTest, it_deserializes_a_packet) {
    buffer = { UBX::SYNC_1, UBX::SYNC_2, 0x01, 0x02, 0x02, 0x00, 0xAA, 0xBB, 0x00, 0x00 };
    UBX::Frame frame = UBX::Frame::fromPacket(&buffer[0], buffer.size());
    ASSERT_EQ(0x01, frame.msg_class);
    ASSERT_EQ(0x02, frame.msg_id);
    ASSERT_EQ(vector<uint8_t>({0xAA, 0xBB}), frame.payload);
}

TEST_F(UBXTest, it_serializes_a_frame) {
    UBX::Frame frame = { .msg_class = 0x01, .msg_id = 0x02, .payload = {0xAA, 0xBB} };
    vector<uint8_t> packet = frame.toPacket();
    buffer = { UBX::SYNC_1, UBX::SYNC_2, 0x01, 0x02, 0x02, 0x00, 0xAA, 0xBB };
    insertChecksum(buffer);
    ASSERT_EQ(buffer, packet);
}

TEST_F(UBXTest, it_returns_a_valset_all_layers_packet) {
    vector<uint8_t> packet = getConfigValueSetPacket(cfg::I2C_ENABLED, true);
    buffer = { UBX::SYNC_1, UBX::SYNC_2, UBX::MSG_CLASS_CFG, UBX::MSG_ID_VALSET, 0x09, 0x00,
               0x00, UBX::LAYER_ALL, 0x00, 0x00, 0x03, 0x00, 0x51, 0x10, 0x01 };

    insertChecksum(buffer);
    ASSERT_EQ(buffer, packet);
}

TEST_F(UBXTest, it_returns_a_valset_ram_layer_packet) {
    vector<uint8_t> packet = getConfigValueSetPacket(cfg::I2C_ENABLED, false, false);
    buffer = { UBX::SYNC_1, UBX::SYNC_2, UBX::MSG_CLASS_CFG, UBX::MSG_ID_VALSET, 0x09, 0x00,
               0x00, UBX::LAYER_RAM, 0x00, 0x00, 0x03, 0x00, 0x51, 0x10, 0x00 };

    insertChecksum(buffer);
    ASSERT_EQ(buffer, packet);
}

template<typename T>
void toLittleEndian(vector<uint8_t> &buffer, T value)
{
    uint8_t shifter = 0;
    T bytes = reinterpret_cast<const T&>(value);
    for (size_t i = 0; i < sizeof(T); i++) {
        buffer.push_back((bytes >> shifter) & 0xFF);
        shifter += 8;
    }
}

TEST_F(UBXTest, it_parses_a_pvt_frame) {
    vector<uint8_t> payload;
    toLittleEndian<uint32_t>(payload, 3600);
    toLittleEndian<uint16_t>(payload, 2019);
    toLittleEndian<uint8_t>(payload, 8);
    toLittleEndian<uint8_t>(payload, 20);
    toLittleEndian<uint8_t>(payload, 23);
    toLittleEndian<uint8_t>(payload, 15);
    toLittleEndian<uint8_t>(payload, 33);
    toLittleEndian<uint8_t>(payload, 1);
    toLittleEndian<uint32_t>(payload, 2860);
    toLittleEndian<int32_t>(payload, -1000);
    toLittleEndian<uint8_t>(payload, 4);
    toLittleEndian<uint8_t>(payload, 2);
    toLittleEndian<uint8_t>(payload, 3);
    toLittleEndian<uint8_t>(payload, 24);  // n_sats
    toLittleEndian<int32_t>(payload, 1202200000);  // latitude
    toLittleEndian<int32_t>(payload, 453000000);  // longitude
    toLittleEndian<int32_t>(payload, 2200);  // height
    toLittleEndian<int32_t>(payload, 5000);  // height above mean sea level
    toLittleEndian<uint32_t>(payload, 130);  // horizontal accuracy
    toLittleEndian<uint32_t>(payload, 270);  // vertical accuracy
    toLittleEndian<int32_t>(payload, 2300);  // vel_n
    toLittleEndian<int32_t>(payload, 3300);  // vel_e
    toLittleEndian<int32_t>(payload, 8400);  // vel_d
    toLittleEndian<int32_t>(payload, 7700);  // ground speed
    toLittleEndian<int32_t>(payload, 2370000);  // heading of motion
    toLittleEndian<uint32_t>(payload, 233);  // speed accuracy
    toLittleEndian<uint32_t>(payload, 83200);  // heading accuracy
    toLittleEndian<uint16_t>(payload, 421);  // position dop
    toLittleEndian<uint8_t>(payload, 7);  // additional flags
    toLittleEndian<uint8_t>(payload, 0);
    toLittleEndian<uint8_t>(payload, 0);
    toLittleEndian<uint8_t>(payload, 0);
    toLittleEndian<uint8_t>(payload, 0);
    toLittleEndian<uint8_t>(payload, 0);
    toLittleEndian<int32_t>(payload, 93000);  // heading of vehicle
    toLittleEndian<int16_t>(payload, 751);  // magnetic declination
    toLittleEndian<uint16_t>(payload, 531);  // magnetic declination accuracy

    PVT data = UBX::parsePVT(payload);
    ASSERT_EQ(3600, data.time_of_week);
    // Timestamp converted using https://www.epochconverter.com/
    ASSERT_EQ(1566342932999999ull, data.time.toMicroseconds());
    ASSERT_EQ(1, data.valid);
    ASSERT_EQ(2860, data.time_accuracy);
    ASSERT_EQ(PVT::GNSS_PLUS_DEAD_RECKONING, data.fix_type);
    ASSERT_EQ(2, data.fix_flags);
    ASSERT_EQ(3, data.additional_flags);
    ASSERT_EQ(24, data.num_sats);
    ASSERT_FLOAT_EQ(120.22, data.longitude.getDeg());
    ASSERT_FLOAT_EQ(45.3, data.latitude.getDeg());
    ASSERT_FLOAT_EQ(2.2, data.height);
    ASSERT_FLOAT_EQ(5.0, data.height_above_mean_sea_level);
    ASSERT_FLOAT_EQ(0.13, data.horizontal_accuracy);
    ASSERT_FLOAT_EQ(0.27, data.vertical_accuracy);
    ASSERT_FLOAT_EQ(2.3, data.vel_ned.x());
    ASSERT_FLOAT_EQ(3.3, data.vel_ned.y());
    ASSERT_FLOAT_EQ(8.4, data.vel_ned.z());
    ASSERT_FLOAT_EQ(7.7, data.ground_speed);
    ASSERT_FLOAT_EQ(23.7, data.heading_of_motion.getDeg());
    ASSERT_FLOAT_EQ(0.233, data.speed_accuracy);
    ASSERT_FLOAT_EQ(0.832, data.heading_accuracy.getDeg());
    ASSERT_FLOAT_EQ(4.21, data.position_dop);
    ASSERT_FLOAT_EQ(7, data.more_flags);
    ASSERT_FLOAT_EQ(0.93, data.heading_of_vehicle.getDeg());
    ASSERT_FLOAT_EQ(7.51, data.magnetic_declination.getDeg());
    ASSERT_FLOAT_EQ(5.31, data.magnetic_declination_accuracy.getDeg());
}

TEST_F(UBXTest, it_parses_an_rf_frame) {
    vector<uint8_t> payload;
    toLittleEndian<uint8_t>(payload, 0);
    toLittleEndian<uint8_t>(payload, 2);
    toLittleEndian<uint8_t>(payload, 0);
    toLittleEndian<uint8_t>(payload, 0);
    toLittleEndian<uint8_t>(payload, 12);
    toLittleEndian<uint8_t>(payload, 01);
    toLittleEndian<uint8_t>(payload, 1);
    toLittleEndian<uint8_t>(payload, 2);
    toLittleEndian<uint32_t>(payload, 1266);
    toLittleEndian<uint8_t>(payload, 0);
    toLittleEndian<uint8_t>(payload, 0);
    toLittleEndian<uint8_t>(payload, 0);
    toLittleEndian<uint8_t>(payload, 0);
    toLittleEndian<uint16_t>(payload, 25);
    toLittleEndian<uint16_t>(payload, 76);
    toLittleEndian<uint8_t>(payload, 25);
    toLittleEndian<int8_t>(payload, 87);
    toLittleEndian<uint8_t>(payload, 53);
    toLittleEndian<int8_t>(payload, 92);
    toLittleEndian<uint8_t>(payload, 43);
    payload.resize(4 + (24 * 2));
    payload[28] = 15;

    RFInfo data = UBX::parseRF(payload);
    ASSERT_EQ(0, data.version);
    ASSERT_EQ(2, data.n_blocks);
    ASSERT_EQ(12, data.blocks[0].block_id);
    ASSERT_EQ(RFInfo::JAMMING_OK, data.blocks[0].jamming_state);
    ASSERT_EQ(RFInfo::ANTENNA_STATUS_UNKNOWN, data.blocks[0].antenna_status);
    ASSERT_EQ(RFInfo::ANTENNA_POWER_UNKNOWN, data.blocks[0].antenna_power);
    ASSERT_EQ(1266, data.blocks[0].post_status);
    ASSERT_EQ(25, data.blocks[0].noise_per_measurement);
    ASSERT_EQ(76, data.blocks[0].agc_count);
    ASSERT_EQ(25, data.blocks[0].jamming_indicator);
    ASSERT_EQ(87, data.blocks[0].ofs_i);
    ASSERT_EQ(53, data.blocks[0].mag_i);
    ASSERT_EQ(92, data.blocks[0].ofs_q);
    ASSERT_EQ(43, data.blocks[0].mag_q);
    ASSERT_EQ(15, data.blocks[1].block_id);
}

TEST_F(UBXTest, it_parses_a_sig_frame) {
    vector<uint8_t> payload;
    toLittleEndian<uint32_t>(payload, 3600);
    toLittleEndian<uint8_t>(payload, 0);
    toLittleEndian<uint8_t>(payload, 2);
    toLittleEndian<uint8_t>(payload, 0);
    toLittleEndian<uint8_t>(payload, 0);

    toLittleEndian<uint8_t>(payload, 25);
    toLittleEndian<uint8_t>(payload, 21);
    toLittleEndian<uint8_t>(payload, 53);
    toLittleEndian<uint8_t>(payload, 43);
    toLittleEndian<int16_t>(payload, 120);
    toLittleEndian<uint8_t>(payload, 11);
    toLittleEndian<uint8_t>(payload, 31);
    toLittleEndian<uint8_t>(payload, 62);
    toLittleEndian<uint8_t>(payload, 74);
    toLittleEndian<int16_t>(payload, 112);

    payload.resize(8 + (16 * 2));
    payload[24] = 15;

    SignalInfo data = UBX::parseSIG(payload);
    ASSERT_EQ(3600, data.time_of_week);
    ASSERT_EQ(0, data.version);
    ASSERT_EQ(2, data.n_signals);

    ASSERT_EQ(25, data.signals[0].gnss_id);
    ASSERT_EQ(21, data.signals[0].satellite_id);
    ASSERT_EQ(53, data.signals[0].signal_id);
    ASSERT_EQ(43, data.signals[0].frequency_id);
    ASSERT_EQ(12, data.signals[0].pseudorange_residual);
    ASSERT_EQ(11, data.signals[0].signal_strength);
    ASSERT_EQ(31, data.signals[0].quality_indicator);
    ASSERT_EQ(62, data.signals[0].correction_source);
    ASSERT_EQ(74, data.signals[0].ionospheric_model);
    ASSERT_EQ(112, data.signals[0].signal_flags);
    ASSERT_EQ(15, data.signals[1].gnss_id);
}


TEST_F(UBXTest, it_parses_a_sat_frame) {
    vector<uint8_t> payload;
    toLittleEndian<uint32_t>(payload, 3600);
    toLittleEndian<uint8_t>(payload, 0);
    toLittleEndian<uint8_t>(payload, 2);
    toLittleEndian<uint8_t>(payload, 0);
    toLittleEndian<uint8_t>(payload, 0);

    toLittleEndian<uint8_t>(payload, 25); // gnss id
    toLittleEndian<uint8_t>(payload, 21); // satellite id
    toLittleEndian<uint8_t>(payload, 53); // signal strength
    toLittleEndian<int8_t>(payload, 43); // elevation
    toLittleEndian<int16_t>(payload, 120); // azimuth
    toLittleEndian<int16_t>(payload, 31);  // pseudorange residual
    toLittleEndian<uint32_t>(payload, 112); // flags

    payload.resize(8 + (12 * 2));
    payload[20] = 15;

    SatelliteInfo data = UBX::parseSAT(payload);
    ASSERT_EQ(3600, data.time_of_week);
    ASSERT_EQ(0, data.version);

    ASSERT_EQ(2, data.signals.size());
    ASSERT_EQ(25, data.signals[0].gnss_id);
    ASSERT_EQ(21, data.signals[0].satellite_id);
    ASSERT_FLOAT_EQ(43, data.signals[0].elevation.getDeg());
    ASSERT_FLOAT_EQ(120, data.signals[0].azimuth.getDeg());
    ASSERT_FLOAT_EQ(3.1, data.signals[0].pseudorange_residual);
    ASSERT_EQ(53, data.signals[0].signal_strength);
    ASSERT_EQ(112, data.signals[0].signal_flags);
    ASSERT_EQ(15, data.signals[1].gnss_id);
}

TEST_F(UBXTest, it_rejects_a_relposned_payload_that_is_too_small) {
    vector<uint8_t> payload;
    payload.resize(63);
    ASSERT_THROW(UBX::parseRelPosNED(payload), std::invalid_argument);
}

TEST_F(UBXTest, it_rejects_a_relposned_payload_that_is_too_big) {
    vector<uint8_t> payload;
    payload.resize(65);
    ASSERT_THROW(UBX::parseRelPosNED(payload), std::invalid_argument);
}

TEST_F(UBXTest, it_parses_a_relposned_frame) {
    vector<uint8_t> payload;
    toLittleEndian<uint8_t>(payload, 1); // version
    toLittleEndian<uint8_t>(payload, 0); // reserved0
    toLittleEndian<uint16_t>(payload, 152); // reference station ID
    toLittleEndian<uint32_t>(payload, 3600); // time of week
    toLittleEndian<int32_t>(payload, -52); // N (cm)
    toLittleEndian<int32_t>(payload, 123); // E (cm)
    toLittleEndian<int32_t>(payload, 28); // D (cm)
    toLittleEndian<int32_t>(payload, 32); // Length (cm)
    toLittleEndian<int32_t>(payload, 20010403ul); // Heading (1e-5 deg)
    toLittleEndian<uint32_t>(payload, 0); // reserved1
    toLittleEndian<int8_t>(payload, -90); // N (0.1mm)
    toLittleEndian<int8_t>(payload, 25); // E (0.1mm)
    toLittleEndian<int8_t>(payload, -44); // D (0.1mm)
    toLittleEndian<int8_t>(payload, -42); // Length (0.1mm)
    toLittleEndian<uint32_t>(payload, 212); // Accuracy N (0.1mm)
    toLittleEndian<uint32_t>(payload, 4221); // Accuracy E (0.1mm)
    toLittleEndian<uint32_t>(payload, 8353); // Accuracy D (0.1mm)
    toLittleEndian<uint32_t>(payload, 244); // Accuracy Length (0.1mm)
    toLittleEndian<uint32_t>(payload, 4030201ul); // Accuracy Heading (1e-5deg)
    toLittleEndian<uint32_t>(payload, 0); // reserved2
    toLittleEndian<uint32_t>(payload, 1 | 2 | 4 | 256 | 512); // flags

    auto data = UBX::parseRelPosNED(payload);

    ASSERT_EQ(3600, data.time_of_week);
    ASSERT_EQ(152, data.reference_station_id);
    ASSERT_FLOAT_EQ(-0.52 - 0.0090, data.relative_position_NED.x());
    ASSERT_FLOAT_EQ(1.23 + 0.0025, data.relative_position_NED.y());
    ASSERT_FLOAT_EQ(0.28 - 0.0044, data.relative_position_NED.z());
    ASSERT_FLOAT_EQ(0.32 - 0.0042, data.relative_position_length);
    ASSERT_FLOAT_EQ(200.10403 - 360, data.relative_position_heading.getDeg());

    ASSERT_FLOAT_EQ(0.0212, data.accuracy_NED.x());
    ASSERT_FLOAT_EQ(0.4221, data.accuracy_NED.y());
    ASSERT_FLOAT_EQ(0.8353, data.accuracy_NED.z());
    ASSERT_FLOAT_EQ(0.0244, data.accuracy_length);
    ASSERT_FLOAT_EQ(40.30201, data.accuracy_heading.getDeg());

    ASSERT_EQ(1 | 2 | 4 | 256 | 512, data.flags);
}

TEST_F(UBXTest, it_does_not_fill_the_relative_position_fields_if_it_is_invalid) {
    vector<uint8_t> payload;
    toLittleEndian<uint8_t>(payload, 1); // version
    toLittleEndian<uint8_t>(payload, 0); // reserved0
    toLittleEndian<uint16_t>(payload, 152); // reference station ID
    toLittleEndian<uint32_t>(payload, 3600); // time of week
    toLittleEndian<int32_t>(payload, -52); // N (cm)
    toLittleEndian<int32_t>(payload, 123); // E (cm)
    toLittleEndian<int32_t>(payload, 28); // D (cm)
    toLittleEndian<int32_t>(payload, 32); // Length (cm)
    toLittleEndian<int32_t>(payload, 20010403ul); // Heading (1e-5 deg)
    toLittleEndian<uint32_t>(payload, 0); // reserved1
    toLittleEndian<int8_t>(payload, -90); // N (0.1mm)
    toLittleEndian<int8_t>(payload, 25); // E (0.1mm)
    toLittleEndian<int8_t>(payload, -44); // D (0.1mm)
    toLittleEndian<int8_t>(payload, -42); // Length (0.1mm)
    toLittleEndian<uint32_t>(payload, 212); // Accuracy N (0.1mm)
    toLittleEndian<uint32_t>(payload, 4221); // Accuracy E (0.1mm)
    toLittleEndian<uint32_t>(payload, 8353); // Accuracy D (0.1mm)
    toLittleEndian<uint32_t>(payload, 244); // Accuracy Length (0.1mm)
    toLittleEndian<uint32_t>(payload, 4030201ul); // Accuracy Heading (1e-5deg)
    toLittleEndian<uint32_t>(payload, 0); // reserved2
    toLittleEndian<uint32_t>(payload, 1 | 2 | 256 | 512); // flags, 4 == relative position valid, 256 = relative position heading valid

    auto data = UBX::parseRelPosNED(payload);

    ASSERT_EQ(3600, data.time_of_week);
    ASSERT_EQ(152, data.reference_station_id);
    ASSERT_TRUE(base::isUnknown(data.relative_position_NED.x()));
    ASSERT_TRUE(base::isUnknown(data.relative_position_NED.y()));
    ASSERT_TRUE(base::isUnknown(data.relative_position_NED.z()));
    ASSERT_FLOAT_EQ(0.32 - 0.0042, data.relative_position_length);
    ASSERT_FLOAT_EQ(200.10403 - 360, data.relative_position_heading.getDeg());

    ASSERT_TRUE(base::isUnknown(data.accuracy_NED.x()));
    ASSERT_TRUE(base::isUnknown(data.accuracy_NED.y()));
    ASSERT_TRUE(base::isUnknown(data.accuracy_NED.z()));
    ASSERT_FLOAT_EQ(0.0244, data.accuracy_length);
    ASSERT_FLOAT_EQ(40.30201, data.accuracy_heading.getDeg());

    ASSERT_EQ(1 | 2 | 256 | 512, data.flags);
}

TEST_F(UBXTest, it_does_not_fill_the_relative_position_length_and_angle_if_they_are_invalid) {
    vector<uint8_t> payload;
    toLittleEndian<uint8_t>(payload, 1); // version
    toLittleEndian<uint8_t>(payload, 0); // reserved0
    toLittleEndian<uint16_t>(payload, 152); // reference station ID
    toLittleEndian<uint32_t>(payload, 3600); // time of week
    toLittleEndian<int32_t>(payload, -52); // N (cm)
    toLittleEndian<int32_t>(payload, 123); // E (cm)
    toLittleEndian<int32_t>(payload, 28); // D (cm)
    toLittleEndian<int32_t>(payload, 32); // Length (cm)
    toLittleEndian<int32_t>(payload, 20010403ul); // Heading (1e-5 deg)
    toLittleEndian<uint32_t>(payload, 0); // reserved1
    toLittleEndian<int8_t>(payload, -90); // N (0.1mm)
    toLittleEndian<int8_t>(payload, 25); // E (0.1mm)
    toLittleEndian<int8_t>(payload, -44); // D (0.1mm)
    toLittleEndian<int8_t>(payload, -42); // Length (0.1mm)
    toLittleEndian<uint32_t>(payload, 212); // Accuracy N (0.1mm)
    toLittleEndian<uint32_t>(payload, 4221); // Accuracy E (0.1mm)
    toLittleEndian<uint32_t>(payload, 8353); // Accuracy D (0.1mm)
    toLittleEndian<uint32_t>(payload, 244); // Accuracy Length (0.1mm)
    toLittleEndian<uint32_t>(payload, 4030201ul); // Accuracy Heading (1e-5deg)
    toLittleEndian<uint32_t>(payload, 0); // reserved2
    toLittleEndian<uint32_t>(payload, 1 | 2 | 4 | 512); // flags

    auto data = UBX::parseRelPosNED(payload);

    ASSERT_EQ(3600, data.time_of_week);
    ASSERT_EQ(152, data.reference_station_id);
    ASSERT_FLOAT_EQ(-0.52 - 0.0090, data.relative_position_NED.x());
    ASSERT_FLOAT_EQ(1.23 + 0.0025, data.relative_position_NED.y());
    ASSERT_FLOAT_EQ(0.28 - 0.0044, data.relative_position_NED.z());
    ASSERT_TRUE(base::isUnknown(data.relative_position_length));
    ASSERT_TRUE(base::isUnknown(data.relative_position_heading.getDeg()));

    ASSERT_FLOAT_EQ(0.0212, data.accuracy_NED.x());
    ASSERT_FLOAT_EQ(0.4221, data.accuracy_NED.y());
    ASSERT_FLOAT_EQ(0.8353, data.accuracy_NED.z());
    ASSERT_TRUE(base::isUnknown(data.accuracy_length));
    ASSERT_TRUE(base::isUnknown(data.accuracy_heading.getDeg()));

    ASSERT_EQ(1 | 2 | 4 | 512, data.flags);
}
