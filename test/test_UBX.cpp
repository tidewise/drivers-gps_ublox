#include <vector>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

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
    vector<uint8_t> packet = getConfigValueSetPacket(UBX::I2C_ENABLED, true);
    buffer = { UBX::SYNC_1, UBX::SYNC_2, UBX::MSG_CLASS_CFG, UBX::MSG_ID_VALSET, 0x09, 0x00,
               0x00, UBX::LAYER_ALL, 0x00, 0x00, 0x03, 0x00, 0x51, 0x10, 0x01 };

    insertChecksum(buffer);
    ASSERT_EQ(buffer, packet);
}

TEST_F(UBXTest, it_returns_a_valset_ram_layer_packet) {
    vector<uint8_t> packet = getConfigValueSetPacket(UBX::I2C_ENABLED, false, false);
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
    toLittleEndian<int32_t>(payload, 8215);
    toLittleEndian<uint8_t>(payload, 4);
    toLittleEndian<uint8_t>(payload, 2);
    toLittleEndian<uint8_t>(payload, 3);
    toLittleEndian<uint8_t>(payload, 24);
    toLittleEndian<int32_t>(payload, 12000);
    toLittleEndian<int32_t>(payload, 45000);
    toLittleEndian<int32_t>(payload, 2200);
    toLittleEndian<int32_t>(payload, 5000);
    toLittleEndian<uint32_t>(payload, 130);
    toLittleEndian<uint32_t>(payload, 270);
    toLittleEndian<int32_t>(payload, 2300);
    toLittleEndian<int32_t>(payload, 3300);
    toLittleEndian<int32_t>(payload, 8400);
    toLittleEndian<int32_t>(payload, 7700);
    toLittleEndian<int32_t>(payload, 237);
    toLittleEndian<uint32_t>(payload, 233);
    toLittleEndian<uint32_t>(payload, 832);
    toLittleEndian<uint16_t>(payload, 421);
    toLittleEndian<uint8_t>(payload, 7);
    toLittleEndian<uint8_t>(payload, 0);
    toLittleEndian<uint8_t>(payload, 0);
    toLittleEndian<uint8_t>(payload, 0);
    toLittleEndian<uint8_t>(payload, 0);
    toLittleEndian<uint8_t>(payload, 0);
    toLittleEndian<int32_t>(payload, 9300);
    toLittleEndian<int16_t>(payload, 751);
    toLittleEndian<uint16_t>(payload, 531);

    GPSData data = UBX::parsePvt(payload);
    ASSERT_EQ(3600, data.time_of_week);
    ASSERT_EQ(2019, data.year);
    ASSERT_EQ(8, data.month);
    ASSERT_EQ(20, data.day);
    ASSERT_EQ(23, data.hour);
    ASSERT_EQ(15, data.min);
    ASSERT_EQ(33, data.sec);
    ASSERT_EQ(1, data.valid);
    ASSERT_EQ(2860, data.time_accuracy);
    ASSERT_EQ(8215, data.fraction);
    ASSERT_EQ(4, data.fix_type);
    ASSERT_EQ(2, data.fix_flags);
    ASSERT_EQ(3, data.additional_flags);
    ASSERT_EQ(24, data.num_sats);
    ASSERT_EQ(12000, data.longitude);
    ASSERT_EQ(45000, data.latitude);
    ASSERT_EQ(2200, data.height);
    ASSERT_EQ(5000, data.height_above_mean_sea_level);
    ASSERT_EQ(130, data.horizontal_accuracy);
    ASSERT_EQ(270, data.vertical_accuracy);
    ASSERT_EQ(2300, data.vel_north);
    ASSERT_EQ(3300, data.vel_east);
    ASSERT_EQ(8400, data.vel_down);
    ASSERT_EQ(7700, data.ground_speed);
    ASSERT_EQ(237, data.heading_of_motion);
    ASSERT_EQ(233, data.speed_accuracy);
    ASSERT_EQ(832, data.heading_accuracy);
    ASSERT_EQ(421, data.position_dop);
    ASSERT_EQ(7, data.more_flags);
    ASSERT_EQ(9300, data.heading_of_vehicle);
    ASSERT_EQ(751, data.magnetic_declination);
    ASSERT_EQ(531, data.magnetic_declination_accuracy);
}
