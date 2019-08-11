#include <vector>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <gps_ublox/protocols/UBX.hpp>

using namespace std;
using namespace gps_ublox::protocols;

struct UBXTest : public ::testing::Test {
    vector<uint8_t> buffer;
    UBXTest() {
        buffer.resize(256, 0);
    }
    UBX ubx;
};

TEST_F(UBXTest, it_waits_for_more_bytes_if_the_buffer_is_too_small) {
    ASSERT_EQ(0, ubx.extractPacket(&buffer[0], 7));
}
TEST_F(UBXTest, it_discards_a_byte_if_it_is_not_a_start_byte) {
    buffer[0] = 0x00;
    ASSERT_EQ(-1, ubx.extractPacket(&buffer[0], 8));
}
TEST_F(UBXTest, it_discards_a_byte_if_the_start_sequence_is_invalid) {
    buffer[0] = UBX::SYNC_1;
    buffer[1] = 0x00;
    ASSERT_EQ(-1, ubx.extractPacket(&buffer[0], 8));
}
TEST_F(UBXTest, it_discards_a_byte_if_the_packet_has_an_invalid_checksum) {
    buffer[0] = UBX::SYNC_1;
    buffer[1] = UBX::SYNC_2;
    buffer[2] = buffer[3] = buffer[4] = buffer[5] = 0;
    buffer[6] = buffer[7] = 1;
    ASSERT_EQ(-1, ubx.extractPacket(&buffer[0], 8));
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
    ASSERT_EQ(8, ubx.extractPacket(&buffer[0], 10));
}

TEST_F(UBXTest, it_waits_for_more_bytes_if_packet_is_incomplete) {
    buffer = { UBX::SYNC_1, UBX::SYNC_2, 0x01, 0x02, 0x01, 0x00, 0x00, 0x00 };
    ASSERT_EQ(0, ubx.extractPacket(&buffer[0], 8));
}

TEST_F(UBXTest, it_accepts_a_valid_full_packet) {
    buffer = { UBX::SYNC_1, UBX::SYNC_2, 0x01, 0x02, 0x02, 0x00, 0xAA, 0xBB };
    insertChecksum(buffer);
    ASSERT_EQ(10, ubx.extractPacket(&buffer[0], 12));
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
    vector<uint8_t> packet = ubx.getCfgValSetPacket(UBX::I2C_ENABLED, true);
    buffer = { UBX::SYNC_1, UBX::SYNC_2, UBX::UBX_CFG, UBX::VALSET, 0x09, 0x00,
               0x00, UBX::LAYER_ALL, 0x00, 0x00, 0x03, 0x00, 0x51, 0x10, 0x01 };

    insertChecksum(buffer);
    ASSERT_EQ(buffer, packet);
}

TEST_F(UBXTest, it_returns_a_valset_ram_layer_packet) {
    vector<uint8_t> packet = ubx.getCfgValSetPacket(UBX::I2C_ENABLED, false, false);
    buffer = { UBX::SYNC_1, UBX::SYNC_2, UBX::UBX_CFG, UBX::VALSET, 0x09, 0x00,
               0x00, UBX::LAYER_RAM, 0x00, 0x00, 0x03, 0x00, 0x51, 0x10, 0x00 };

    insertChecksum(buffer);
    ASSERT_EQ(buffer, packet);
}
