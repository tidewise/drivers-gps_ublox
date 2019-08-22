#include <stdint.h>
#include <stddef.h>

#include <array>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <gps_ublox/UBX.hpp>
#include <gps_ublox/GPSData.hpp>
#include <gps_ublox/RFInfo.hpp>

using namespace std;
using namespace gps_ublox;
using namespace UBX;

Frame Frame::fromPacket(const uint8_t *buffer, size_t size)
{
    uint16_t payload_size = buffer[LENGTH_LSB_IDX] |
                            (buffer[LENGTH_MSB_IDX] << 8);

    Frame frame = {
        .msg_class = buffer[MSG_CLASS_IDX],
        .msg_id = buffer[MSG_ID_IDX],
        .payload = vector<uint8_t>(&buffer[PAYLOAD_IDX],
                                   &buffer[PAYLOAD_IDX] + payload_size)
    };
    return frame;
}

vector<uint8_t> Frame::toPacket() const
{
    vector<uint8_t> packet = {
        SYNC_1,
        SYNC_2,
        msg_class,
        msg_id
    };

    packet.push_back(payload.size() & 0x00FF);
    packet.push_back((payload.size() & 0xFF00) >> 8);
    packet.insert(packet.end(), payload.begin(), payload.end());

    array<uint8_t, 2> ck = checksum(&packet[MSG_CLASS_IDX], &packet[0] + packet.size());

    packet.push_back(ck[0]);
    packet.push_back(ck[1]);

    return packet;
}

int UBX::extractPacket(const uint8_t *buffer, size_t buffer_size)
{
    if (buffer_size < FRAMING_SIZE_OVERHEAD)
        return 0;

    if ((buffer[SYNC_1_IDX] != SYNC_1) || (buffer[SYNC_2_IDX] != SYNC_2))
        return -1;

    uint16_t payload_size = buffer[LENGTH_LSB_IDX] | (buffer[LENGTH_MSB_IDX] << 8);
    size_t packet_size = payload_size + FRAMING_SIZE_OVERHEAD;

    if (buffer_size < packet_size)
        return 0;

    uint8_t ck_a = buffer[PAYLOAD_IDX + payload_size];
    uint8_t ck_b = buffer[PAYLOAD_IDX + payload_size + 1];

    // checksum is calculated over the message, starting from and including the
    // MSG_CLASS field up until, but excluding, the checksum field
    array<uint8_t, 2> ck = checksum(&buffer[MSG_CLASS_IDX],
                                    &buffer[PAYLOAD_IDX + payload_size]);

    if (ck[0] != ck_a || ck[1] != ck_b)
        return -1;

    return packet_size;
}

array<uint8_t, 2> UBX::checksum(const uint8_t *buffer, const uint8_t *end) {
    uint8_t ck_a, ck_b = 0;
    while (buffer != end) {
        ck_a += *buffer++;
        ck_b += ck_a;
    }
    return array<uint8_t, 2>{ck_a, ck_b};
}

template<typename T>
static T fromLittleEndian(const uint8_t *buffer)
{
    T result = 0;
    uint8_t shifter = 0;
    for (size_t i = 0; i < sizeof(T); i++) {
        result |= static_cast<T>(buffer[i]) << shifter;
        shifter += 8;
    }
    return reinterpret_cast<T const&>(result);
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

RFInfo UBX::parseRF(const vector<uint8_t> &payload) {
    if (payload.size() < 4) {
        std::stringstream ss("Invalid RF payload (invalid size = ");
        ss << payload.size() << ")";
        throw std::invalid_argument(ss.str());
    }

    uint8_t n_blocks = payload[1];
    if (payload.size() != 4 + (n_blocks * (unsigned int)24)) {
        std::stringstream ss("Invalid RF payload (invalid size = ");
        ss << payload.size() << ")";
        throw std::invalid_argument(ss.str());
    }

    RFInfo data;
    data.version = payload[0];
    data.n_blocks = payload[1];
    data.blocks.resize(n_blocks);

    for (size_t i = 0; i < n_blocks; i++) {
        data.blocks[i].block_id = payload[4 + (24 * i)];
        data.blocks[i].flags = payload[5 + (24 * i)];
        data.blocks[i].antenna_status = payload[6 + (24 * i)];
        data.blocks[i].antenna_power = payload[7 + (24 * i)];
        data.blocks[i].post_status = fromLittleEndian<uint32_t>(&payload[8 + (24 * i)]);
        data.blocks[i].noise_per_measurement = fromLittleEndian<uint16_t>(&payload[16 + (24 * i)]);
        data.blocks[i].agc_count = fromLittleEndian<uint16_t>(&payload[18 + (24 * i)]);
        data.blocks[i].jamming_indicator = payload[20 + (24 * i)];
        data.blocks[i].ofs_i = static_cast<int8_t>(payload[21 + (24 * i)]);
        data.blocks[i].mag_i = payload[22 + (24 * i)];
        data.blocks[i].ofs_q = static_cast<int8_t>(payload[23 + (24 * i)]);
        data.blocks[i].mag_q = payload[24 + (24 * i)];
    }
    return data;
}

GPSData UBX::parsePVT(const vector<uint8_t> &payload) {
    if (payload.size() != 92) {
        std::stringstream ss;
        ss << "Invalid PVT payload (invalid size = "
           << payload.size() << ")";
        throw std::invalid_argument(ss.str());
    }

    GPSData data;
    data.time_of_week = fromLittleEndian<uint32_t>(&payload[0]);
    data.year = fromLittleEndian<uint16_t>(&payload[4]);
    data.month = fromLittleEndian<uint8_t>(&payload[6]);
    data.day = fromLittleEndian<uint8_t>(&payload[7]);
    data.hour = fromLittleEndian<uint8_t>(&payload[8]);
    data.min = fromLittleEndian<uint8_t>(&payload[9]);
    data.sec = fromLittleEndian<uint8_t>(&payload[10]);
    data.valid = fromLittleEndian<uint8_t>(&payload[11]);
    data.time_accuracy = fromLittleEndian<uint32_t>(&payload[12]);
    data.fraction = fromLittleEndian<int32_t>(&payload[16]);
    data.fix_type = fromLittleEndian<uint8_t>(&payload[20]);
    data.fix_flags = fromLittleEndian<uint8_t>(&payload[21]);
    data.additional_flags = fromLittleEndian<uint8_t>(&payload[22]);
    data.num_sats = fromLittleEndian<uint8_t>(&payload[23]);
    data.longitude = fromLittleEndian<int32_t>(&payload[24]);
    data.latitude = fromLittleEndian<int32_t>(&payload[28]);
    data.height = fromLittleEndian<int32_t>(&payload[32]);
    data.height_above_mean_sea_level = fromLittleEndian<int32_t>(&payload[36]);
    data.horizontal_accuracy = fromLittleEndian<uint32_t>(&payload[40]);
    data.vertical_accuracy = fromLittleEndian<uint32_t>(&payload[44]);
    data.vel_north = fromLittleEndian<int32_t>(&payload[48]);
    data.vel_east = fromLittleEndian<int32_t>(&payload[52]);
    data.vel_down = fromLittleEndian<int32_t>(&payload[56]);
    data.ground_speed = fromLittleEndian<int32_t>(&payload[60]);
    data.heading_of_motion = fromLittleEndian<int32_t>(&payload[64]);
    data.speed_accuracy = fromLittleEndian<uint32_t>(&payload[68]);
    data.heading_accuracy = fromLittleEndian<uint32_t>(&payload[72]);
    data.position_dop = fromLittleEndian<uint16_t>(&payload[76]);
    data.more_flags = fromLittleEndian<uint8_t>(&payload[78]);
    data.heading_of_vehicle = fromLittleEndian<int32_t>(&payload[84]);
    data.magnetic_declination = fromLittleEndian<int16_t>(&payload[88]);
    data.magnetic_declination_accuracy = fromLittleEndian<uint16_t>(&payload[90]);
    return data;
}

namespace gps_ublox {
namespace UBX {

template<typename T>
std::vector<uint8_t> getConfigValueSetPacket(uint32_t key_id,
                                             T value,
                                             bool persist)
{
    Frame frame;
    frame.msg_class = MSG_CLASS_CFG;
    frame.msg_id = MSG_ID_VALSET;

    frame.payload.push_back(0);
    frame.payload.push_back(persist ? LAYER_ALL : LAYER_RAM);
    frame.payload.push_back(0);
    frame.payload.push_back(0);
    toLittleEndian<uint32_t>(frame.payload, key_id);
    toLittleEndian<T>(frame.payload, value);

    return frame.toPacket();
}

template vector<uint8_t> getConfigValueSetPacket(uint32_t, uint8_t, bool);
template vector<uint8_t> getConfigValueSetPacket(uint32_t, uint16_t, bool);

template<>
vector<uint8_t> getConfigValueSetPacket<bool>(uint32_t key_id,
                                              bool value,
                                              bool persist)
{
    return getConfigValueSetPacket<uint8_t>(key_id, value ? 1 : 0, persist);
}

}
}