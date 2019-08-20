#include <stdint.h>
#include <stddef.h>

#include <array>
#include <vector>

#include <gps_ublox/UBX.hpp>

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

namespace gps_ublox {
namespace UBX {

template<typename T>
std::vector<uint8_t> getConfigValueSetPacket(ConfigKeyId key_id,
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

template vector<uint8_t> getConfigValueSetPacket(ConfigKeyId, uint8_t, bool);
template vector<uint8_t> getConfigValueSetPacket(ConfigKeyId, uint16_t, bool);

template<>
vector<uint8_t> getConfigValueSetPacket<bool>(ConfigKeyId key_id,
                                              bool value,
                                              bool persist)
{
    return getConfigValueSetPacket<uint8_t>(key_id, value ? 1 : 0, persist);
}

}
}