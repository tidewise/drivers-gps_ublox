#include <stdint.h>
#include <stddef.h>

#include <vector>

#include <gps_ublox/protocols/UBX.hpp>

using namespace std;
using namespace gps_ublox;
using namespace protocols;

const uint8_t UBX::SYNC_1 = 0xB5;
const uint8_t UBX::SYNC_2 = 0x62;
const uint8_t UBX::SYNC_1_IDX = 0;
const uint8_t UBX::SYNC_2_IDX = 1;
const uint8_t UBX::MSG_CLASS_IDX = 2;
const uint8_t UBX::MSG_ID_IDX = 3;
const uint8_t UBX::LENGTH_LSB_IDX = 4;
const uint8_t UBX::LENGTH_MSB_IDX = 5;
const uint8_t UBX::PAYLOAD_IDX = 6;
const size_t UBX::FRAMING_SIZE_OVERHEAD = 8;

UBX::Frame UBX::Frame::fromPacket(const uint8_t *buffer, size_t size)
{
    Frame frame;
    frame.msg_class = buffer[MSG_CLASS_IDX];
    frame.msg_id = buffer[MSG_ID_IDX];

    uint16_t payload_size = buffer[LENGTH_LSB_IDX] | (buffer[LENGTH_MSB_IDX] << 8);
    frame.payload = vector<uint8_t>(&buffer[PAYLOAD_IDX], &buffer[PAYLOAD_IDX] + payload_size);

    return frame;
}

vector<uint8_t> UBX::Frame::toPacket() const
{
    vector<uint8_t> packet;
    packet.push_back(SYNC_1);
    packet.push_back(SYNC_2);
    packet.push_back(this->msg_class);
    packet.push_back(this->msg_id);
    packet.push_back(this->payload.size() & 0x00FF);
    packet.push_back((this->payload.size() & 0xFF00) >> 8);
    packet.insert(packet.end(), this->payload.begin(), this->payload.end());

    vector<uint8_t> ck = UBX::checksum(&packet[MSG_CLASS_IDX], &packet[0] + packet.size());

    packet.push_back(ck[0]);
    packet.push_back(ck[1]);

    return packet;
}

int UBX::extractPacket(const uint8_t *buffer, size_t buffer_size) const
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
    vector<uint8_t> ck = checksum(&buffer[MSG_CLASS_IDX],
                                  &buffer[PAYLOAD_IDX + payload_size]);

    if (ck[0] != ck_a || ck[1] != ck_b)
        return -1;

    return packet_size;
}

vector<uint8_t> UBX::checksum(const uint8_t *buffer, const uint8_t *end) {
    uint8_t ck_a, ck_b = 0;
    while (buffer != end) {
        ck_a += *buffer++;
        ck_b += ck_a;
    }
    return vector<uint8_t>{ck_a, ck_b};
}