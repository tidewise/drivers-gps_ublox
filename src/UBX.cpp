#include <stdint.h>
#include <stddef.h>
#define _STANDARD_SOURCE
#include <time.h>

#include <array>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <gps_ublox/CommsInfo.hpp>
#include <gps_ublox/PVT.hpp>
#include <gps_ublox/RFInfo.hpp>
#include <gps_ublox/SignalInfo.hpp>
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
    uint8_t ck_a = 0, ck_b = 0;
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
        throw std::invalid_argument(
            "Invalid RF payload (size " + to_string(payload.size()) + ", "
            "expected at least 4)"
        );
    }

    uint8_t n_blocks = payload[1];
    unsigned int expected = 4 + (n_blocks * 24u);
    if (payload.size() != expected) {
        throw std::invalid_argument(
            "Invalid RF payload (size " + to_string(payload.size()) + ", "
            "expected " + to_string(expected) + ")"
        );
    }

    RFInfo data;
    data.version = payload[0];
    data.n_blocks = payload[1];
    data.blocks.resize(n_blocks);

    for (size_t i = 0; i < n_blocks; i++) {
        data.blocks[i].block_id = payload[4 + (24 * i)];
        data.blocks[i].jamming_state = static_cast<RFInfo::JammingState>(0x03 & payload[5 + (24 * i)]);
        data.blocks[i].antenna_status = static_cast<RFInfo::AntennaStatus>(payload[6 + (24 * i)]);
        data.blocks[i].antenna_power = static_cast<RFInfo::AntennaPower>(payload[7 + (24 * i)]);
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

SignalInfo UBX::parseSIG(const vector<uint8_t> &payload) {
    if (payload.size() < 8) {
        throw std::invalid_argument(
            "Invalid SIG payload (size " + to_string(payload.size())
            + ", expected at least 8)"
        );
    }

    uint8_t n_signals = payload[5];
    unsigned int expected = 8 + (n_signals * 16u);
    if (payload.size() != expected) {
        throw std::invalid_argument(
            "Invalid SIG payload (size " + to_string(payload.size())
            + ", expected " + to_string(expected) + ")"
        );
    }

    SignalInfo data;
    data.time_of_week = fromLittleEndian<uint32_t>(&payload[0]);
    data.version = payload[4];
    data.n_signals = payload[5];
    data.signals.resize(n_signals);

    for (size_t i = 0; i < n_signals; i++) {
        data.signals[i].gnss_id = payload[8 + (16 * i)];
        data.signals[i].satellite_id = payload[9 + (16 * i)];
        data.signals[i].signal_id = payload[10 + (16 * i)];
        data.signals[i].frequency_id = payload[11 + (16 * i)];
        data.signals[i].pseudorange_residual = fromLittleEndian<int16_t>(&payload[12 + (16 * i)]) * 0.1;
        data.signals[i].signal_strength = payload[14 + (16 * i)];
        data.signals[i].quality_indicator = payload[15 + (16 * i)];
        data.signals[i].correction_source = payload[16 + (16 * i)];
        data.signals[i].ionospheric_model = payload[17 + (16 * i)];
        data.signals[i].signal_flags = fromLittleEndian<uint16_t>(&payload[18 + (16 * i)]);
    }
    return data;
}

SatelliteInfo UBX::parseSAT(const vector<uint8_t> &payload) {
    if (payload.size() < 8) {
        throw std::invalid_argument(
            "Invalid SAT payload (size " + to_string(payload.size()) +
            " smaller than minimum of 8)"
        );
    }

    uint8_t n_sats = payload[5];
    size_t expected = 8 + (n_sats * 12u);
    if (payload.size() != expected) {
        throw std::invalid_argument(
            "Invalid SAT payload (size " + to_string(payload.size()) +
            ", expected " + to_string(expected) + ")"
        );
    }

    SatelliteInfo data;
    data.time_of_week = fromLittleEndian<uint32_t>(&payload[0]);
    data.version = payload[4];
    data.signals.resize(n_sats);

    for (size_t i = 0; i < n_sats; i++) {
        data.signals[i].gnss_id = payload[8 + (12 * i)];
        data.signals[i].satellite_id = payload[9 + (12 * i)];
        data.signals[i].signal_strength = payload[10 + (12 * i)];
        data.signals[i].elevation = base::Angle::fromDeg(fromLittleEndian<int8_t>(&payload[11 + (12 * i)]));
        data.signals[i].azimuth = base::Angle::fromDeg(fromLittleEndian<int16_t>(&payload[12 + (12 * i)]));
        data.signals[i].pseudorange_residual = fromLittleEndian<int16_t>(&payload[14 + (12 * i)]) * 0.1;
        data.signals[i].signal_flags = fromLittleEndian<uint32_t>(&payload[16 + (12 * i)]);
    }
    return data;
}

const base::Time CORRECTION_AGE[] = {
    base::Time::fromSeconds(9999),
    base::Time::fromSeconds(0),
    base::Time::fromSeconds(1),
    base::Time::fromSeconds(2),
    base::Time::fromSeconds(5),
    base::Time::fromSeconds(10),
    base::Time::fromSeconds(15),
    base::Time::fromSeconds(20),
    base::Time::fromSeconds(30),
    base::Time::fromSeconds(45),
    base::Time::fromSeconds(60),
    base::Time::fromSeconds(90),
    base::Time::fromSeconds(120),
    base::Time::fromSeconds(120),
    base::Time::fromSeconds(120),
    base::Time::fromSeconds(120)
};

PVT UBX::parsePVT(const vector<uint8_t> &payload) {
    if (payload.size() != 92) {
        std::stringstream ss;
        ss << "Invalid PVT payload (invalid size = "
           << payload.size() << ")";
        throw std::invalid_argument(ss.str());
    }

    PVT data;
    data.time_of_week = fromLittleEndian<uint32_t>(&payload[0]);

    tm utctm;
    memset(&utctm, 0, sizeof(utctm));
    utctm.tm_year = fromLittleEndian<uint16_t>(&payload[4]) - 1900;
    utctm.tm_mon = fromLittleEndian<uint8_t>(&payload[6]) - 1;
    utctm.tm_mday = fromLittleEndian<uint8_t>(&payload[7]);
    utctm.tm_hour = fromLittleEndian<uint8_t>(&payload[8]);
    utctm.tm_min = fromLittleEndian<uint8_t>(&payload[9]);
    utctm.tm_sec = fromLittleEndian<uint8_t>(&payload[10]);
    int32_t nanoseconds = fromLittleEndian<int32_t>(&payload[16]); // in nanoseconds
    uint64_t secs = timegm(&utctm);
    data.time = base::Time::fromSeconds(secs, nanoseconds / 1000);

    data.valid = fromLittleEndian<uint8_t>(&payload[11]);
    data.time_accuracy = fromLittleEndian<uint32_t>(&payload[12]);
    data.fix_type = static_cast<PVT::GNSSFixType>(fromLittleEndian<uint8_t>(&payload[20]));
    data.fix_flags = fromLittleEndian<uint8_t>(&payload[21]);
    data.additional_flags = fromLittleEndian<uint8_t>(&payload[22]);
    data.num_sats = fromLittleEndian<uint8_t>(&payload[23]);
    data.longitude = base::Angle::fromDeg((double)fromLittleEndian<int32_t>(&payload[24]) * 1e-7);
    data.latitude = base::Angle::fromDeg((double)fromLittleEndian<int32_t>(&payload[28]) * 1e-7);
    data.height = (double)fromLittleEndian<int32_t>(&payload[32]) / 1000.0;
    data.height_above_mean_sea_level = (double)fromLittleEndian<int32_t>(&payload[36]) / 1000.0;
    data.horizontal_accuracy = (double)fromLittleEndian<uint32_t>(&payload[40]) / 1000.0;
    data.vertical_accuracy = (double)fromLittleEndian<uint32_t>(&payload[44]) / 1000.0;
    data.vel_ned.x() = (double)fromLittleEndian<int32_t>(&payload[48]) / 1000.0;
    data.vel_ned.y() = (double)fromLittleEndian<int32_t>(&payload[52]) / 1000.0;
    data.vel_ned.z() = (double)fromLittleEndian<int32_t>(&payload[56]) / 1000.0;
    data.ground_speed = (double)fromLittleEndian<int32_t>(&payload[60]) / 1000.0;
    data.heading_of_motion = base::Angle::fromDeg((double)fromLittleEndian<int32_t>(&payload[64]) * 1e-5);
    data.speed_accuracy = (double)fromLittleEndian<uint32_t>(&payload[68]) / 1000.0;
    data.heading_accuracy = base::Angle::fromDeg((double)fromLittleEndian<uint32_t>(&payload[72]) * 1e-5);
    data.position_dop = (double)fromLittleEndian<uint16_t>(&payload[76]) * 0.01;
    data.more_flags = fromLittleEndian<uint8_t>(&payload[78]);
    data.age_of_differential_corrections = CORRECTION_AGE[(data.more_flags >> 1) & 0xF];
    data.heading_of_vehicle = base::Angle::fromDeg((double)fromLittleEndian<int32_t>(&payload[84]) * 1e-5);
    data.magnetic_declination = base::Angle::fromDeg((double)fromLittleEndian<int16_t>(&payload[88]) * 1e-2);
    data.magnetic_declination_accuracy = base::Angle::fromDeg((double)fromLittleEndian<uint16_t>(&payload[90]) * 1e-2);
    return data;
}

RelPosNED UBX::parseRelPosNED(const vector<uint8_t> &payload) {
    if (payload.size() != 64) {
        std::stringstream ss;
        ss << "Invalid RelPosNED payload (invalid size = "
           << payload.size() << ", expected 64)";
        throw std::invalid_argument(ss.str());
    }

    RelPosNED data;
    data.time_of_week = fromLittleEndian<uint32_t>(&payload[4]);
    data.reference_station_id = fromLittleEndian<uint32_t>(&payload[2]);
    data.flags = fromLittleEndian<uint32_t>(&payload[60]);

    if (data.flags & RelPosNED::FLAGS_RELATIVE_POSITION_VALID) {
        int32_t relPosN = fromLittleEndian<int32_t>(&payload[8]);
        int32_t relPosE = fromLittleEndian<int32_t>(&payload[12]);
        int32_t relPosD = fromLittleEndian<int32_t>(&payload[16]);

        int8_t relPosHPN = fromLittleEndian<int8_t>(&payload[32]);
        int8_t relPosHPE = fromLittleEndian<int8_t>(&payload[33]);
        int8_t relPosHPD = fromLittleEndian<int8_t>(&payload[34]);

        int32_t accN = fromLittleEndian<int32_t>(&payload[36]);
        int32_t accE = fromLittleEndian<int32_t>(&payload[40]);
        int32_t accD = fromLittleEndian<int32_t>(&payload[44]);

        data.relative_position_NED = base::Vector3d(
            static_cast<double>(relPosN) * 1e-2 + static_cast<double>(relPosHPN) * 1e-4,
            static_cast<double>(relPosE) * 1e-2 + static_cast<double>(relPosHPE) * 1e-4,
            static_cast<double>(relPosD) * 1e-2 + static_cast<double>(relPosHPD) * 1e-4
        );

        data.accuracy_NED = base::Vector3d(
            static_cast<double>(accN) * 1e-4,
            static_cast<double>(accE) * 1e-4,
            static_cast<double>(accD) * 1e-4
        );
    }

    if (data.flags & RelPosNED::FLAGS_HEADING_VALID) {
        int32_t relPosLength = fromLittleEndian<int32_t>(&payload[20]);
        int32_t relPosAngle = fromLittleEndian<int32_t>(&payload[24]);

        int8_t relPosHPLength = fromLittleEndian<int8_t>(&payload[35]);

        int32_t accLength = fromLittleEndian<int32_t>(&payload[48]);
        int32_t accHeading = fromLittleEndian<int32_t>(&payload[52]);

        data.relative_position_length =
            static_cast<double>(relPosLength) * 1e-2 +
            static_cast<double>(relPosHPLength) * 1e-4;
        data.relative_position_heading =
            base::Angle::fromDeg(static_cast<double>(relPosAngle) * 1e-5);
        data.accuracy_length = static_cast<double>(accLength) * 1e-4;
        data.accuracy_heading = base::Angle::fromDeg(static_cast<double>(accHeading) * 1e-5);
    }

    return data;
}

CommsInfo UBX::parseCommsInfo(const std::vector<uint8_t> &payload) {
    if (payload.size() < 8) {
        throw invalid_argument(
            "payload size smaller than 8, cannot be a UBX-MON-COMMS"
        );
    }

    uint8_t portCount = payload[1];
    size_t expectedSize = portCount * 40 + 8;
    if (payload.size() != expectedSize) {
        throw invalid_argument(
            "UBX-MON-COMMS with " + to_string(portCount) + " ports should be of size " +
            to_string(expectedSize) + ", but got payload of size " +
            to_string(payload.size())
        );
    }

    CommsInfo info;
    info.time = base::Time::now();
    info.tx_errors = payload[2];

    CommsInfo::PortInfo portInfo;
    uint8_t const* const protocols = payload.data() + 4;
    for (int i = 0; i < 4; ++i) {
        uint16_t protocolId = protocols[i];
        if (protocolId != 0xff) {
            CommsInfo::MessageCount count;
            count.protocol = static_cast<CommsInfo::Protocols>(protocolId);
            portInfo.message_count.push_back(count);
        }
    }

    uint8_t const* portBuffer = payload.data() + 8;
    for (int portI = 0; portI < portCount; ++portI, portBuffer += 40) {
        switch(fromLittleEndian<uint16_t>(portBuffer)) {
            case 0:
                portInfo.device_port = PORT_I2C;
                break;
            case 0x0100:
                portInfo.device_port = PORT_UART1;
                break;
            case 0x0201:
                portInfo.device_port = PORT_UART2;
                break;
            case 0x0300:
                portInfo.device_port = PORT_USB;
                break;
            case 0x0400:
                portInfo.device_port = PORT_SPI;
                break;
        }

        portInfo.tx_pending = fromLittleEndian<uint16_t>(portBuffer + 2);
        portInfo.tx_bytes = fromLittleEndian<uint32_t>(portBuffer + 4);
        portInfo.tx_usage = fromLittleEndian<uint8_t>(portBuffer + 8);
        portInfo.tx_peak_usage = fromLittleEndian<uint8_t>(portBuffer + 9);

        portInfo.rx_pending = fromLittleEndian<uint16_t>(portBuffer + 10);
        portInfo.rx_bytes = fromLittleEndian<uint32_t>(portBuffer + 12);
        portInfo.rx_usage = fromLittleEndian<uint8_t>(portBuffer + 16);
        portInfo.rx_peak_usage = fromLittleEndian<uint8_t>(portBuffer + 17);

        portInfo.overrun_errors = fromLittleEndian<uint16_t>(portBuffer + 18);

        auto message_count_it = portInfo.message_count.begin();
        for (int i = 0; i < 4; ++i) {
            if (protocols[i] != 0xff) {
                message_count_it->count =
                    fromLittleEndian<uint16_t>(portBuffer + 20 + i * 2);
                message_count_it++;

            }
        }

        info.per_port.push_back(portInfo);
    }

    return info;
}

BoardInfo UBX::parseVER(const vector<uint8_t> &payload) {
    BoardInfo info;
    info.software_version = string(reinterpret_cast<const char*>(&payload[0]));
    info.hardware_version = string(reinterpret_cast<const char*>(&payload[30]));

    for (size_t i = 40; i < payload.size(); i += 30) {
        info.extensions.push_back(string(reinterpret_cast<const char*>(&payload[i])));
    }
    return info;
}

RTCMReceivedMessage UBX::parseRTCMReceivedMessage(const std::vector<uint8_t> &payload) {
    if (payload.size() != 8) {
        throw std::invalid_argument(
            "Invalid payload size for UBX-RXM-RTCM, got " + to_string(payload.size()) +
            ", was expecting 8"
        );
    }

    RTCMReceivedMessage msg;
    msg.time = base::Time::now();
    msg.flags = payload[1];

    uint16_t msgType = fromLittleEndian<uint16_t>(&payload[6]);
    if (msgType == 4072) {
        msgType = msgType * 10 + fromLittleEndian<uint16_t>(&payload[2]);
    }
    msg.message_type = msgType;
    msg.reference_station_id = fromLittleEndian<uint16_t>(&payload[4]);
    return msg;
}

namespace gps_ublox {
namespace UBX {

template<typename T>
std::vector<uint8_t> getConfigValueSetPacket(
    uint32_t key_id, T value, bool persist
) {
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
template vector<uint8_t> getConfigValueSetPacket(uint32_t, uint32_t, bool);

template<>
vector<uint8_t> getConfigValueSetPacket<bool>(uint32_t key_id,
                                              bool value,
                                              bool persist)
{
    return getConfigValueSetPacket<uint8_t>(key_id, value ? 1 : 0, persist);
}

}
}
