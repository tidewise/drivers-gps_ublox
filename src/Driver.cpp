#include <gps_base/rtcm3.hpp>
#include <gps_ublox/BoardInfo.hpp>
#include <gps_ublox/cfg.hpp>
#include <gps_ublox/Driver.hpp>
#include <gps_ublox/PVT.hpp>
#include <gps_ublox/RFInfo.hpp>
#include <base/Time.hpp>
#include <iostream>
#include <map>
#include <stdexcept>

using namespace std;
using namespace gps_ublox;
using namespace UBX;

Driver::Driver() : iodrivers_base::Driver(BUFFER_SIZE)
{
}

int Driver::extractPacket(const uint8_t *buffer, size_t buffer_size) const
{
    if (buffer_size == 0) {
        return 0;
    }
    else if (gps_base::rtcm3::isPreamble(buffer, buffer_size)) {
        return gps_base::rtcm3::extractPacket(buffer, buffer_size);
    }
    else {
        return UBX::extractPacket(buffer, buffer_size);
    }
}

BoardInfo Driver::readBoardInfo() {
    Frame frame = pollFrame(MSG_CLASS_MON, MSG_ID_VER);
    return UBX::parseVER(frame.payload);
}

PVT Driver::readPVT() {
    Frame frame = pollFrame(MSG_CLASS_NAV, MSG_ID_PVT);
    return UBX::parsePVT(frame.payload);
}

PVT Driver::waitForPVT() {
    Frame frame = waitForFrame(MSG_CLASS_NAV, MSG_ID_PVT);
    return UBX::parsePVT(frame.payload);
}

RelPosNED Driver::waitForRelPosNED() {
    Frame frame = waitForFrame(MSG_CLASS_NAV, MSG_ID_RELPOSNED);
    return UBX::parseRelPosNED(frame.payload);
}

RFInfo Driver::readRFInfo() {
    Frame frame = pollFrame(MSG_CLASS_MON, MSG_ID_RF);
    return UBX::parseRF(frame.payload);
}

SignalInfo Driver::readSignalInfo() {
    Frame frame = pollFrame(MSG_CLASS_NAV, MSG_ID_SIG);
    return UBX::parseSIG(frame.payload);
}

SatelliteInfo Driver::readSatelliteInfo() {
    Frame frame = pollFrame(MSG_CLASS_NAV, MSG_ID_SAT);
    return UBX::parseSAT(frame.payload);
}

Frame Driver::pollFrame(uint8_t class_id, uint8_t msg_id)
{
    Frame frame = {
        .msg_class = class_id,
        .msg_id    = msg_id
    };
    vector<uint8_t> packet = frame.toPacket();
    writePacket(&packet[0], packet.size());
    return waitForFrame(class_id, msg_id);
}

void Driver::poll(PollCallbacks& callbacks) {
    pollOneFrame(callbacks, getReadTimeout());

    // Pull as much RTCM data as possible in one shot
    try {
        while(true) {
            pollOneFrame(callbacks, base::Time());
        }
    }
    catch(iodrivers_base::TimeoutError&) {
    }
}

void Driver::pollOneFrame(PollCallbacks& callbacks, base::Time const& timeout) {
    int bytes = readPacket(mReadBuffer, BUFFER_SIZE, timeout);

    if (bytes == 0) {
        return;
    }

    if (gps_base::rtcm3::isPreamble(mReadBuffer, bytes)) {
        callbacks.rtcm(mReadBuffer, bytes);
        return;
    }

    Frame frame = Frame::fromPacket(mReadBuffer, bytes);
    if (frame.msg_class == MSG_CLASS_NAV && frame.msg_id == MSG_ID_PVT) {
        callbacks.pvt(UBX::parsePVT(frame.payload));
    }
    else if (frame.msg_class == MSG_CLASS_NAV && frame.msg_id == MSG_ID_RELPOSNED) {
        callbacks.relposned(UBX::parseRelPosNED(frame.payload));
    }
    else if (frame.msg_class == UBX::MSG_CLASS_NAV && frame.msg_id == UBX::MSG_ID_SAT) {
        callbacks.satelliteInfo(UBX::parseSAT(frame.payload));
    }
    else if (frame.msg_class == UBX::MSG_CLASS_NAV && frame.msg_id == UBX::MSG_ID_SIG) {
        callbacks.signalInfo(UBX::parseSIG(frame.payload));
    }
    else if (frame.msg_class == UBX::MSG_CLASS_MON && frame.msg_id == UBX::MSG_ID_RF) {
        callbacks.rfInfo(UBX::parseRF(frame.payload));
    }
    else if (frame.msg_class == UBX::MSG_CLASS_RXM && frame.msg_id == UBX::MSG_ID_RTCM) {
        callbacks.rtcmReceivedMessage(UBX::parseRTCMReceivedMessage(frame.payload));
    }
    else if (frame.msg_class == UBX::MSG_CLASS_MON && frame.msg_id == UBX::MSG_ID_COMMS) {
        callbacks.commsInfo(UBX::parseCommsInfo(frame.payload));
    }
}

Frame Driver::waitForPacket(const uint8_t *class_id, const uint8_t *msg_id,
                            const std::vector<uint8_t> *payload)
{
    base::Time deadline = base::Time::now() + getReadTimeout();
    while (base::Time::now() < deadline) {
        base::Time remaining = deadline - base::Time::now();
        int bytes = readPacket(mReadBuffer, BUFFER_SIZE, remaining);

        if (gps_base::rtcm3::isPreamble(mReadBuffer, bytes)) {
            continue;
        }

        Frame frame = Frame::fromPacket(mReadBuffer, bytes);
        if (class_id && *class_id != frame.msg_class) continue;
        if (msg_id && *msg_id != frame.msg_id) continue;
        if (payload && *payload != frame.payload) continue;
        return frame;
    }

    throw iodrivers_base::TimeoutError(
        iodrivers_base::TimeoutError::PACKET,
        "Did not receive the expected packet within the timeout");
}

Frame Driver::readFrame()
{
    return waitForPacket();
}

void Driver::writeRTCM(std::vector<uint8_t> const& data) {
    mRTCMReassembly.push(data);
    while(true) {
        vector<uint8_t> message = mRTCMReassembly.pull();
        if (message.empty()) {
            return;
        }

        writePacket(message.data(), message.size());
    }
}

Frame Driver::waitForFrame(uint8_t class_id, uint8_t msg_id)
{
    return waitForPacket(&class_id, &msg_id);
}

bool Driver::waitForAck(uint8_t class_id, uint8_t msg_id)
{
    uint8_t ack_class = MSG_CLASS_ACK;
    vector<uint8_t> payload = { class_id, msg_id };

    return waitForPacket(&ack_class, nullptr, &payload).msg_id == MSG_ID_ACK;
}

template<typename T>
void Driver::setConfigKeyValue(uint32_t key_id, T value, bool persist)
{
    vector<uint8_t> packet = getConfigValueSetPacket<T>(key_id, value, persist);
    writePacket(&packet[0], packet.size());

    if (!waitForAck(MSG_CLASS_CFG, MSG_ID_VALSET)) {
        throw ConfigValueSetError("Configuration rejected by the device");
    }
}

template void Driver::setConfigKeyValue<bool>(uint32_t, bool, bool);
template void Driver::setConfigKeyValue<uint8_t>(uint32_t, uint8_t, bool);
template void Driver::setConfigKeyValue<uint16_t>(uint32_t, uint16_t, bool);

void Driver::setPortEnabled(DevicePort port, bool state, bool persist) {
    setConfigKeyValue(cfg::getPortControlKey(port), state, persist);
}

void Driver::saveConfiguration() {
    Frame frame = {
        .msg_class = 0x06,
        .msg_id    = 0x09
    };
    frame.payload.resize(12, 0);
    frame.payload[0] = 0;
    frame.payload[4] = 1;
    frame.payload[8] = 0;
    vector<uint8_t> packet = frame.toPacket();
    writePacket(&packet[0], packet.size());
    waitForAck(0x06, 0x09);
}

void Driver::resetConfigurationToDefaults() {
    Frame frame = {
        .msg_class = 0x06,
        .msg_id    = 0x09
    };
    frame.payload.resize(12, 0);
    frame.payload[0] = 1;
    frame.payload[4] = 0;
    frame.payload[8] = 1;
    vector<uint8_t> packet = frame.toPacket();
    writePacket(&packet[0], packet.size());
    waitForAck(0x06, 0x09);
}

void Driver::setOutputRate(DevicePort port, MessageOutputType msg, uint8_t rate, bool persist) {
    setConfigKeyValue(cfg::getOutputRateKey(port, msg), rate, persist);
}

void Driver::setRTCMOutputRate(DevicePort port, uint16_t msg, uint8_t rate, bool persist) {
    setConfigKeyValue(cfg::getRTCMOutputKey(port, msg), rate, persist);
}

void Driver::setOdometer(bool state, bool persist)
{
    setConfigKeyValue(cfg::ODO_USE_ODO, state, persist);
}

void Driver::setLowSpeedCourseOverGroundFilter(bool state, bool persist)
{
    setConfigKeyValue(cfg::ODO_USE_COG, state, persist);
}

void Driver::setOutputLowPassFilteredVelocity(bool state, bool persist)
{
    setConfigKeyValue(cfg::ODO_OUTLPVEL, state, persist);
}

void Driver::setOutputLowPassFilteredHeading(bool state, bool persist)
{
    setConfigKeyValue(cfg::ODO_OUTLPCOG, state, persist);
}

void Driver::setOdometerProfile(OdometerProfile profile, bool persist)
{
    setConfigKeyValue(cfg::ODO_PROFILE, static_cast<uint8_t>(profile), persist);
}

void Driver::setUpperSpeedLimitForHeadingFilter(uint8_t speed, bool persist)
{
    setConfigKeyValue(cfg::ODO_COGMAXSPEED, speed, persist);
}

void Driver::setMaxPositionAccuracyForLowSpeedHeadingFilter(uint8_t accuracy, bool persist)
{
    setConfigKeyValue(cfg::ODO_COGMAXPOSACC, accuracy, persist);
}

void Driver::setVelocityLowPassFilterLevel(uint8_t gain, bool persist)
{
    setConfigKeyValue(cfg::ODO_VELLPGAIN, gain, persist);
}

void Driver::setHeadingLowPassFilterLevel(uint8_t gain, bool persist)
{
    setConfigKeyValue(cfg::ODO_COGLPGAIN, gain, persist);
}

void Driver::setPositionMeasurementPeriod(uint16_t period, bool persist)
{
    setConfigKeyValue(cfg::RATE_MEAS, period, persist);
}

void Driver::setMeasurementsPerSolutionRatio(uint16_t ratio, bool persist)
{
    if (ratio > 127) {
        throw std::invalid_argument("Maximum number of measurements per solution is 127");
    }
    setConfigKeyValue(cfg::RATE_NAV, ratio, persist);
}

void Driver::setPortProtocol(DevicePort port, DataDirection direction,
                             DeviceProtocol protocol, bool state, bool persist)
{
    uint32_t key_id = port + direction + protocol;
    setConfigKeyValue(key_id, state, persist);
}

void Driver::setMeasurementRefTime(MeasurementRefTime system, bool persist)
{
    setConfigKeyValue(cfg::RATE_TIMEREF, static_cast<uint8_t>(system), persist);
}

void Driver::setDynamicModel(DynamicModel model, bool persist)
{
    setConfigKeyValue(cfg::NAVSPG_DYNMODEL, static_cast<uint8_t>(model), persist);
}

void Driver::setSpeedThreshold(uint8_t speed, bool persist)
{
    setConfigKeyValue(cfg::MOT_GNSSSPEED_THRS, speed, persist);
}

void Driver::setStaticHoldDistanceThreshold(uint16_t distance, bool persist)
{
    setConfigKeyValue(cfg::MOT_GNSSDIST_THRS, distance, persist);
}
