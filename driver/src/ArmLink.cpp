// Copyright by BeeX [2026]

#include <driver/ArmLink.h>

#include <chrono>
#include <thread>

namespace driver {
namespace {

constexpr size_t kReadChunkBytes = 256;
constexpr auto   kReplyPollPeriod = std::chrono::milliseconds(1);

}  // namespace

ArmLink::ArmLink(std::shared_ptr<BytePort> port, double reply_timeout_s)
        : port_(std::move(port)), reply_timeout_s_(reply_timeout_s) {}

bool ArmLink::setPosition(uint8_t device, float position) {
    std::lock_guard<std::mutex> lock(mutex_);
    return send(device, packet_id::POSITION, encodeFloat(position));
}

bool ArmLink::setVelocity(uint8_t device, float velocity) {
    std::lock_guard<std::mutex> lock(mutex_);
    return send(device, packet_id::VELOCITY, encodeFloat(velocity));
}

bool ArmLink::setStandby(uint8_t device) {
    std::lock_guard<std::mutex> lock(mutex_);
    return send(device, packet_id::MODE, {mode::STANDBY});
}

bool ArmLink::readPosition(uint8_t device, float &position) {
    std::lock_guard<std::mutex> lock(mutex_);
    Packet reply;
    return request(device, packet_id::POSITION, reply) && decodeFloat(reply.data, position);
}

bool ArmLink::readMode(uint8_t device, uint8_t &mode) {
    std::lock_guard<std::mutex> lock(mutex_);
    Packet reply;
    if (!request(device, packet_id::MODE, reply) || reply.data.empty()) {
        return false;
    }
    mode = reply.data[0];
    return true;
}

bool ArmLink::send(uint8_t device, uint8_t id, const std::vector<uint8_t> &data) {
    const std::vector<uint8_t> frame = encodePacket(device, id, data);
    return port_->write(frame.data(), frame.size()) == static_cast<int>(frame.size());
}

bool ArmLink::request(uint8_t device, uint8_t field, Packet &reply) {
    if (!send(device, packet_id::REQUEST, {field})) {
        return false;
    }
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(reply_timeout_s_);
    uint8_t    chunk[kReadChunkBytes];
    while (std::chrono::steady_clock::now() < deadline) {
        const int received = port_->read(chunk, sizeof(chunk));
        if (received <= 0) {
            std::this_thread::sleep_for(kReplyPollPeriod);
            continue;
        }
        for (const Packet &packet : reader_.feed(chunk, static_cast<size_t>(received))) {
            if (packet.device == device && packet.id == field) {
                reply = packet;
                return true;
            }
        }
    }
    return false;
}

}  // namespace driver
