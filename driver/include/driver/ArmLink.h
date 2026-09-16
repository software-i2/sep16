// Copyright by BeeX [2026]

#ifndef DRIVER_ARMLINK_H
#define DRIVER_ARMLINK_H

#include <driver/BytePort.h>
#include <driver/Protocol.h>

#include <memory>
#include <mutex>

namespace driver {

// Commands and requests to single joints, in wire units. Safe to call from several threads.
class ArmLink {
public:
    ArmLink(std::shared_ptr<BytePort> port, double reply_timeout_s);

    bool setPosition(uint8_t device, float position);
    bool setVelocity(uint8_t device, float velocity);
    bool setStandby(uint8_t device);

    bool readPosition(uint8_t device, float &position);
    bool readMode(uint8_t device, uint8_t &mode);

private:
    bool send(uint8_t device, uint8_t id, const std::vector<uint8_t> &data);
    bool request(uint8_t device, uint8_t field, Packet &reply);

    std::shared_ptr<BytePort> port_;
    double                    reply_timeout_s_;
    PacketReader              reader_;
    std::mutex                mutex_;
};

}  // namespace driver

#endif  // DRIVER_ARMLINK_H
