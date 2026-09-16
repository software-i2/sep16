// Copyright by BeeX [2026]

#ifndef DRIVER_SIMULATEDARM_H
#define DRIVER_SIMULATEDARM_H

#include <driver/BytePort.h>
#include <driver/Protocol.h>

#include <deque>
#include <mutex>
#include <vector>

namespace driver {

// Answers the same packets as the arm and walks each joint towards its target.
class SimulatedArm : public BytePort {
public:
    // Everything in wire units: radians for rotary joints, millimetres for the jaw.
    struct Joint {
        uint8_t device_id;
        float   min;
        float   max;
        float   start;
        float   speed;
    };

    explicit SimulatedArm(const std::vector<Joint> &joints);

    bool isOpen() const override { return true; }
    int  write(const uint8_t *bytes, size_t count) override;
    int  read(uint8_t *bytes, size_t capacity) override;

private:
    struct State {
        Joint   joint;
        float   position;
        float   target;
        float   velocity;
        uint8_t mode;
    };

    void advance();
    void handle(const Packet &packet);
    void answer(const State &state, uint8_t field);

    std::vector<State>  joints_;
    double              last_advance_s_;
    PacketReader        reader_;
    std::deque<uint8_t> outgoing_;
    std::mutex          mutex_;
};

}  // namespace driver

#endif  // DRIVER_SIMULATEDARM_H
