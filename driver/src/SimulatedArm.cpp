// Copyright by BeeX [2026]

#include <driver/SimulatedArm.h>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace driver {
namespace {

// A host stall longer than this does not teleport the joints on the next packet.
constexpr double kLongestStep_s = 1.0;

double steadySeconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

SimulatedArm::SimulatedArm(const std::vector<Joint> &joints) : last_advance_s_(steadySeconds()) {
    for (const Joint &joint : joints) {
        joints_.push_back({joint, joint.start, joint.start, 0.0f, mode::STANDBY});
    }
}

int SimulatedArm::write(const uint8_t *bytes, size_t count) {
    std::lock_guard<std::mutex> lock(mutex_);
    advance();
    for (const Packet &packet : reader_.feed(bytes, count)) {
        handle(packet);
    }
    return static_cast<int>(count);
}

int SimulatedArm::read(uint8_t *bytes, size_t capacity) {
    std::lock_guard<std::mutex> lock(mutex_);
    advance();
    size_t count = 0;
    while (count < capacity && !outgoing_.empty()) {
        bytes[count++] = outgoing_.front();
        outgoing_.pop_front();
    }
    return static_cast<int>(count);
}

void SimulatedArm::advance() {
    const double now = steadySeconds();
    const double dt  = std::min(now - last_advance_s_, kLongestStep_s);
    last_advance_s_  = now;
    if (dt <= 0.0) {
        return;
    }

    for (State &state : joints_) {
        if (state.mode == mode::VELOCITY) {
            state.position = std::min(std::max(state.position + static_cast<float>(state.velocity * dt), state.joint.min),
                                      state.joint.max);
            state.target   = state.position;
            continue;
        }
        const float step  = static_cast<float>(state.joint.speed * dt);
        const float error = state.target - state.position;
        state.position += std::fabs(error) <= step ? error : std::copysign(step, error);
    }
}

void SimulatedArm::handle(const Packet &packet) {
    for (State &state : joints_) {
        if (state.joint.device_id != packet.device) {
            continue;
        }
        float value = 0.0f;
        switch (packet.id) {
        case packet_id::POSITION:
            if (decodeFloat(packet.data, value)) {
                state.target   = std::min(std::max(value, state.joint.min), state.joint.max);
                state.velocity = 0.0f;
                state.mode     = mode::POSITION;
            }
            break;
        case packet_id::VELOCITY:
            if (decodeFloat(packet.data, value)) {
                state.velocity = value;
                state.mode     = mode::VELOCITY;
            }
            break;
        case packet_id::MODE:
            if (!packet.data.empty()) {
                state.mode     = packet.data[0];
                state.velocity = 0.0f;
                state.target   = state.position;
            }
            break;
        case packet_id::REQUEST:
            for (uint8_t field : packet.data) {
                answer(state, field);
            }
            break;
        default:
            break;
        }
        return;
    }
}

void SimulatedArm::answer(const State &state, uint8_t field) {
    std::vector<uint8_t> frame;
    if (field == packet_id::POSITION) {
        frame = encodePacket(state.joint.device_id, packet_id::POSITION, encodeFloat(state.position));
    } else if (field == packet_id::MODE) {
        frame = encodePacket(state.joint.device_id, packet_id::MODE, {state.mode});
    }
    outgoing_.insert(outgoing_.end(), frame.begin(), frame.end());
}

}  // namespace driver
