// Copyright by BeeX [2026]

#ifndef DRIVER_PROTOCOL_H
#define DRIVER_PROTOCOL_H

#include <cstddef>
#include <cstdint>
#include <vector>

namespace driver {

// Reach BPL packet ids and modes, fixed by the vendor protocol.
namespace packet_id {
constexpr uint8_t MODE     = 0x01;
constexpr uint8_t VELOCITY = 0x02;
constexpr uint8_t POSITION = 0x03;
constexpr uint8_t REQUEST  = 0x60;
}  // namespace packet_id

namespace mode {
constexpr uint8_t STANDBY  = 0x00;
constexpr uint8_t POSITION = 0x02;
constexpr uint8_t VELOCITY = 0x03;
}  // namespace mode

// The rotary joints speak radians on the wire, the jaw millimetres.
constexpr double WIRE_UNITS_PER_RADIAN = 1.0;
constexpr double WIRE_UNITS_PER_METRE  = 1000.0;

struct Packet {
    uint8_t              device = 0;
    uint8_t              id     = 0;
    std::vector<uint8_t> data;
};

// One frame: COBS-encoded data, id, device, length and CRC8, then a zero byte.
std::vector<uint8_t> encodePacket(uint8_t device, uint8_t id, const std::vector<uint8_t> &data);

std::vector<uint8_t> encodeFloat(float value);
bool                 decodeFloat(const std::vector<uint8_t> &data, float &value);

// Buffers incoming bytes and returns every complete, valid packet.
class PacketReader {
public:
    std::vector<Packet> feed(const uint8_t *bytes, size_t count);

private:
    std::vector<uint8_t> buffer_;
};

}  // namespace driver

#endif  // DRIVER_PROTOCOL_H
