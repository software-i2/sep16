// Copyright by BeeX [2026]

#include <driver/Protocol.h>

#include <algorithm>
#include <array>
#include <cstring>

namespace driver {
namespace {

// A stuck line never delivers a zero byte; older bytes than this can no longer start a frame.
constexpr size_t kMaxBufferedBytes = 4096;

constexpr uint8_t kCrcPolynomial = 0xB2;
constexpr uint8_t kCrcFinalXor   = 0xFF;
constexpr size_t  kTrailerBytes  = 4;

const std::array<uint8_t, 256> kCrcTable = []() {
    std::array<uint8_t, 256> table{};
    for (int i = 0; i < 256; ++i) {
        uint8_t value = static_cast<uint8_t>(i);
        for (int bit = 0; bit < 8; ++bit) {
            value = (value & 1u) ? static_cast<uint8_t>((value >> 1) ^ kCrcPolynomial)
                                 : static_cast<uint8_t>(value >> 1);
        }
        table[i] = value;
    }
    return table;
}();

uint8_t crc8(const uint8_t *bytes, size_t count) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < count; ++i) {
        crc = kCrcTable[crc ^ bytes[i]];
    }
    return crc ^ kCrcFinalXor;
}

std::vector<uint8_t> cobsEncode(const std::vector<uint8_t> &input) {
    std::vector<uint8_t> output;
    output.reserve(input.size() + input.size() / 254 + 2);

    size_t  code_index = 0;
    uint8_t code       = 1;
    output.push_back(0x00);

    for (uint8_t byte : input) {
        if (byte != 0x00) {
            output.push_back(byte);
            if (++code != 0xFF) {
                continue;
            }
        }
        output[code_index] = code;
        code_index         = output.size();
        output.push_back(0x00);
        code = 1;
    }
    output[code_index] = code;
    return output;
}

bool cobsDecode(const std::vector<uint8_t> &input, std::vector<uint8_t> &output) {
    output.clear();
    size_t i = 0;
    while (i < input.size()) {
        const uint8_t code = input[i++];
        if (code == 0x00) {
            return false;
        }
        for (uint8_t k = 1; k < code; ++k, ++i) {
            if (i >= input.size()) {
                return false;
            }
            output.push_back(input[i]);
        }
        if (code != 0xFF && i < input.size()) {
            output.push_back(0x00);
        }
    }
    return true;
}

// Body layout: data, id, device, length, crc.
bool parsePacket(const std::vector<uint8_t> &frame, Packet &out) {
    std::vector<uint8_t> body;
    if (!cobsDecode(frame, body) || body.size() < kTrailerBytes) {
        return false;
    }
    if (body[body.size() - 2] != static_cast<uint8_t>(body.size())
        || crc8(body.data(), body.size() - 1) != body.back()) {
        return false;
    }
    out.id     = body[body.size() - 4];
    out.device = body[body.size() - 3];
    out.data.assign(body.begin(), body.end() - kTrailerBytes);
    return true;
}

}  // namespace

std::vector<uint8_t> encodePacket(uint8_t device, uint8_t id, const std::vector<uint8_t> &data) {
    std::vector<uint8_t> body = data;
    body.push_back(id);
    body.push_back(device);
    body.push_back(static_cast<uint8_t>(data.size() + kTrailerBytes));
    body.push_back(crc8(body.data(), body.size()));

    std::vector<uint8_t> frame = cobsEncode(body);
    frame.push_back(0x00);
    return frame;
}

std::vector<uint8_t> encodeFloat(float value) {
    std::vector<uint8_t> bytes(sizeof(float));
    std::memcpy(bytes.data(), &value, sizeof(float));
    return bytes;
}

bool decodeFloat(const std::vector<uint8_t> &data, float &value) {
    if (data.size() < sizeof(float)) {
        return false;
    }
    std::memcpy(&value, data.data(), sizeof(float));
    return true;
}

std::vector<Packet> PacketReader::feed(const uint8_t *bytes, size_t count) {
    buffer_.insert(buffer_.end(), bytes, bytes + count);
    if (buffer_.size() > kMaxBufferedBytes) {
        buffer_.erase(buffer_.begin(), buffer_.end() - kMaxBufferedBytes);
    }

    std::vector<Packet> packets;
    for (;;) {
        const auto end = std::find(buffer_.begin(), buffer_.end(), uint8_t{0x00});
        if (end == buffer_.end()) {
            break;
        }
        const std::vector<uint8_t> frame(buffer_.begin(), end);
        buffer_.erase(buffer_.begin(), end + 1);

        Packet packet;
        if (!frame.empty() && parsePacket(frame, packet)) {
            packets.push_back(packet);
        }
    }
    return packets;
}

}  // namespace driver
