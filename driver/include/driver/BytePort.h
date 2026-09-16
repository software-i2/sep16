// Copyright by BeeX [2026]

#ifndef DRIVER_BYTEPORT_H
#define DRIVER_BYTEPORT_H

#include <cstddef>
#include <cstdint>

namespace driver {

// A byte stream to the arm: the serial line, or the simulated arm.
class BytePort {
public:
    virtual ~BytePort() = default;

    virtual bool isOpen() const = 0;

    // Bytes written, or -1 on error.
    virtual int write(const uint8_t *bytes, size_t count) = 0;

    // Bytes read, 0 when nothing is waiting, -1 on error. Never blocks.
    virtual int read(uint8_t *bytes, size_t capacity) = 0;
};

}  // namespace driver

#endif  // DRIVER_BYTEPORT_H
