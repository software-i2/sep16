// Copyright by BeeX [2026]

#ifndef DRIVER_SERIALPORT_H
#define DRIVER_SERIALPORT_H

#include <driver/BytePort.h>

#include <string>

namespace driver {

// Raw binary serial line, opened on construction and closed on destruction.
class SerialPort : public BytePort {
public:
    SerialPort(const std::string &device, int baud_rate);
    ~SerialPort() override;

    bool isOpen() const override { return fd_ >= 0; }
    int  write(const uint8_t *bytes, size_t count) override;
    int  read(uint8_t *bytes, size_t capacity) override;

    const std::string &openError() const { return open_error_; }

private:
    int         fd_ = -1;
    std::string open_error_;
};

}  // namespace driver

#endif  // DRIVER_SERIALPORT_H
