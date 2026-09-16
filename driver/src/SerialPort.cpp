// Copyright by BeeX [2026]

#include <driver/SerialPort.h>

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace driver {
namespace {

bool baudConstant(int baud_rate, speed_t &out) {
    switch (baud_rate) {
    case 9600:
        out = B9600;
        return true;
    case 19200:
        out = B19200;
        return true;
    case 38400:
        out = B38400;
        return true;
    case 57600:
        out = B57600;
        return true;
    case 115200:
        out = B115200;
        return true;
    case 230400:
        out = B230400;
        return true;
    default:
        return false;
    }
}

}  // namespace

SerialPort::SerialPort(const std::string &device, int baud_rate) {
    speed_t speed = 0;
    if (!baudConstant(baud_rate, speed)) {
        open_error_ = "unsupported baud rate " + std::to_string(baud_rate);
        return;
    }

    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        open_error_ = "cannot open " + device + ": " + std::strerror(errno);
        return;
    }

    struct termios tty {};
    if (tcgetattr(fd_, &tty) != 0) {
        open_error_ = std::string("tcgetattr: ") + std::strerror(errno);
        ::close(fd_);
        fd_ = -1;
        return;
    }

    // Raw mode: any line discipline would corrupt binary packets.
    cfmakeraw(&tty);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        open_error_ = std::string("tcsetattr: ") + std::strerror(errno);
        ::close(fd_);
        fd_ = -1;
        return;
    }
    tcflush(fd_, TCIOFLUSH);
}

SerialPort::~SerialPort() {
    if (fd_ >= 0) {
        ::close(fd_);
    }
}

int SerialPort::write(const uint8_t *bytes, size_t count) {
    if (fd_ < 0) {
        return -1;
    }
    const ssize_t written = ::write(fd_, bytes, count);
    return written < 0 ? -1 : static_cast<int>(written);
}

int SerialPort::read(uint8_t *bytes, size_t capacity) {
    if (fd_ < 0) {
        return -1;
    }
    const ssize_t received = ::read(fd_, bytes, capacity);
    if (received < 0) {
        return (errno == EAGAIN || errno == EWOULDBLOCK) ? 0 : -1;
    }
    return static_cast<int>(received);
}

}  // namespace driver
