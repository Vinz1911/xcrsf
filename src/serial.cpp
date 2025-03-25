/*
 * MIT License
 *
 * Copyright (c) 2025 Vinzenz Weist
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <utility>

#include "xcrsf/serial.h"

namespace crossfire {
    UARTSerial::UARTSerial(std::string uart_path, const speed_t baud_rate): uart_path_(std::move(uart_path)), baud_rate_(baud_rate) {
        /* Construct */
    }

    UARTSerial::~UARTSerial() {
        close(this->uart_fd_);
    }

    int UARTSerial::open_port() {
        this->uart_fd_ = open(this->uart_path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (this->uart_fd_ == -1) { return this->uart_fd_; }

        const auto status = this->reconfigure_port(this->baud_rate_);
        if (status == -1) { close(this->uart_fd_); return status; }
        return this->uart_fd_;
    }

    int UARTSerial::close_port() const {
        return close(this->uart_fd_);
    }

    int UARTSerial::reconfigure_port(const speed_t baud_rate) const {
        termios2 options{}; int status{};
        status = ioctl(this->uart_fd_, TCGETS2, &options);
        if (status == -1) { return status; }

        options.c_cflag &= ~CBAUD;
        options.c_cflag |= BOTHER;
        options.c_cflag |= CLOCAL;
        options.c_cflag |= HUPCL;
        options.c_cflag |= CREAD;
        options.c_cflag |= CS8;

        options.c_ispeed = baud_rate;
        options.c_ospeed = baud_rate;

        options.c_iflag = 0;
        options.c_oflag = 0;
        options.c_lflag = 0;

        options.c_cc[VTIME] = 0;
        options.c_cc[VMIN] = 0;

        status = ioctl(this->uart_fd_, TCSETS2, &options); return status;
    }
} // namespace crossfire