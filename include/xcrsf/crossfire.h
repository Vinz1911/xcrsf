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

#pragma once
#include <thread>

#include "xcrsf/serial.h"

namespace crossfire {
    class XCrossfire {
        UARTSerial uart_serial_;
        int uart_fd_ = -1;
        uint16_t channel_data_[16]{};
        std::mutex channel_lock_;
        std::thread thread_parser_;

        void join_all();
        void crsf_parser();
        void update_channel(const uint8_t* crsf_cata);
        static uint16_t extract_channel(const uint8_t* data, int index);

    public:
        explicit XCrossfire(const std::string& uart_path, speed_t baud_rate = 420000);
        ~XCrossfire();

        [[nodiscard]] bool open_port();
        bool close_port();
        uint16_t* get_channels();
    };
} // namespace crossfire