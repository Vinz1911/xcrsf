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

#include <unistd.h>
#include <vector>
#include <chrono>

#include "xcrsf/crc.h"
#include "xcrsf/crossfire.h"

namespace crossfire {
    static constexpr auto STD_MEMORY_ORDER = std::memory_order::relaxed;
    static constexpr auto STD_READ_INTERRUPT = std::chrono::microseconds(100);

    static constexpr auto CRSF_INIT_BYTE = 0x00;
    static constexpr auto CRSF_SYNC_BYTE = 0xC8;
    static constexpr auto CRSF_PAYLOAD_SIZE = 0x18;
    static constexpr auto CRSF_TIMEOUT = std::chrono::milliseconds(250);

    XCrossfire::XCrossfire(const std::string& uart_path, const speed_t baud_rate): uart_serial_(uart_path, baud_rate) {
        /* Construct */
    }

    XCrossfire::~XCrossfire() {
        this->close_port();
    }

    bool XCrossfire::is_paired() const {
        return is_paired_.load(STD_MEMORY_ORDER);
    }

    bool XCrossfire::open_port() {
        this->uart_fd_ = this->uart_serial_.open_port();
        if (this->uart_fd_ == -1) { return false; }
        this->thread_parser_ = std::thread{&XCrossfire::crsf_parser, this};
        this->timeout_ = std::chrono::high_resolution_clock::now();
        this->is_paired_.store(true, STD_MEMORY_ORDER); return true;
    }

    bool XCrossfire::close_port() {
        if (this->uart_serial_.close_port() == 0) {
            if (this->thread_parser_.joinable()) {
                this->thread_parser_.join();
            } return true;
        } return false;
    }

    std::array<uint16_t, 16> XCrossfire::get_channels() {
        std::lock_guard lock(this->channel_lock_);
        return this->channel_data_;
    }

    void XCrossfire::update_channel(const uint8_t* crsf_cata) {
        std::lock_guard lock(this->channel_lock_);
        if (crsf_cata[0] == CRSF_SYNC_BYTE && crsf_cata[1] == CRSF_PAYLOAD_SIZE) {
            for (int i = 0; i < 16; i++) { this->channel_data_[i] = extract_channel(crsf_cata, i); }
        }
    }

    uint16_t XCrossfire::extract_channel(const uint8_t* data, const int index) {
        const int bit_position = index * 11, byte_index = 3 + bit_position / 8, bit_offset = bit_position % 8;
        uint16_t value = data[byte_index] >> bit_offset | data[byte_index + 1] << (8 - bit_offset);
        if (bit_offset > 5) { value |= data[byte_index + 2] << (16 - bit_offset); } return value & 0x07FF;
    }

    void XCrossfire::crsf_parser() {
        std::vector<uint8_t> buffer(3, CRSF_INIT_BYTE); uint8_t byte = CRSF_INIT_BYTE;
        while (this->is_paired_.load(STD_MEMORY_ORDER)) {
            if (read(this->uart_fd_, &byte, 1)) {
                if (byte == CRSF_SYNC_BYTE && buffer[0] == CRSF_INIT_BYTE) { buffer[0] = byte; continue; }
                if (buffer[0] == CRSF_SYNC_BYTE && buffer[1] == CRSF_INIT_BYTE) { buffer[1] = byte; continue; }
                if (buffer[0] == CRSF_SYNC_BYTE && buffer[2] == CRSF_INIT_BYTE) { buffer[2] = byte;
                    while (buffer.size() < buffer[1] + 2) { if (read(this->uart_fd_, &byte, 1)) { buffer.emplace_back(byte); } else { std::this_thread::sleep_for(STD_READ_INTERRUPT); } }
                    if (CRCValidator::get_crc8(&buffer[2], buffer[1] - 1) == buffer[buffer[1] + 1]) { this->update_channel(buffer.data()); }
                    if (buffer[1] == CRSF_PAYLOAD_SIZE) { this->timeout_ = std::chrono::high_resolution_clock::now(); } buffer.assign(3, CRSF_INIT_BYTE);
                }
            } else { std::this_thread::sleep_for(STD_READ_INTERRUPT); }
            if (std::chrono::high_resolution_clock::now() > this->timeout_ + CRSF_TIMEOUT) { this->is_paired_.store(false, STD_MEMORY_ORDER); }
        }
        this->is_paired_.store(false, STD_MEMORY_ORDER);
    }
} // namespace crossfire