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

#include <cstring>

#include "xcrsf/crc.h"
#include "xcrsf/handler.h"
#include "xcrsf/utils.h"

namespace crossfire {
    static constexpr auto STD_MEMORY_ORDER = std::memory_order::relaxed;
    static constexpr auto STD_READ_INTERRUPT = std::chrono::microseconds(100);
    static constexpr auto STD_TIMEOUT = std::chrono::milliseconds(250);

    Handler::Handler(const std::string &uart_path, const speed_t baud_rate): uart_serial_(uart_path, baud_rate) { }

    Handler::~Handler() { }

    bool Handler::open_port() {
        if (is_paired.load(STD_MEMORY_ORDER)) { return false; }
        this->uart_fd_ = this->uart_serial_.open_port();
        if (this->uart_fd_ == -1) { return false; }

        this->is_paired.store(true, STD_MEMORY_ORDER);
        this->thread_parser_ = std::thread{&Handler::receive_crsf, this};
        return true;
    }

    bool Handler::close_port() {
        this->is_paired.store(false, STD_MEMORY_ORDER);
        if (this->thread_parser_.joinable()) { this->thread_parser_.join(); }
        return this->uart_serial_.close_port() == 0;
    }

    CRSFLink Handler::get_link_state() {
        std::lock_guard lock(this->crsf_lock_);
        return this->link_state_;
    }

    std::array<uint16_t, 16> Handler::get_channel_state() {
        std::lock_guard lock(this->crsf_lock_);
        return this->channel_state_;
    }

    void Handler::parse_message(const uint8_t* crsf_cata) {
        std::lock_guard lock(this->crsf_lock_);
        if (crsf_cata[0] == CRSF_SYNC_BYTE && crsf_cata[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
            if (crsf_cata[1] < 24) { return; }
            for (int i = 0; i < 16; i++) { this->channel_state_[i] = get_channel_data(crsf_cata, i); }
        }

        if (crsf_cata[0] == CRSF_SYNC_BYTE && crsf_cata[2] == CRSF_FRAMETYPE_LINK_STATISTICS) {
            if (crsf_cata[1] < 12) { return; }
            this->link_state_.uplink_rssi_antenna_1 = crsf_cata[3]; this->link_state_.uplink_rssi_antenna_2 = crsf_cata[4];
            this->link_state_.uplink_link_quality = crsf_cata[5]; this->link_state_.uplink_snr = crsf_cata[6];
            this->link_state_.active_antenna = crsf_cata[7]; this->link_state_.rf_Mode = crsf_cata[8];
            this->link_state_.uplink_tx_power = crsf_cata[9]; this->link_state_.downlink_rssi_antenna = crsf_cata[10];
            this->link_state_.downlink_link_quality = crsf_cata[11]; this->link_state_.downlink_snr = crsf_cata[12];
        }
    }

    bool Handler::send_crsf(const uint8_t packet, const std::vector<uint8_t>& payload) const {
        if (!this->is_paired.load(STD_MEMORY_ORDER) || payload.size() > CRSF_MAX_FRAME_SIZE - 4) { return false; }
        const auto length = payload.size(); uint8_t buffer[CRSF_MAX_FRAME_SIZE];
        buffer[0] = CRSF_SYNC_BYTE; buffer[1] = length + 2; buffer[2] = packet;

        std::memcpy(&buffer[3], payload.data(), length); buffer[length + 3] = CRCValidator::get_crc8(&buffer[2], length + 1);
        return write(this->uart_fd_, buffer, length + 4);
    }

    void Handler::receive_crsf() {
        std::vector<uint8_t> buffer(0, 0x00); uint8_t byte = 0x00; this->timeout_ = std::chrono::high_resolution_clock::now();
        while (this->is_paired.load(STD_MEMORY_ORDER)) {
            const auto is_read = read(this->uart_fd_, &byte, 1);
            if (std::chrono::high_resolution_clock::now() > this->timeout_ + STD_TIMEOUT) { this->is_paired.store(false, STD_MEMORY_ORDER); }
            if (!is_read) { std::this_thread::sleep_for(STD_READ_INTERRUPT); continue; }
            if (byte != CRSF_SYNC_BYTE && buffer.empty()) { continue; }

            if (buffer.size() < 3 || buffer.size() < buffer[1] + 2) { buffer.emplace_back(byte);
                if (buffer.size() == buffer[1] + 2 && CRCValidator::get_crc8(&buffer[2], buffer[1] - 1) == buffer[buffer[1] + 1]) { this->parse_message(buffer.data()); }
                if (buffer[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) { this->timeout_ = std::chrono::high_resolution_clock::now(); }
                if (buffer.size() >= buffer[1] + 2) { buffer.assign(0, 0x00); }
            }
        }
    }
} // namespace crossfire