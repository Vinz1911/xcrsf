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
#include <cstring>

#include "xcrsf/crc.h"
#include "xcrsf/crossfire.h"
#include "xcrsf/definitions.h"
#include "xcrsf/utils.h"

namespace crossfire {
    static constexpr auto STD_MEMORY_ORDER = std::memory_order::relaxed;
    static constexpr auto STD_READ_INTERRUPT = std::chrono::microseconds(100);
    static constexpr auto STD_TIMEOUT = std::chrono::milliseconds(250);

    XCrossfire::XCrossfire(const std::string& uart_path, const speed_t baud_rate): uart_serial_(uart_path, baud_rate) { }

    XCrossfire::~XCrossfire() {
        this->close_port();
    }

    bool XCrossfire::is_paired() const {
        return is_paired_.load(STD_MEMORY_ORDER);
    }

    bool XCrossfire::open_port() {
        if (is_paired_.load(STD_MEMORY_ORDER)) { return false; }
        this->uart_fd_ = this->uart_serial_.open_port();
        if (this->uart_fd_ == -1) { return false; }

        this->is_paired_.store(true, STD_MEMORY_ORDER);
        this->thread_parser_ = std::thread{&XCrossfire::receive_crsf, this}; return true;
    }

    bool XCrossfire::close_port() {
        this->is_paired_.store(false, STD_MEMORY_ORDER);
        if (this->thread_parser_.joinable()) { this->thread_parser_.join(); }
        return this->uart_serial_.close_port() == 0;
    }

    CRSFLink XCrossfire::get_link_state() {
        std::lock_guard lock(this->crsf_lock_);
        return this->link_state_;
    }

    std::array<uint16_t, 16> XCrossfire::get_channel_state() {
        std::lock_guard lock(this->crsf_lock_);
        return this->channel_state_;
    }

    bool XCrossfire::set_telemetry_vario(const int16_t vertical_speed) const {
        auto vario = CRSFVario{};
        vario.vertical_speed = swap_byte_order_int16(static_cast<int16_t>(vertical_speed * 100));

        const std::vector payload(reinterpret_cast<uint8_t*>(&vario), reinterpret_cast<uint8_t*>(&vario) + sizeof(CRSFVario));
        return this->send_crsf(CRSF_VARIO, payload);
    }

    bool XCrossfire::set_telemetry_battery(const float voltage, const float current, const uint32_t capacity, const uint8_t percent) const {
        auto battery = CRSFBattery{};
        battery.voltage = swap_byte_order_uint16(static_cast<uint16_t>(voltage * 10));
        battery.current = swap_byte_order_uint16(static_cast<uint16_t>(current * 10));
        battery.capacity = swap_byte_order_uint16(capacity) << 8;
        battery.percent = percent;

        const std::vector payload(reinterpret_cast<uint8_t*>(&battery), reinterpret_cast<uint8_t*>(&battery) + sizeof(CRSFBattery));
        return this->send_crsf(CRSF_BATTERY_SENSOR, payload);
    }

    bool XCrossfire::set_telemetry_attitude(const uint16_t pitch, const uint16_t roll, const uint16_t yaw) const {
        auto attitude = CRSFAttitude{};
        attitude.pitch = swap_byte_order_uint16(pitch);
        attitude.roll = swap_byte_order_uint16(roll);
        attitude.yaw = swap_byte_order_uint16(yaw);

        const std::vector payload(reinterpret_cast<uint8_t*>(&attitude), reinterpret_cast<uint8_t*>(&attitude) + sizeof(CRSFAttitude));
        return this->send_crsf(CRSF_ATTITUDE, payload);
    }

    bool XCrossfire::set_telemetry_gps(const float latitude, const float longitude, const uint16_t groundspeed, const uint16_t heading, const uint16_t altitude, const uint8_t satellites) const {
        auto global_position = CRSFGlobalPosition{};
        global_position.latitude = swap_byte_order_int32(static_cast<int32_t>(latitude * 10000000));
        global_position.longitude = swap_byte_order_int32(static_cast<int32_t>(longitude * 10000000));
        global_position.groundspeed = swap_byte_order_uint16(groundspeed * 10);
        global_position.heading = swap_byte_order_uint16(heading * 100);
        global_position.altitude = swap_byte_order_uint16(altitude + 1000);
        global_position.satellites = satellites;

        const std::vector payload(reinterpret_cast<uint8_t*>(&global_position), reinterpret_cast<uint8_t*>(&global_position) + sizeof(CRSFGlobalPosition));
        return this->send_crsf(CRSF_GPS, payload);
    }

    void XCrossfire::update_state(const uint8_t* crsf_cata) {
        std::lock_guard lock(this->crsf_lock_);
        if (crsf_cata[0] == CRSF_SYNC && crsf_cata[1] == CRSF_RC_CHANNEL + 2) {
            for (int i = 0; i < 16; i++) { this->channel_state_[i] = extract_channel(crsf_cata, i); }
        }

        if (crsf_cata[0] == CRSF_SYNC && crsf_cata[1] == CRSF_LINK_STATE + 2) {
            for (int i = 0; i < 10; i++) {
                if (i == 3) { this->link_state_.uplink_snr = static_cast<int8_t>(crsf_cata[3 + i]); }
                else if (i == 9) { this->link_state_.downlink_snr = static_cast<int8_t>(crsf_cata[3 + i]); }
                else { *(&this->link_state_.uplink_rssi_antenna_left + i) = static_cast<uint8_t>(crsf_cata[3 + i]); }
            }
        }
    }

    uint16_t XCrossfire::extract_channel(const uint8_t* data, const int index) {
        const int bit_position = index * 11, byte_index = 3 + bit_position / 8, bit_offset = bit_position % 8;
        uint16_t value = data[byte_index] >> bit_offset | data[byte_index + 1] << (8 - bit_offset);
        if (bit_offset > 5) { value |= data[byte_index + 2] << (16 - bit_offset); } return value & 0x07FF;
    }

    bool XCrossfire::send_crsf(const uint8_t packet, const std::vector<uint8_t>& payload) const {
        if (!this->is_paired_.load(STD_MEMORY_ORDER) || payload.size() > CRSF_MAX_PACKET) { return false; }
        const auto length = payload.size(); uint8_t buffer[CRSF_MAX_PACKET + 4];
        buffer[0] = CRSF_SYNC; buffer[1] = length + 2; buffer[2] = packet;

        std::memcpy(&buffer[3], payload.data(), length); buffer[length + 3] = CRCValidator::get_crc8(&buffer[2], length + 1);
        return write(this->uart_fd_, buffer, length + 4);
    }

    void XCrossfire::receive_crsf() {
        std::vector<uint8_t> buffer(3, CRSF_INIT); uint8_t byte = CRSF_INIT;
        this->timeout_ = std::chrono::high_resolution_clock::now();
        while (this->is_paired_.load(STD_MEMORY_ORDER)) {
            if (read(this->uart_fd_, &byte, 1)) {
                if (byte == CRSF_SYNC && buffer[0] == CRSF_INIT) { buffer[0] = byte; continue; }
                if (buffer[0] == CRSF_SYNC && buffer[1] == CRSF_INIT) { buffer[1] = byte; continue; }
                if (buffer[0] == CRSF_SYNC && buffer[2] == CRSF_INIT) { buffer[2] = byte;
                    while (buffer.size() < buffer[1] + 2) { if (read(this->uart_fd_, &byte, 1)) { buffer.emplace_back(byte); } else { std::this_thread::sleep_for(STD_READ_INTERRUPT); } }
                    if (CRCValidator::get_crc8(&buffer[2], buffer[1] - 1) == buffer[buffer[1] + 1]) { this->update_state(buffer.data()); }
                    if (buffer[1] == CRSF_RC_CHANNEL + 2) { this->timeout_ = std::chrono::high_resolution_clock::now(); } buffer.assign(3, CRSF_INIT);
                }
            } else { std::this_thread::sleep_for(STD_READ_INTERRUPT); }
            if (std::chrono::high_resolution_clock::now() > this->timeout_ + STD_TIMEOUT) { this->is_paired_.store(false, STD_MEMORY_ORDER); }
        }
    }
} // namespace crossfire