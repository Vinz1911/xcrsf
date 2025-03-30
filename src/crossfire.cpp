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
    XCrossfire::XCrossfire(const std::string& uart_path, const speed_t baud_rate): handler_(uart_path, baud_rate) { }

    XCrossfire::~XCrossfire() {
        this->close_port();
    }

    bool XCrossfire::open_port() {
        return this->handler_.open_port();
    }

    bool XCrossfire::close_port() {
        return this->handler_.close_port();
    }

    bool XCrossfire::is_paired() const {
        return this->handler_.is_paired.load(std::memory_order::relaxed);
    }

    CRSFLink XCrossfire::get_link_state() {
        return this->handler_.get_link_state();
    }

    std::array<uint16_t, 16> XCrossfire::get_channel_state() {
        return this->handler_.get_channel_state();
    }

    bool XCrossfire::set_telemetry_vario(const double vertical_speed) const {
        auto vario = CRSFVario{};
        vario.vertical_speed = get_big_endian_int16(static_cast<int16_t>(vertical_speed * 100));

        const std::vector payload(reinterpret_cast<uint8_t*>(&vario), reinterpret_cast<uint8_t*>(&vario) + sizeof(CRSFVario));
        return this->handler_.send_crsf(CRSF_FRAMETYPE_VARIO, payload);
    }

    bool XCrossfire::set_telemetry_battery(const double voltage, const double current, const uint32_t capacity, const uint8_t percent) const {
        auto battery = CRSFBattery{};
        battery.voltage = get_big_endian_uint16(static_cast<uint16_t>(voltage * 10));
        battery.current = get_big_endian_uint16(static_cast<uint16_t>(current * 10));
        battery.capacity = get_big_endian_uint16(capacity) << 8;
        battery.percent = percent;

        const std::vector payload(reinterpret_cast<uint8_t*>(&battery), reinterpret_cast<uint8_t*>(&battery) + sizeof(CRSFBattery));
        return this->handler_.send_crsf(CRSF_FRAMETYPE_BATTERY_SENSOR, payload);
    }

    bool XCrossfire::set_telemetry_attitude(const double pitch, const double roll, const double yaw) const {
        auto attitude = CRSFAttitude{};
        attitude.pitch = get_big_endian_uint16(static_cast<uint16_t>(get_attitude(pitch)));
        attitude.roll = get_big_endian_uint16(static_cast<uint16_t>(get_attitude(roll)));
        attitude.yaw = get_big_endian_uint16(static_cast<uint16_t>(get_attitude(yaw)));

        const std::vector payload(reinterpret_cast<uint8_t*>(&attitude), reinterpret_cast<uint8_t*>(&attitude) + sizeof(CRSFAttitude));
        return this->handler_.send_crsf(CRSF_FRAMETYPE_ATTITUDE, payload);
    }

    bool XCrossfire::set_telemetry_gps(const double latitude, const double longitude, const double groundspeed, const double heading, const double altitude, const uint8_t satellites) const {
        auto gps = CRSFGlobalPosition{};
        gps.latitude = get_big_endian_int32(static_cast<int32_t>(latitude * 1e7));
        gps.longitude = get_big_endian_int32(static_cast<int32_t>(longitude * 1e7));
        gps.groundspeed = get_big_endian_uint16(static_cast<uint16_t>(groundspeed * 10));
        gps.heading = get_big_endian_uint16(static_cast<uint16_t>(heading * 100));
        gps.altitude = get_big_endian_uint16(static_cast<uint16_t>(altitude + 1000));
        gps.satellites = satellites;

        const std::vector payload(reinterpret_cast<uint8_t*>(&gps), reinterpret_cast<uint8_t*>(&gps) + sizeof(CRSFGlobalPosition));
        return this->handler_.send_crsf(CRSF_FRAMETYPE_GPS, payload);
    }
} // namespace crossfire