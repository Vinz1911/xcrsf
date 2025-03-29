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
#include <array>
#include <thread>
#include <atomic>

#include "xcrsf/serial.h"
#include "xcrsf/definitions.h"

namespace crossfire {
    class XCrossfire {
        /**
         * @brief The file descriptor.
         */
        int uart_fd_ = -1;

        /**
         * @brief Instance of UARTSerial.
         */
        UARTSerial uart_serial_;

        /**
         * @brief The channel state.
         */
        std::array<uint16_t, 16> channel_state_{};

        /**
         * @brief The link state.
         */
        CRSFLink link_state_{};

        /**
         * @brief Timestamp for timeout.
         */
        std::chrono::system_clock::time_point timeout_{};

        /**
         * @brief Indicator of pairing status.
         */
        std::atomic<bool> is_paired_{};

        /**
         * @brief Mutex lock for channel data.
         */
        std::mutex crsf_lock_;

        /**
         * @brief The parser thread.
         */
        std::thread thread_parser_;

        /**
         * @brief Send CRSF Protocol message.
         *
         * @param packet The packet type.
         * @param payload The payload data.
         * @return True on success otherwise False.
         */
        [[nodiscard]] bool send_crsf(uint8_t packet, const std::vector<uint8_t>& payload) const;

        /**
         * @brief Parse CRSF Protocol message.
         */
        void receive_crsf();

        /**
         * @brief Update the channel information.
         */
        void update_state(const uint8_t* crsf_cata);

        /**
         * @brief Extract the channel information from the payload.
         */
        static uint16_t extract_channel(const uint8_t* data, int index);

    public:
        /**
         * @brief Create instance of XCrossfire.
         *
         * @param uart_path The path from the serial connection.
         * @param baud_rate The baud rate, default is 420_000 for ExpressLRS Receiver.
         */
        explicit XCrossfire(const std::string& uart_path, speed_t baud_rate = 420000);

        /**
         * @brief Destroy instance of XCrossfire.
         */
        ~XCrossfire();

        /**
         * @brief Get the link state information.
         *
         * @return The link statistic's.
         */
        CRSFLink get_link_state();

        /**
         * @brief Get the channel state information.
         *
         * @return The channel's.
         */
        std::array<uint16_t, 16> get_channel_state();

        /**
         * @brief Send vario telemetry data back to the transmitter.
         *
         * @param vertical_speed The speed in m/s.
         * @return True on success otherwise False.
         */
        [[nodiscard]] bool set_telemetry_vario(int16_t vertical_speed) const;

        /**
         * @brief Send battery telemetry data back to the transmitter.
         *
         * @param voltage The Volts from the power source.
         * @param current The Amps from the power source.
         * @param capacity The Milliamps from the power source.
         * @param percent The Percent from the power source.
         * @return True on success otherwise False.
         */
        [[nodiscard]] bool set_telemetry_battery(float voltage, float current, uint32_t capacity, uint8_t percent) const;

        /**
         * @brief Send attitude telemetry data back to the transmitter.
         *
         * @param pitch in radiant's.
         * @param roll in radiant's.
         * @param yaw in radiant's.
         * @return True on success otherwise False.
         */
        [[nodiscard]] bool set_telemetry_attitude(uint16_t pitch, uint16_t roll, uint16_t yaw) const;

        /**
         * @brief Send gps telemetry data back to the transmitter.
         *
         * @param latitude The latitude.
         * @param longitude The longitude.
         * @param groundspeed The ground speed in km/h.
         * @param heading The heading in degree's.
         * @param altitude The altitude in meter.
         * @param satellites The satellite count.
         * @return True on success otherwise False.
         */
        [[nodiscard]] bool set_telemetry_gps(float latitude, float longitude, uint16_t groundspeed, uint16_t heading, uint16_t altitude, uint8_t satellites) const;

        /**
         * @brief Status if receiver is paired.
         *
         * @return True if paired otherwise False.
         */
        [[nodiscard]] bool is_paired() const;

        /**
         * @brief Open serial connection.
         *
         * @return True on success otherwise False.
         */
        [[nodiscard]] bool open_port();

        /**
        * @brief Close serial connection.
        *
        * @return True on success otherwise False.
        */
        bool close_port();
    };
} // namespace crossfire