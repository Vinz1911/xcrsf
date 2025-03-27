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
#include <cstdint>

namespace crossfire {
    /**
     * @brief The CRSF Protocol packet type bytes.
     */
    enum CRSFPacket: uint8_t {
        CRSF_INIT = 0x00,
        CRSF_SYNC = 0xC8,
        CRSF_GPS = 0x02,
        CRSF_VARIO = 0x07,
        CRSF_BATTERY_SENSOR = 0x08,
        CRSF_BARO_ALTITUDE = 0x09,
        CRSF_LINK_STATISTICS = 0x14,
        CRSF_RC_CHANNELS_PACKED = 0x16,
        CRSF_MAX_PACKET = 0x3C,
        CRSF_ATTITUDE = 0x1E
    };

    /**
     * @brief The Battery type.
     */
    struct CRSFBattery {
        uint16_t voltage;
        uint16_t current;
        uint32_t capacity: 24;
        uint8_t percent;
    };
}
