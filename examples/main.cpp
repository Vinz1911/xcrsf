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
#include <vector>

#include "xcrsf/crossfire.h"

int main() {
    auto crossfire = crossfire::XCrossfire("/dev/ttyAMA10");
    if (crossfire.open_port()) { std::printf("Port opened...\n"); }

    while (crossfire.is_paired()) {
        const auto channels = crossfire.get_channels();
        for (int i = 0; i < 4; i++) {
            if (channels.front() != 0) { std::printf("Channel %d: %u\n", i, channels[i]); }
        }
        const auto is_send = crossfire.set_battery_telemetry(11.4, 15.6, 6500, 75);
        if (!is_send) { std::printf("Sending telemetry data failed!\n"); }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return 0;
}
