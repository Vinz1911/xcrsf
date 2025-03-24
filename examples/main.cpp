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

#include <array>
//#include <termios.h>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <cstring>
#include <thread>
#include <vector>
#include <asm/termbits.h>
#include <linux/serial.h>

#define BAUD_RATE 420000
// #define BOTHER 0x00001000

#define BAUDRATE_OFFSET 4

#define CRSF_MAX_PACKET_LEN 64
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8
#define CRSF_PACKET_SIZE 26
#define CRSF_MAX_CHANNEL 16

int16_t loc_channels[16] = {1023};
uint16_t m_channels[CRSF_MAX_CHANNEL];


// Setup UART with termios for standard settings (data bits, stop bits, parity, etc.)
int setup_uart(const char *device) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == -1) {
        std::cerr << "Error opening UART: " << strerror(errno) << std::endl;
        return -1;
    }
    termios2 options;

    auto get_ = ioctl(fd, TCGETS2, &options);
    std::printf("TCGETS2 returned %d\n", get_);

    options.c_cflag = 7344;
    options.c_iflag = 0;
    options.c_oflag = 0;
    options.c_lflag = 0;
    options.c_ispeed = BAUD_RATE;
    options.c_ospeed = BAUD_RATE;

    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 0;

    auto set_ = ioctl(fd, TCSETS2, &options);
    printf("\n\n\n");
    auto get2_ = ioctl(fd, TCGETS2, &options);
    printf("TCGETS2 returned %d\n", get2_);

    return fd;  // Return the file descriptor of the opened UART
}

void printHex(const std::vector<uint8_t>& data) {
    std::cout << std::hex << std::setfill('0');  // Set hex formatting
    for (size_t i = 0; i < data.size(); ++i) {
        std::cout << "0x" << std::setw(2) << static_cast<int>(data[i]);
        if (i < data.size() - 1) {
            std::cout << ", ";
        }
    }
    std::cout << std::dec << std::endl;  // Reset to decimal output
}

uint16_t extract_channel(const uint8_t* data, const int index) {
    const int bit_position = index * 11, byte_index = 3 + bit_position / 8, bit_offset = bit_position % 8;
    uint16_t value = data[byte_index] >> bit_offset | data[byte_index + 1] << (8 - bit_offset);

    if (bit_offset > 5) { value |= data[byte_index + 2] << (16 - bit_offset); } return value & 0x07FF;
}

void updateChannels(const uint8_t* crsf_cata, uint16_t* m_channels) {
    if (crsf_cata[1] == 24) {
        for (int i = 0; i < 16; i++) {
            m_channels[i] = extract_channel(crsf_cata, i);
        }
    }

    for (int i = 0; i<10; i++) {
        std::printf("Channel %i: %i\n", i, m_channels[i]);
    }
}

uint8_t get_crc8(const uint8_t *ptr, const uint8_t length) {
    static const uint8_t crc_table[256] = {
        0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
        0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
        0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
        0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
        0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
        0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
        0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
        0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
        0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
        0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
        0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
        0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
        0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
        0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
        0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
        0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9,
    };

    uint8_t crc = 0;
    for (uint8_t i = 0; i < length; i++) { crc = crc_table[crc ^ *ptr++]; }
    return crc;
}

int main() {
    const char* uartPort = "/dev/ttyAMA10";
    int uart_fd = setup_uart(uartPort);
    if (uart_fd == -1) return 1;
    std::memcpy(m_channels, loc_channels, CRSF_MAX_CHANNEL);

    std::printf("Started\n");

    std::vector<uint8_t> buffer(3, 0x00);
    uint8_t byte = 0x00;

    //static uint8_t receiver_data[CRSF_PACKET_SIZE];
    uint8_t data_count = 0;

    auto interrupt = std::chrono::microseconds(100);
    while (true) {
        if (read(uart_fd, &byte, 1)) {
            if (byte == 0xC8 && buffer[0] == 0x00) { buffer[0] = byte; continue; }
            if (buffer[0] == 0xC8 && buffer[1] == 0x00) { buffer[1] = byte; continue; }
            if (buffer[0] == 0xC8 && buffer[2] == 0x00) {
                buffer[2] = byte;
                while (buffer.size() < buffer[1] + 2) {
                    if (read(uart_fd, &byte, 1)) { buffer.emplace_back(byte); } else { std::this_thread::sleep_for(interrupt); }
                }
                if (get_crc8(&buffer[2], buffer[1] - 1) == buffer[buffer[1] + 1]) {
                    updateChannels(buffer.data(), m_channels);
                }
                buffer.assign(3, 0x00);
            }
        } else {
            std::this_thread::sleep_for(interrupt);
        }
    }

    close(uart_fd);
    return 0;
}