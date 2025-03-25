# XpressCRSF
**XpressCRSF** is a fast and lightweight `CRSF (Crossfire)` protocol parser for ExpressLRS receivers on Raspberry Pi. 
It operates at the default baud rate of 420,000, using the termios library for efficient serial communication.
XpressCRSF enables real-time parsing of telemetry and control data on Raspberry Pi devices.

## License:
[![License](https://img.shields.io/badge/License-MIT-blue.svg?longCache=true&style=flat)](https://github.com/Vinz1911/xcrsf/blob/master/LICENSE)

## C++ Version:
[![C++23](https://img.shields.io/badge/C++-23-blue.svg?logo=c%2B%2B&style=flat)](https://isocpp.org)

# Import:

```c++
#include "xcrsf/crossfire.h"

auto crossfire = crossfire::XCrossfire("/dev/ttyAMA10");
crossfire.open_port()
```

# Build and Install:

```shell
# build and install libraries
mkdir build
cd build/
cmake ..
make && make install
```

# Usage:

```c++
int main() {
    auto crossfire = crossfire::XCrossfire("/dev/ttyAMA10");
    if (crossfire.open_port()) {
        std::printf("Port opened...\n");
    }

    while (crossfire.is_paired()) {
        const auto channels = crossfire.get_channels();
        for (int i = 0; i < 16; i++) {
            if (channels.front() != 0) { std::printf("Channel %d: %u\n", i, channels[i]); }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return 0;
}
```