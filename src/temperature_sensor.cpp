//
// Created by jo on 2/17/18.
//
#include "temperature_sensor.h"
#include "config.h"
#include <cassert>
#include <bitset>

sensor::SensorFormat::SensorFormat(const std::uint32_t _header, const std::uint32_t _tempc, const std::uint32_t _rest)
{
    header = _header & 0b1111'1111'1111;
    tempc = _tempc & 0b1111'1111'1111;
    rest = _rest & 0b1111'1111'1111;
}

bool sensor::SensorFormat::is_valid()
{
    if (header == sensor::SIGNAL_HEADER) {
        return true;
    }
    debug::debug_print("Not valid signal!");
    if constexpr (debug::debug) {
        std::bitset<32> b(header);
        std::cout << "Header: " << b << '\n';
    }
    return false;
}

sensor::SensorFormat::SensorFormat(const std::array<sensor::Coding,36> &buffer)
{
    header = tempc = rest = 0;
    using sensor::Coding;

    std::int32_t i{0};
    for (const auto &ele : buffer) {
        if(ele == Coding::ONE) {
            if (i < 12) {
                header |= (1 << (11-i));
            } else if (i>=12 && i < 24) {
                tempc |= (1 << (23-i));

            } else if (i>=24 && i < 36){

                rest |= (1 << (35-i));
            }
        }
        ++i;
    }
}
