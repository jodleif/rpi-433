//
// Created by jo on 2/17/18.
//
#pragma once

#include <cstdint>
#include <array>
namespace sensor {
constexpr static std::size_t SIGNAL_LENGTH {36};
constexpr static std::uint32_t SIGNAL_HEADER {0b0010'0100'1000};
enum class Coding : std::uint8_t {
    ZERO, // ~~ 1800-2050 us
    ONE, // ~~ 800-1000 us
    BREAK, // ~~ 3800us
    OTHER
};

struct SensorFormat {
    std::uint32_t header;
    std::uint32_t tempc;
    std::uint32_t rest;
    SensorFormat(std::uint32_t header, std::uint32_t tempc, std::uint32_t rest);
    SensorFormat(const std::array<Coding,36>& buf);
    bool is_valid();
};

}
