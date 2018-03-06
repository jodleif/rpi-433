//
// Created by jo on 2/5/18.
//
#pragma once
#include <iostream>
namespace debug {
constexpr static bool debug {false};
template <typename T>
void debug_print(T to_print){
    if constexpr (debug) {
        std::cout << to_print << '\n';
    }
}
}

