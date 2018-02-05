//
// Created by jo on 2/11/18.
//

#pragma once
#include <cstdint>

//constexpr static std::uint32_t BCM2708_PERI_BASE {0x3F000000};
//constexpr static std::uint32_t GPIO_BASE {BCM2708_PERI_BASE + 0x200000}; /* GPIO controller */

constexpr static std::uint32_t PAGE_SIZE {4*1024};
constexpr static std::uint32_t BLOCK_SIZE {4*1024};

/*
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)

int  mem_fd;
void *gpio_map;

// I/O access
volatile unsigned *gpio;


// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0

#define GET_GPIO(g) (*(gpio+13)&(1<<g)) // 0 if LOW, (1<<g) if HIGH

#define GPIO_PULL *(gpio+37) // Pull up/pull down
#define GPIO_PULLCLK0 *(gpio+38) // Pull up/pull down clock*/

// GPIO setup Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)

class GPIOPort {
private:
    volatile std::uint32_t* gpio;
    const std::uint32_t port;
public:
    GPIOPort(std::uint32_t port);
    std::uint32_t read_gpio() noexcept;
};


void inp_gpio(volatile std::uint32_t * gpio, const std::uint32_t pin) noexcept;

void out_gpio(volatile std::uint32_t * gpio, const std::uint32_t pin) noexcept;

std::uint32_t get_gpio(volatile std::uint32_t * gpio, const std::uint32_t pin) noexcept;

volatile std::uint32_t * setup_io() noexcept;
