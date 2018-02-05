//
// Created by jo on 2/11/18.
//

#include "memory_read.h"
#include <cstdint>
#include <fcntl.h>
#include <sys/mman.h>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <stdexcept>

void inp_gpio(volatile std::uint32_t *gpio, const std::uint32_t pin) noexcept
{
    *(gpio + ((pin) / 10)) &= ~(7 << (((pin) % 10) * 3));
}

void out_gpio(volatile std::uint32_t *gpio, const std::uint32_t pin) noexcept
{
    *(gpio + ((pin) / 10)) |= (1 << (((pin) % 10) * 3));
}

std::uint32_t get_gpio(volatile std::uint32_t *gpio, const std::uint32_t pin) noexcept
{
    return ((*(gpio + 13) & (1 << pin)) >> pin); // 0 if LOW, (1<<g) if HIGH
}

volatile std::uint32_t *setup_io() noexcept
{
    int mem_fd{0};
    /* open /dev/mem */
    if ((mem_fd = open("/dev/gpiomem", O_RDWR | O_SYNC)) < 0) {
        std::puts("can't open /dev/gpiomem \n");
        return nullptr;
    }

    /* mmap GPIO */
    auto gpio_map = mmap(
            NULL,             //Any adddress in our space will do
            BLOCK_SIZE,       //Map length
            PROT_READ | PROT_WRITE,// Enable reading & writting to mapped memory
            MAP_SHARED,       //Shared with other processes
            mem_fd,           //File to map
            0 //Offset to GPIO peripheral
    );

    close(mem_fd); //No need to keep mem_fd open after mmap

    if (gpio_map == MAP_FAILED) {
        printf("mmap error %d\n", reinterpret_cast<std::intptr_t>(gpio_map));//errno also set!
        return nullptr;
    }

    // Always use volatile pointer!
    return reinterpret_cast<volatile std::uint32_t*>(gpio_map);
}

GPIOPort::GPIOPort(const std::uint32_t port_) : gpio(setup_io()), port(port_)
{
    if(gpio == nullptr) {
        throw std::runtime_error("Can't open /dev/gpiomem");
    }
    inp_gpio(gpio, port);
}

std::uint32_t GPIOPort::read_gpio() noexcept
{
    return get_gpio(gpio,port);
}
