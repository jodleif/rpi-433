#include <iostream>
#include <chrono>
#include <vector>
#include <algorithm>
#include <array>
#include <thread>
#include <boost/circular_buffer.hpp>
#include <temperature_sensor.h>
#include <numeric>
#include "memory_read.h"
#include "temperature_sensor.h"
#include "config.h"

struct Reads {
    std::chrono::microseconds delta_start;
    std::int32_t value;

    Reads(std::chrono::microseconds delta, bool v) : delta_start(delta), value(v)
    {
    };
};


using std::chrono::steady_clock;

void print_duration(const Reads r)
{
    std::cout << r.delta_start.count() << ", " << r.value << "\n";
}

template<typename Clock, typename TimePoint>
inline void wait_for(Clock &clk, TimePoint until) noexcept
{

    std::this_thread::sleep_until(until);
    /*while (true) {
        TimePoint np = clk.now();
        if (np >= until) break;
    }*/
}

constexpr std::size_t nof_to_read{5'000'000};

void record()
{
    using std::chrono::steady_clock;
    using tp = steady_clock::time_point;
    using MicroSec = std::chrono::microseconds;
    using namespace std::chrono_literals;
    steady_clock clk;
    std::vector<Reads> durations;
    durations.reserve(nof_to_read);

    auto p = GPIOPort(17);
    //auto p = connection::MappedPort(17);
    //volatile auto i = p.read_some(); // first read is slow
    tp start = clk.now();
    auto st = clk.now();
    for (std::size_t i{0}; i < nof_to_read; ++i) {
        tp now = clk.now();
        MicroSec dur = std::chrono::duration_cast<MicroSec>(now - start);
        durations.emplace_back(Reads(dur, p.read_gpio()));
        wait_for(clk, now + 20us);
    }
    std::cerr << "Done recording, recorded for:"
              << std::chrono::duration_cast<std::chrono::seconds>(clk.now() - st).count() << "seconds\n";
    std::for_each(durations.begin(), durations.end(), print_duration);
    std::cerr << "Average delay: " << (*durations.rbegin()).delta_start.count() / nof_to_read << " microsec\n";
    std::cerr << "Size of Reads: " << sizeof(Reads) << '\n';
}

bool wait_for_val(GPIOPort &port, steady_clock &clk, std::uint32_t val, std::size_t req)
{

    boost::circular_buffer<std::int8_t> buffer(req, -1);
    using namespace std::chrono_literals;
    while (true) {
        auto now = clk.now();
        buffer.push_back(port.read_gpio());
        if (std::all_of(buffer.begin(), buffer.end(), [val](const auto v) {
            return v == val;
        })) {
            return true;
        }
        wait_for(clk, now + 20us);
    }
}

bool wait_for_val(GPIOPort &port, steady_clock &clk, std::uint32_t val) noexcept
{

    using namespace std::chrono_literals;
    while (true) {
        auto now = clk.now();
        volatile auto const read = port.read_gpio();
        if (read == val) {
            return true;
        }
        wait_for(clk, now + 20us);
    }
}

template<typename T>
inline auto median3(T container)
{
    if (container.size() == 3) {
        auto a = container[0];
        auto b = container[1];
        auto c = container[2];
        return std::max(std::min(a, b), std::min(std::max(a, b), c));
    } else if (container.size() == 2) {
        return (container[0] + container[1]) / 2;
    } else {
        return container[0];
    }
}

void print_temp(const boost::circular_buffer<std::int32_t> &stuff)
{
    auto median = median3(stuff);
    std::cout << median / (10.0) << " celcius" << std::endl;
}


void read_temperatures()
{
    steady_clock clk;
    using sensor::Coding;
    std::ios::sync_with_stdio(false);
    auto p = GPIOPort(17);
    using MicroSec = std::chrono::microseconds;
    using namespace std::chrono_literals;
    std::array<Coding, 36> buffer;
    boost::circular_buffer<std::int32_t> results(3);
    std::int32_t counter{0};
    while (true) {
        top:;
        wait_for_val(p, clk, 1);
        wait_for_val(p, clk, 0);
        auto now = clk.now();
        wait_for_val(p, clk, 1);
        auto duration = std::chrono::duration_cast<MicroSec>(clk.now() - now);
        if (duration > 500us && duration < 1100us) {
            buffer[counter] = Coding::ZERO;

        } else if (duration > 1500us && duration < 2500us) {
            buffer[counter] = Coding::ONE;
        } else if (duration > 3000us) {
            counter = 0;
            goto top;
        }
        if (counter >= 35) {
            auto signal = sensor::SensorFormat(buffer);
            if (signal.is_valid()) {
                results.push_back(signal.tempc);
                print_temp(results);
                if(results.size()==3) {
                    std::this_thread::sleep_for(5s);
                }
            } else {
                debug::debug_print("Invalid reading!!");
            }
            counter = 0;
            goto top;
        }
        ++counter;

    }
}

void set_affinity()
{
    std::puts("Setting affinity to CPU 0");
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset);
    auto handle = std::this_thread::get_id();
    auto native_handle = *(reinterpret_cast<pthread_t*>(&handle));
    pthread_setaffinity_np(native_handle, sizeof(cpu_set_t), &cpuset);
    std::puts("Affinity set");
}

void set_thread_prio ()
{
    std::puts("Setting maximum priority for thread");
    auto handle = std::this_thread::get_id();
    auto native_handle = *(reinterpret_cast<pthread_t*>(&handle));

    int policy {0};
    pthread_attr_t thattr;
    pthread_attr_init(&thattr);
    pthread_attr_getschedpolicy(&thattr, &policy);

    auto const max_prio = sched_get_priority_max(policy);

    pthread_setschedprio(native_handle, max_prio);
    pthread_attr_destroy(&thattr);

    std::puts("Max pri set!");
}

int main(int argc, char *argv[])
{
    set_affinity();
    set_thread_prio();

    if (argc >= 2) {
        record();
    } else {
        read_temperatures();
    }
    return 0;
}