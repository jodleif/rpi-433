cmake_minimum_required(VERSION 3.9)
project(rpi_433)

set(CMAKE_CXX_STANDARD 17)
find_package(Boost REQUIRED COMPONENTS system program_options iostreams)

set(SOURCES src/main.cpp  src/memory_read.cpp src/temperature_sensor.cpp)
set(HEADERS include/config.h include/memory_read.h include/temperature_sensor.h)
include_directories(include)
add_executable(${CMAKE_PROJECT_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(${CMAKE_PROJECT_NAME} ${Boost_LIBRARIES} pthread)
