cmake_minimum_required(VERSION 3.9)
project(rpi_433)

set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
set(CMAKE_CXX_STANDARD 17)
#find_package(Boost REQUIRED COMPONENTS system program_options)
find_package( ZLIB REQUIRED )

set(CMAKE_CXX_FLAGS "-Wredundant-decls -Wcast-align -Wmissing-declarations -Wmissing-include-dirs -Wswitch-enum -Wswitch-default -Wextra -Wall -Winvalid-pch -Wredundant-decls -Wformat=2 -Wmissing-format-attribute -Wformat-nonliteral")
find_path(ROCKSDB_ROOT_DIR
        NAMES include/rocksdb/db.h
        )

find_library(ROCKSDB_LIBRARIES
        NAMES rocksdb
        HINTS ${ROCKSDB_ROOT_DIR}/lib/rocksdb
        )

find_path(ROCKSDB_INCLUDE_DIRS
        NAMES rocksdb/db.h
        HINTS ${ROCKSDB_ROOT_DIR}/include
        )
set(SOURCES src/main.cpp  src/memory_read.cpp src/temperature_sensor.cpp)
set(HEADERS include/config.h include/memory_read.h include/temperature_sensor.h)
include_directories(include)
include_directories(SYSTEM ${ROCKSDB_INCLUDE_DIRS} readerwriterqueue)
add_executable(${CMAKE_PROJECT_NAME} ${SOURCES} ${HEADERS})
# enable link time optimization
target_link_libraries(${CMAKE_PROJECT_NAME} pthread atomic ${ROCKSDB_LIBRARIES} ${ZLIB_LIBRARIES})
