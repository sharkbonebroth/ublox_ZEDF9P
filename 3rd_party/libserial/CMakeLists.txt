cmake_minimum_required (VERSION 3.2)
project(libserial)
include_guard(GLOBAL)

find_package(Threads REQUIRED)

set(LIBSERIAL_SOURCES
    SerialPort.cpp
    SerialStream.cpp
    SerialStreamBuf.cpp)

add_library(libserial_LIB ${LIBSERIAL_SOURCES})
add_library(dash3rdparty::libserial ALIAS libserial_LIB)
target_compile_features(libserial_LIB PRIVATE cxx_std_17)
target_include_directories(libserial_LIB PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(libserial_LIB Threads::Threads)