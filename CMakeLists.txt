cmake_minimum_required(VERSION 3.15)

project(ublox_ZEDF9P VERSION 1.0)

find_package(Boost REQUIRED COMPONENTS system regex thread)

add_library(ublox_ZEDF9P
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ublox_ZEDF9P.cpp
)

add_library(ublox::ublox_ZEDF9P ALIAS ublox_ZEDF9P)

target_include_directories(ublox_ZEDF9P 
PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(ublox_ZEDF9P
PUBLIC
  Boost::system
  Boost::regex
  Boost::thread
)